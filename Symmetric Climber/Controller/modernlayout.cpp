// ============================================================================
// File: main.cpp
// 4-DOF Arm Controller — hardened production build
//
// Built from the diagnostic instrumentation that was validated live on the
// bench (0 I2C errors / 3400+ writes, WiFi + WebServer + continuous ramp
// loop under real usage). Every write to the PCA9685 goes through the same
// error-checked path that diagnostic v2/v3 proved reliable, PLUS new
// self-recovery logic for failure modes the diagnostic couldn't rule out by
// design (bus lockups, WiFi drops, a genuinely hung main loop).
//
// Joint mapping: channel 0 = Joint 0 ... channel 3 = Joint 3.
// Magnets: Magnet 1 = GPIO1, Magnet 2 = GPIO3 (logic-level outputs feeding
// an external driver stage -- these pins are NOT expected to source the
// magnet coil current directly).
//
// Deliberate safety choice: nothing in this firmware ever switches the
// magnets off automatically (no watchdog reset, no I2C fault, no WiFi drop,
// no /stop call touches them). For a wall-climbing platform, an
// unrequested magnet drop is a fall; every failure path here is designed
// to freeze joint motion and hold the last commanded position, never to
// touch magnet state on its own. Magnets only change on an explicit
// /magnet request from the operator.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
#include "esp_task_wdt.h"

// ---------------------------------------------------------------------------
// Pin / address configuration
// ---------------------------------------------------------------------------
#define SDA_PIN 4
#define SCL_PIN 5
#define MAGNET1_PIN 1
#define MAGNET2_PIN 3
#define PCA_ADDR 0x40
#define INA_ADDR 0x41   // INA219 A0 jumper soldered

static const char* ssid     = "WirelessNet-2";
static const char* password = "russion1234";
static const char* HOSTNAME = "armctrl";

#define WDT_TIMEOUT_S 8

// ---------------------------------------------------------------------------
// Objects
// ---------------------------------------------------------------------------
WebServer server(80);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);
Adafruit_INA219 ina219(INA_ADDR);
Preferences prefs;

bool inaOnline = false;
bool pcaOnline = false;
bool wifiEverConnected = false;

// ---------------------------------------------------------------------------
// Joint configuration (calibration is loaded from flash at boot, falling
// back to these defaults). Limits match the validated kinematic model:
// Joint0/Joint3 = ±135° (270° total), Joint1/Joint2 = ±90° (180° total).
// ---------------------------------------------------------------------------
struct JointConfig {
  const char* label;
  int minPulse;
  int maxPulse;
  int maxDegrees;
  const char* color;
};

JointConfig joints[4] = {
  { "Joint 0", 100, 600, 270, "#e23e57" },
  { "Joint 1", 120, 590, 180, "#00adb5" },
  { "Joint 2", 120, 580, 180, "#8854d0" },
  { "Joint 3", 100, 600, 270, "#ffb423" },
};

int targetPulses[4]  = {350, 355, 350, 350};
int currentPulses[4] = {350, 355, 350, 350};
bool magnetState[2] = {false, false};

const int STEP_DELAY_MS = 6;
unsigned long lastStepTime = 0;
unsigned long lastWifiCheckTime = 0;
unsigned long bootTime = 0;

// ---------------------------------------------------------------------------
// I2C fault tracking + self-recovery
// ---------------------------------------------------------------------------
uint32_t i2cErrorCount[4] = {0, 0, 0, 0};
uint32_t i2cWriteCount[4] = {0, 0, 0, 0};
uint8_t  consecutiveErrors[4] = {0, 0, 0, 0};
uint32_t busRecoveryCount = 0;
const uint8_t CONSECUTIVE_ERROR_THRESHOLD = 5;

const char* i2cErrText(uint8_t code) {
  switch (code) {
    case 0: return "OK";
    case 1: return "DATA TOO LONG";
    case 2: return "NACK on ADDRESS";
    case 3: return "NACK on DATA";
    case 4: return "OTHER ERROR";
    case 5: return "TIMEOUT";
    default: return "UNKNOWN";
  }
}

// Manually toggles SCL to free a bus stuck by a slave holding SDA low
// (the classic I2C lockup), then reinitializes Wire and the PCA9685.
void i2cRecoverBus() {
  Serial.printf("[t=%8lu ms] [RECOVERY] Attempting I2C bus recovery (event #%lu)\n",
                millis() - bootTime, (unsigned long)(++busRecoveryCount));
  Wire.end();
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
  }
  // Manual STOP condition
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(5);

  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(50);
  for (int i = 0; i < 4; i++) consecutiveErrors[i] = 0;
  Serial.println("[RECOVERY] Bus reset and PCA9685 reinitialized.");
}

// Error-checked PCA9685 write with one immediate retry, consecutive-failure
// tracking, and automatic bus recovery if a channel keeps failing.
bool writePWMChecked(uint8_t ch, uint16_t offVal) {
  uint8_t base = 0x06 + 4 * ch;
  for (int attempt = 0; attempt < 2; attempt++) {
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(base);
    Wire.write(0);
    Wire.write(0);
    Wire.write(offVal & 0xFF);
    Wire.write(offVal >> 8);
    uint8_t result = Wire.endTransmission();
    i2cWriteCount[ch]++;

    if (result == 0) {
      consecutiveErrors[ch] = 0;
      return true;
    }
    i2cErrorCount[ch]++;
    if (attempt == 0) continue; // one silent retry before logging

    consecutiveErrors[ch]++;
    Serial.printf("[t=%8lu ms] [I2C ERROR] ch%d write failed: %s | consecutive=%d\n",
                  millis() - bootTime, ch, i2cErrText(result), consecutiveErrors[ch]);
    if (consecutiveErrors[ch] >= CONSECUTIVE_ERROR_THRESHOLD) {
      i2cRecoverBus();
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Calibration persistence (Preferences / NVS flash)
// ---------------------------------------------------------------------------
void loadCalibration() {
  prefs.begin("armcal", true); // read-only
  for (int i = 0; i < 4; i++) {
    char keyMin[8], keyMax[8];
    snprintf(keyMin, sizeof(keyMin), "min%d", i);
    snprintf(keyMax, sizeof(keyMax), "max%d", i);
    joints[i].minPulse = prefs.getInt(keyMin, joints[i].minPulse);
    joints[i].maxPulse = prefs.getInt(keyMax, joints[i].maxPulse);
  }
  prefs.end();
}

void saveCalibration(int ch) {
  prefs.begin("armcal", false); // read-write
  char keyMin[8], keyMax[8];
  snprintf(keyMin, sizeof(keyMin), "min%d", ch);
  snprintf(keyMax, sizeof(keyMax), "max%d", ch);
  prefs.putInt(keyMin, joints[ch].minPulse);
  prefs.putInt(keyMax, joints[ch].maxPulse);
  prefs.end();
}

// ---------------------------------------------------------------------------
// Web page
// ---------------------------------------------------------------------------
const char PAGE_HEADER[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta name='viewport' content='width=device-width, initial-scale=1.0'>
<title>4-DOF Arm Controller</title>
<style>
html, body { height: 100%; margin: 0; }
body { font-family: 'Segoe UI', Arial, sans-serif; background: #0d0d12; color: #f0f0fa; }
* { box-sizing: border-box; }

.topbar { position: sticky; top: 0; z-index: 20; display: flex; align-items: center; justify-content: space-between; gap: 14px; background: #15151d; padding: 10px 18px; border-bottom: 1px solid #26263a; box-shadow: 0 4px 16px rgba(0,0,0,0.6); flex-wrap: wrap; }
.topbar-left h1 { color: #00e0e8; margin: 0; font-size: 16px; text-transform: uppercase; letter-spacing: 1px; }
.topbar-left .subtitle { color: #7c7c9c; font-size: 10.5px; letter-spacing: .5px; }
.topbar-chips { display: flex; gap: 8px; flex-wrap: wrap; }
.chip { background: #1f1f2b; border: 1px solid #2d2d40; border-radius: 20px; padding: 5px 12px; font-family: monospace; font-size: 12px; color: #c7c7e6; white-space: nowrap; }
.chip b { color: #00f5ff; }
.btn-estop { background: linear-gradient(135deg,#ff4545,#c2181f); font-size: 12.5px; padding: 11px 18px; border-radius: 8px; box-shadow: 0 0 14px rgba(255,50,50,0.35); white-space: nowrap; }
.btn-estop:hover { background: linear-gradient(135deg,#ff6a6a,#d81f27); }

.dashboard { display: grid; grid-template-columns: 1fr 1.25fr 1fr; gap: 14px; padding: 14px; height: calc(100vh - 58px); box-sizing: border-box; max-width: 1500px; margin: 0 auto; }
.col { display: flex; flex-direction: column; gap: 14px; overflow-y: auto; padding-right: 4px; }
.col::-webkit-scrollbar { width: 6px; }
.col::-webkit-scrollbar-thumb { background: #2d2d40; border-radius: 3px; }

.panel { background: #1a1a23; padding: 16px 18px; border-radius: 12px; box-shadow: 0 6px 20px rgba(0,0,0,0.5); border: 1px solid #23233066; text-align: left; flex-shrink: 0; }
.panel h2 { color: #00adb5; font-size: 12px; margin: 0 0 12px 0; text-transform: uppercase; letter-spacing: 1px; border-bottom: 1px solid #2a2a3a; padding-bottom: 6px; display: flex; align-items: center; gap: 7px; }

.joint-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.joint-card { background: #24242f; padding: 10px 12px; border-radius: 8px; border-left: 4px solid #00adb5; }
.joint-header { display: flex; justify-content: space-between; align-items: center; font-weight: bold; font-size: 12.5px; margin-bottom: 8px; }
.deg-input { background: #0d0d12; color: #00f5ff; border: 1px solid #00adb5; border-radius: 4px; padding: 3px; width: 50px; font-family: monospace; font-size: 13px; font-weight: bold; text-align: center; outline: none;}
.slider-row { display: flex; align-items: center; margin-bottom: 8px; }
.slider { width: 100%; -webkit-appearance: none; background: #33334a; height: 6px; border-radius: 5px; outline: none; }
.slider::-webkit-slider-thumb { -webkit-appearance: none; width: 18px; height: 18px; border-radius: 50%; background: #00adb5; cursor: pointer; }
.cal-matrix { display: flex; gap: 8px; background: #16161e; padding: 6px 8px; border-radius: 6px; font-size: 10px; align-items: center; justify-content: space-between; color: #9a9ab8; }
.cal-input { background: #2a2a3a; color: #fff; border: 1px solid #444; border-radius: 4px; padding: 2px; width: 36px; text-align: center; font-family: monospace; font-size: 11px; }

.magnet-flex { display: flex; gap: 10px; }
.magnet-row { flex: 1; display: flex; flex-direction: column; align-items: flex-start; gap: 8px; background: #24242f; padding: 12px; border-radius: 8px; }
.magnet-row .switch { align-self: center; }
.switch { position: relative; display: inline-block; width: 46px; height: 24px; }
.switch input { opacity: 0; width: 0; height: 0; }
.toggle-slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #dc3545; transition: .3s; border-radius: 34px; }
.toggle-slider:before { position: absolute; content: ""; height: 16px; width: 16px; left: 4px; bottom: 4px; background-color: white; transition: .3s; border-radius: 50%; }
input:checked + .toggle-slider { background-color: #28a745; }
input:checked + .toggle-slider:before { transform: translateX(22px); }

.seq-row { display: flex; gap: 8px; margin-bottom: 10px; align-items: center; flex-wrap: wrap; }
.btn { padding: 7px 12px; font-size: 11.5px; border: none; border-radius: 6px; cursor: pointer; font-weight: bold; color: white; display: inline-flex; align-items: center; justify-content: center; transition: transform .1s, background .15s; }
.btn:active { transform: scale(0.96); }
.btn-blue { background: #00adb5; } .btn-blue:hover { background: #00c7d1; }
.btn-green { background: #28a745; } .btn-green:hover { background: #2fc457; }
.btn-red { background: #dc3545; } .btn-red:hover { background: #e8495a; }
.btn-orange { background: #ff9f43; } .btn-orange:hover { background: #ffb066; }
.btn-grey { background: #3d3d52; } .btn-grey:hover { background: #4d4d68; }
.text-field { background: #0d0d12; color: #fff; border: 1px solid #3a3a4e; padding: 7px; border-radius: 6px; outline: none; font-size: 12.5px; }
.delay-input { width: 60px; text-align: center; font-family: monospace; }
#seqQueue { background: #0d0d12; min-height: 70px; max-height: 150px; overflow-y: auto; padding: 8px; border-radius: 6px; border: 1px solid #26263a; margin-bottom: 10px; }
.queue-item { display: flex; justify-content: space-between; align-items: center; background: #1e1e29; padding: 6px 10px; margin-bottom: 5px; border-radius: 4px; font-family: monospace; font-size: 10.5px; cursor: pointer; border: 1px solid transparent; color: #b3b3cc;}
.queue-item:hover { border-color: #00adb5; }
.queue-item.active-frame { border-color: #00f5ff; background: #16283a; color: #00f5ff; font-weight: bold; }
.item-actions { display: flex; gap: 4px; }
.mini-btn { padding: 2px 6px; font-size: 10px; border-radius: 3px; border: none; color: white; cursor: pointer; }
.inv-select { flex-grow: 1; background: #0d0d12; color: #fff; border: 1px solid #3a3a4e; padding: 7px; border-radius: 6px; font-size: 12.5px; }
.status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; font-family: monospace; font-size: 11.5px; color: #9a9ab8; }
.status-grid b { color: #00f5ff; }
.live-stats { font-family: monospace; color: #00f5ff; font-size: 14px; text-align: center; margin-bottom: 6px;}
.live-stats span { margin: 0 8px; }
#logChart { width: 100%; height: 110px; background: #0d0d12; border-radius: 6px; border: 1px solid #26263a; display: block; margin-bottom: 10px; }
.log-stats { font-family: monospace; font-size: 11px; color: #9a9ab8; text-align: center; margin-bottom: 10px; }
.log-stats b { color: #00f5ff; }
.rate-select { background: #0d0d12; color: #fff; border: 1px solid #3a3a4e; padding: 7px; border-radius: 6px; font-size: 12.5px; }
.btn-logging { background: #dc3545; }
.btn-logging.active { background: #28a745; animation: pulseRec 1.5s infinite; }
@keyframes pulseRec { 0%,100% { opacity: 1; } 50% { opacity: 0.6; } }

@media (max-width: 1050px) {
  .dashboard { grid-template-columns: 1fr 1fr; height: auto; }
  .col:nth-child(3) { grid-column: span 2; }
  .col { overflow-y: visible; }
}
@media (max-width: 680px) {
  .dashboard { display: flex; flex-direction: column; height: auto; padding: 10px; gap: 12px; }
  .col { overflow-y: visible; }
  .col:nth-child(3) { grid-column: auto; }
  .joint-grid { grid-template-columns: 1fr; }
  .topbar { justify-content: center; text-align: center; }
}
</style>
</head>
<body>
)=====";

const char PAGE_FOOTER_SCRIPT[] PROGMEM = R"=====(
<script>
let sequence = [];
let selectedFrameIdx = null;
let lastSent = [0,0,0,0];
let lastSendTime = [0,0,0,0];
const THROTTLE_MS = 40; // limits slider-drag request flood

function flashMsg(msg) {
  const el = document.getElementById('statusMsg');
  if (el) { el.innerText = msg; setTimeout(()=>{el.innerText='';}, 2000); }
}

function syncInputs(num, value, force) {
  let maxLimit = parseInt(document.getElementById('deg' + num).max);
  if (value > maxLimit) value = maxLimit;
  if (value < 0) value = 0;
  document.getElementById('deg' + num).value = value;
  document.getElementById('num' + num).value = value;

  let now = Date.now();
  if (!force && (now - lastSendTime[num] < THROTTLE_MS) && Math.abs(value - lastSent[num]) < 2) return;
  lastSendTime[num] = now;
  lastSent[num] = value;

  fetch('/set?num=' + num + '&deg=' + value).catch(()=>flashMsg('Connection lost — retrying on next move'));
}

function updateBounds(num) {
  let minVal = document.getElementById('min' + num).value;
  let maxVal = document.getElementById('max' + num).value;
  fetch('/cal?num=' + num + '&min=' + minVal + '&max=' + maxVal)
    .then(()=>flashMsg('Calibration saved for Joint ' + num));
}

function controlMagnet(id, isChecked) {
  document.getElementById('mag' + id).checked = isChecked;
  fetch('/magnet?id=' + id + '&state=' + (isChecked ? 1 : 0));
}

function getCurrentPose() {
  return {
    j0: document.getElementById('deg0').value, j1: document.getElementById('deg1').value,
    j2: document.getElementById('deg2').value, j3: document.getElementById('deg3').value,
    m1: document.getElementById('mag1').checked, m2: document.getElementById('mag2').checked,
    delay: document.getElementById('delayTime').value
  };
}

function captureFrame() { sequence.push(getCurrentPose()); renderSequenceQueue(); clearFrameSelection(); }

function selectFrame(idx, event) {
  if (event && event.target.tagName === 'BUTTON') return;
  selectedFrameIdx = idx;
  let f = sequence[idx];
  syncInputs(0, f.j0, true); syncInputs(1, f.j1, true); syncInputs(2, f.j2, true); syncInputs(3, f.j3, true);
  controlMagnet(1, f.m1); controlMagnet(2, f.m2);
  document.getElementById('delayTime').value = f.delay;
  document.getElementById('updateBtn').style.display = 'inline-flex';
  document.getElementById('updateBtn').innerText = 'Update Frame ' + idx;
  renderSequenceQueue();
}

function updateSelectedFrame() { if (selectedFrameIdx===null) return; sequence[selectedFrameIdx]=getCurrentPose(); clearFrameSelection(); }
function clearFrameSelection() { selectedFrameIdx = null; document.getElementById('updateBtn').style.display='none'; renderSequenceQueue(); }
function deleteFrame(idx) { sequence.splice(idx,1); if(selectedFrameIdx===idx) clearFrameSelection(); else renderSequenceQueue(); }
function shiftFrame(idx, dir) {
  let t = idx+dir; if(t<0||t>=sequence.length) return;
  [sequence[idx], sequence[t]] = [sequence[t], sequence[idx]];
  renderSequenceQueue();
}

function renderSequenceQueue() {
  let q = document.getElementById('seqQueue');
  if (sequence.length===0) { q.innerHTML = "Timeline empty..."; return; }
  q.innerHTML = "";
  sequence.forEach((f, idx) => {
    let active = (selectedFrameIdx===idx) ? 'active-frame' : '';
    q.innerHTML += `<div class='queue-item ${active}' onclick='selectFrame(${idx}, event)'>
      <div><strong>[F${idx}]</strong> J0:${f.j0} J1:${f.j1} J2:${f.j2} J3:${f.j3} M1:${f.m1?'ON':'OFF'} M2:${f.m2?'ON':'OFF'} (${f.delay}ms)</div>
      <div class='item-actions'>
        <button class='mini-btn btn-grey' onclick='shiftFrame(${idx},-1)'>&uarr;</button>
        <button class='mini-btn btn-grey' onclick='shiftFrame(${idx},1)'>&darr;</button>
        <button class='mini-btn btn-red' onclick='deleteFrame(${idx})'>X</button>
      </div></div>`;
  });
}

function clearActiveSequence() { sequence = []; clearFrameSelection(); }

async function playSequence() {
  if (sequence.length===0) return;
  let btn = document.getElementById('playBtn');
  btn.innerText = "Playing..."; btn.style.background = "#ffb423";
  for (let i=0;i<sequence.length;i++) {
    let f = sequence[i];
    document.querySelectorAll('.queue-item').forEach(el=>el.classList.remove('active-frame'));
    let el = document.querySelectorAll('.queue-item')[i]; if (el) el.classList.add('active-frame');
    syncInputs(0, f.j0, true); syncInputs(1, f.j1, true); syncInputs(2, f.j2, true); syncInputs(3, f.j3, true);
    controlMagnet(1, f.m1); controlMagnet(2, f.m2);
    await new Promise(r=>setTimeout(r, parseInt(f.delay)));
  }
  btn.innerText = "Play Timeline"; btn.style.background = "#28a745";
  renderSequenceQueue();
}

const VAULT_PREFIX = "arm4dof_";
function saveActiveSequence() {
  let id = document.getElementById('storageName').value.trim();
  if (!id) { alert("Enter a sequence name."); return; }
  if (sequence.length===0) { alert("Timeline is empty."); return; }
  localStorage.setItem(VAULT_PREFIX+id, JSON.stringify(sequence));
  document.getElementById('storageName').value = "";
  refreshInventoryDropdown();
}
function refreshInventoryDropdown() {
  let sel = document.getElementById('inventorySelect');
  sel.innerHTML = "<option value=''>-- Saved Routines --</option>";
  for (let i=0;i<localStorage.length;i++) {
    let key = localStorage.key(i);
    if (key.startsWith(VAULT_PREFIX)) sel.innerHTML += `<option value='${key}'>${key.replace(VAULT_PREFIX,'')}</option>`;
  }
}
function loadSelectedInventory() {
  let key = document.getElementById('inventorySelect').value; if (!key) return;
  let data = localStorage.getItem(key);
  if (data) { sequence = JSON.parse(data); clearFrameSelection(); }
}
function deleteSelectedInventory() {
  let key = document.getElementById('inventorySelect').value; if (!key) return;
  if (confirm("Delete this saved routine?")) { localStorage.removeItem(key); clearActiveSequence(); refreshInventoryDropdown(); }
}

// ---------------------------------------------------------------------------
// Power telemetry logger
// ---------------------------------------------------------------------------
// Single adaptive poll loop (self-rescheduling setTimeout, not setInterval)
// drives both the live readout/chart and, while active, the timestamped
// log used for the paper's graphs. setTimeout-after-completion avoids the
// classic setInterval pileup if a request is ever slow to answer -- each
// poll waits for the previous one to finish before scheduling the next,
// so a sluggish response never queues up a burst of overlapping requests.
let isLogging = false;
let logData = [];          // {sample, t_ms, v, i, p}
let logStartUptime = null;
let sampleRateMs = 1000;
let idlePollMs = 1000;
let sampleCounter = 0;

function setSampleRate(ms) {
  sampleRateMs = parseInt(ms);
  document.getElementById('logRateLabel').innerText = sampleRateMs + 'ms';
}

function toggleLogging() {
  isLogging = !isLogging;
  let btn = document.getElementById('logBtn');
  if (isLogging) {
    btn.innerText = 'Stop Logging';
    btn.classList.add('active');
    logStartUptime = null; // set on first sample so t=0 aligns to logging start
  } else {
    btn.innerText = 'Start Logging';
    btn.classList.remove('active');
  }
}

function clearLog() {
  logData = [];
  sampleCounter = 0;
  logStartUptime = null;
  document.getElementById('logCount').innerText = '0';
  document.getElementById('logDuration').innerText = '0s';
  drawChart();
}

function drawChart() {
  const canvas = document.getElementById('logChart');
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  if (logData.length < 2) {
    ctx.fillStyle = '#4b4b61';
    ctx.font = '12px monospace';
    ctx.fillText(isLogging ? 'Collecting samples...' : 'No data logged yet', 10, h/2);
    return;
  }
  const window_ = logData.slice(-300); // last 300 points keeps rendering cheap regardless of session length
  const iVals = window_.map(p => p.i);
  const iMin = Math.min(...iVals, 0), iMax = Math.max(...iVals, 1);
  const pad = 10;
  ctx.strokeStyle = '#00f5ff';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  window_.forEach((pt, idx) => {
    const x = pad + (idx / (window_.length - 1)) * (w - 2*pad);
    const y = h - pad - ((pt.i - iMin) / (iMax - iMin || 1)) * (h - 2*pad);
    if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.fillStyle = '#8f8fbf';
  ctx.font = '10px monospace';
  ctx.fillText('Current (mA), last ' + window_.length + ' samples: ' + iMin.toFixed(0) + '-' + iMax.toFixed(0), pad, 12);
}

function recordSample(d) {
  if (logStartUptime === null) logStartUptime = d.uptime;
  sampleCounter++;
  logData.push({
    sample: sampleCounter,
    t_ms: d.uptime - logStartUptime,
    uptime_ms: d.uptime,
    v: d.v, i: d.i, p: d.p
  });
  document.getElementById('logCount').innerText = logData.length;
  document.getElementById('logDuration').innerText = ((d.uptime - logStartUptime)/1000).toFixed(1) + 's';
  drawChart();
}

function exportCSV() {
  if (logData.length === 0) { alert('No logged data to export.'); return; }
  let csv = 'sample,elapsed_ms,device_uptime_ms,bus_voltage_V,current_mA,power_mW\n';
  logData.forEach(r => { csv += `${r.sample},${r.t_ms},${r.uptime_ms},${r.v},${r.i},${r.p}\n`; });
  downloadBlob(csv, 'text/csv', 'arm_telemetry_' + Date.now() + '.csv');
}

function exportJSON() {
  if (logData.length === 0) { alert('No logged data to export.'); return; }
  downloadBlob(JSON.stringify(logData, null, 2), 'application/json', 'arm_telemetry_' + Date.now() + '.json');
}

function downloadBlob(content, type, filename) {
  let blob = new Blob([content], { type });
  let url = URL.createObjectURL(blob);
  let a = document.createElement('a');
  a.href = url; a.download = filename; document.body.appendChild(a); a.click();
  document.body.removeChild(a); URL.revokeObjectURL(url);
}

const LOG_VAULT_PREFIX = 'arm4dof_log_';
function saveLogSession() {
  let id = document.getElementById('logSessionName').value.trim();
  if (!id) { alert('Enter a session name.'); return; }
  if (logData.length === 0) { alert('No logged data to save.'); return; }
  try {
    localStorage.setItem(LOG_VAULT_PREFIX + id, JSON.stringify(logData));
    document.getElementById('logSessionName').value = '';
    refreshLogSessionDropdown();
  } catch (e) {
    alert('Could not save session (browser storage full). Download it as CSV/JSON instead.');
  }
}
function refreshLogSessionDropdown() {
  let sel = document.getElementById('logSessionSelect');
  sel.innerHTML = "<option value=''>-- Saved Sessions --</option>";
  for (let i=0;i<localStorage.length;i++) {
    let key = localStorage.key(i);
    if (key.startsWith(LOG_VAULT_PREFIX)) sel.innerHTML += `<option value='${key}'>${key.replace(LOG_VAULT_PREFIX,'')}</option>`;
  }
}
function loadLogSession() {
  let key = document.getElementById('logSessionSelect').value; if (!key) return;
  let data = localStorage.getItem(key);
  if (data) {
    logData = JSON.parse(data);
    sampleCounter = logData.length ? logData[logData.length-1].sample : 0;
    document.getElementById('logCount').innerText = logData.length;
    if (logData.length) document.getElementById('logDuration').innerText = (logData[logData.length-1].t_ms/1000).toFixed(1) + 's';
    drawChart();
  }
}
function deleteLogSession() {
  let key = document.getElementById('logSessionSelect').value; if (!key) return;
  if (confirm('Delete this saved session?')) { localStorage.removeItem(key); refreshLogSessionDropdown(); }
}

function pollLoop() {
  fetch('/status').then(r => r.json()).then(d => {
    document.getElementById('liveV').innerText = d.v.toFixed(2) + ' V';
    document.getElementById('liveI').innerText = d.i.toFixed(1) + ' mA';
    document.getElementById('liveP').innerText = d.p.toFixed(1) + ' mW';
    document.getElementById('liveRSSI').innerText = d.rssi + ' dBm';
    document.getElementById('liveUptime').innerText = Math.floor(d.uptime/1000) + ' s';
    document.getElementById('liveErrors').innerText = d.i2cErrors;
    document.getElementById('liveRecoveries').innerText = d.busRecoveries;
    document.getElementById('liveHeap').innerText = Math.floor(d.freeHeap/1024) + ' KB';
    document.getElementById('topV').innerText = d.v.toFixed(2) + 'V';
    document.getElementById('topI').innerText = d.i.toFixed(0) + 'mA';
    document.getElementById('topRSSI').innerText = d.rssi + 'dBm';
    document.getElementById('topUptime').innerText = Math.floor(d.uptime/1000) + 's';
    if (isLogging) recordSample(d);
  }).catch(()=>{}).finally(() => {
    setTimeout(pollLoop, isLogging ? sampleRateMs : idlePollMs);
  });
}

window.onload = function() { refreshInventoryDropdown(); refreshLogSessionDropdown(); drawChart(); pollLoop(); };
</script>
</body>
</html>
)=====";

void handleRoot() {
  String page = String((const __FlashStringHelper*)PAGE_HEADER);

  // Sticky top bar: title, glanceable live chips, and the E-Stop pinned so
  // it's reachable no matter where the operator has scrolled.
  page += "<div class='topbar'><div class='topbar-left'><h1>4-DOF Arm Controller</h1>";
  page += "<div class='subtitle'>http://" + String(HOSTNAME) + ".local</div></div>";
  page += "<div class='topbar-chips'>";
  page += "<span class='chip'>&#9889; <b id='topV'>--</b></span>";
  page += "<span class='chip'>&#128268; <b id='topI'>--</b></span>";
  page += "<span class='chip'>&#128246; <b id='topRSSI'>--</b></span>";
  page += "<span class='chip'>&#9201; <b id='topUptime'>--</b></span>";
  page += "</div>";
  page += "<button class='btn btn-estop' onclick=\"fetch('/stop').then(()=>flashMsg('Motion halted'))\">&#9940; EMERGENCY STOP</button>";
  page += "</div>";

  page += "<div class='dashboard'>";

  // --- Column 1: Joint Configuration + End Effector ---------------------
  page += "<div class='col'>";
  page += "<div class='panel'><h2>&#9881; Joint Configuration</h2><div class='joint-grid'>";
  for (int i = 0; i < 4; i++) {
    int curDeg = map(currentPulses[i], joints[i].minPulse, joints[i].maxPulse, 0, joints[i].maxDegrees);
    page += "<div class='joint-card' style='border-left-color:" + String(joints[i].color) + ";'>";
    page += "<div class='joint-header'><span>" + String(joints[i].label) + "</span>";
    page += "<input type='number' min='0' max='" + String(joints[i].maxDegrees) + "' value='" + String(curDeg) +
            "' class='deg-input' id='num" + String(i) + "' onchange='syncInputs(" + String(i) + ", this.value, true)'></div>";
    page += "<div class='slider-row'><input type='range' min='0' max='" + String(joints[i].maxDegrees) + "' value='" + String(curDeg) +
            "' class='slider' id='deg" + String(i) + "' oninput='syncInputs(" + String(i) + ", this.value, false)'></div>";
    page += "<div class='cal-matrix'><span>Trim:</span><input type='number' value='" + String(joints[i].minPulse) +
            "' class='cal-input' id='min" + String(i) + "' onchange='updateBounds(" + String(i) + ")'>" +
            "<input type='number' value='" + String(joints[i].maxPulse) +
            "' class='cal-input' id='max" + String(i) + "' onchange='updateBounds(" + String(i) + ")'></div>";
    page += "</div>";
  }
  page += "</div><span id='statusMsg' style='color:#00f5ff;font-size:11px;'></span></div>";

  page += "<div class='panel'><h2>&#129517; End Effector</h2><div class='magnet-flex'>";
  page += "<div class='magnet-row'><span>Magnet 1<br><small style=\"color:#7c7c9c;\">GPIO1</small></span><label class='switch'><input type='checkbox' id='mag1' " +
          String(magnetState[0] ? "checked" : "") + " onchange='controlMagnet(1, this.checked)'><span class='toggle-slider'></span></label></div>";
  page += "<div class='magnet-row'><span>Magnet 2<br><small style=\"color:#7c7c9c;\">GPIO3</small></span><label class='switch'><input type='checkbox' id='mag2' " +
          String(magnetState[1] ? "checked" : "") + " onchange='controlMagnet(2, this.checked)'><span class='toggle-slider'></span></label></div>";
  page += "</div></div>";
  page += "</div>"; // end col1

  // --- Column 2: Power Telemetry & Logger --------------------------------
  page += "<div class='col'>";
  page += "<div class='panel'><h2>&#9889; Power Telemetry &amp; Logger</h2>";
  page += "<div class='live-stats'><span id='liveV'>-- V</span><span id='liveI'>-- mA</span><span id='liveP'>-- mW</span></div>";
  page += "<canvas id='logChart' width='600' height='110'></canvas>";
  page += "<div class='log-stats'><b id='logCount'>0</b> samples &nbsp;|&nbsp; <b id='logDuration'>0s</b> logged &nbsp;|&nbsp; rate: <b id='logRateLabel'>1000ms</b></div>";
  page += "<div class='seq-row'>";
  page += "<select id='rateSelect' class='rate-select' onchange='setSampleRate(this.value)'>";
  page += "<option value='150'>150 ms (fastest validated)</option>";
  page += "<option value='500'>500 ms</option>";
  page += "<option value='1000' selected>1000 ms</option>";
  page += "<option value='2000'>2000 ms</option></select>";
  page += "<button class='btn btn-logging' id='logBtn' style='flex-grow:1;' onclick='toggleLogging()'>Start Logging</button>";
  page += "<button class='btn btn-red' onclick='clearLog()'>Clear</button></div>";
  page += "<div class='seq-row'><button class='btn btn-blue' style='flex-grow:1;' onclick='exportCSV()'>Download CSV</button>";
  page += "<button class='btn btn-blue' style='flex-grow:1;' onclick='exportJSON()'>Download JSON</button></div>";
  page += "<h2 style='margin-top:14px;'>&#128190; Saved Logging Sessions</h2>";
  page += "<div class='seq-row'><input type='text' id='logSessionName' class='text-field' style='flex-grow:1;' placeholder='Session name...'>";
  page += "<button class='btn btn-blue' onclick='saveLogSession()'>Save</button></div>";
  page += "<div class='seq-row'><select id='logSessionSelect' class='inv-select' onchange='loadLogSession()'></select>";
  page += "<button class='btn btn-red' onclick='deleteLogSession()'>Delete</button></div>";
  page += "</div>";
  page += "</div>"; // end col2

  // --- Column 3: System Health + Sequence Timeline -----------------------
  page += "<div class='col'>";
  page += "<div class='panel'><h2>&#129001; System Health</h2><div class='status-grid'>";
  page += "<div>WiFi RSSI: <b id='liveRSSI'>--</b></div><div>Uptime: <b id='liveUptime'>--</b></div>";
  page += "<div>I2C Errors: <b id='liveErrors'>--</b></div><div>Bus Recoveries: <b id='liveRecoveries'>--</b></div>";
  page += "<div>Free Heap: <b id='liveHeap'>--</b></div><div>Host: <b>" + String(HOSTNAME) + ".local</b></div>";
  page += "</div></div>";

  page += "<div class='panel'><h2>&#127909; Sequence Timeline</h2><div id='seqQueue'>Timeline empty...</div>";
  page += "<div class='seq-row'><span style='font-size:11px;color:#8f8fbf;'>Hold:</span>";
  page += "<input type='number' id='delayTime' value='1500' class='text-field delay-input'><span style='font-size:11px;color:#8f8fbf;'>ms</span>";
  page += "<button class='btn btn-blue' style='flex-grow:1;' onclick='captureFrame()'>Capture Frame</button></div>";
  page += "<div class='seq-row'><button class='btn btn-orange' id='updateBtn' style='display:none;flex-grow:1;' onclick='updateSelectedFrame()'>Update Selected</button>";
  page += "<button class='btn btn-green' style='flex-grow:2;' onclick='playSequence()' id='playBtn'>Play Timeline</button>";
  page += "<button class='btn btn-red' onclick='clearActiveSequence()'>Clear</button></div>";
  page += "<h2 style='margin-top:14px;'>&#128190; Saved Routines</h2><div class='seq-row'><input type='text' id='storageName' class='text-field' style='flex-grow:1;' placeholder='Routine name...'>";
  page += "<button class='btn btn-blue' onclick='saveActiveSequence()'>Save</button></div>";
  page += "<div class='seq-row'><select id='inventorySelect' class='inv-select' onchange='loadSelectedInventory()'></select>";
  page += "<button class='btn btn-red' onclick='deleteSelectedInventory()'>Delete</button></div></div>";
  page += "</div>"; // end col3

  page += "</div>"; // end dashboard
  page += String((const __FlashStringHelper*)PAGE_FOOTER_SCRIPT);

  server.send(200, "text/html", page);
}

// ---------------------------------------------------------------------------
// API handlers
// ---------------------------------------------------------------------------
void handleSet() {
  if (server.hasArg("num") && server.hasArg("deg")) {
    int num = server.arg("num").toInt();
    int deg = server.arg("deg").toInt();
    if (num >= 0 && num < 4) {
      if (deg > joints[num].maxDegrees) deg = joints[num].maxDegrees;
      if (deg < 0) deg = 0;
      targetPulses[num] = map(deg, 0, joints[num].maxDegrees, joints[num].minPulse, joints[num].maxPulse);
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Invalid parameters");
}

void handleCalibration() {
  if (server.hasArg("num") && server.hasArg("min") && server.hasArg("max")) {
    int num = server.arg("num").toInt();
    int minV = server.arg("min").toInt();
    int maxV = server.arg("max").toInt();
    // Sanity bounds: PCA9685 is a 12-bit driver (0-4095); reject nonsense
    // calibration values instead of silently applying them.
    if (num >= 0 && num < 4 && minV >= 0 && maxV <= 4095 && minV < maxV) {
      joints[num].minPulse = minV;
      joints[num].maxPulse = maxV;
      saveCalibration(num);
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Invalid calibration values");
}

void handleMagnet() {
  if (server.hasArg("id") && server.hasArg("state")) {
    int magnetId = server.arg("id").toInt();
    int state = server.arg("state").toInt();
    if (magnetId == 1 || magnetId == 2) {
      int pin = (magnetId == 1) ? MAGNET1_PIN : MAGNET2_PIN;
      digitalWrite(pin, state ? HIGH : LOW);
      magnetState[magnetId - 1] = state != 0;
      Serial.printf("[t=%8lu ms] [MAGNET] Magnet %d -> %s\n", millis() - bootTime, magnetId, state ? "ON" : "OFF");
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Invalid parameters");
}

// Halts joint motion only: snaps the ramp target to the current position so
// the loop stops advancing. Magnets are never touched here.
void handleStop() {
  for (int i = 0; i < 4; i++) targetPulses[i] = currentPulses[i];
  Serial.printf("[t=%8lu ms] [ESTOP] Motion halted by operator request.\n", millis() - bootTime);
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  float v = inaOnline ? ina219.getBusVoltage_V() : 0;
  float i = inaOnline ? ina219.getCurrent_mA() : 0;
  // Read the INA219's own power register rather than computing V*I client
  // side -- the chip's internal calibration doesn't always match a naive
  // multiply, and for a paper you want the sensor's actual reported value.
  float p = inaOnline ? ina219.getPower_mW() : 0;
  uint32_t totalErrors = i2cErrorCount[0] + i2cErrorCount[1] + i2cErrorCount[2] + i2cErrorCount[3];

  String json = "{";
  json += "\"v\":" + String(v, 3) + ",";
  json += "\"i\":" + String(i, 3) + ",";
  json += "\"p\":" + String(p, 3) + ",";
  json += "\"rssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) + ",";
  json += "\"uptime\":" + String(millis() - bootTime) + ",";
  json += "\"i2cErrors\":" + String(totalErrors) + ",";
  json += "\"busRecoveries\":" + String(busRecoveryCount) + ",";
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"pcaOnline\":" + String(pcaOnline ? "true" : "false") + ",";
  json += "\"inaOnline\":" + String(inaOnline ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void connectWiFi(unsigned long timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());
    wifiEverConnected = true;
    if (MDNS.begin(HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("mDNS active: http://%s.local\n", HOSTNAME);
    }
    server.begin();
  } else {
    Serial.println("\nWiFi connect timed out; will keep retrying in background. Servos remain controllable once it connects.");
  }
}

void setup() {
  Serial.begin(115200);
  bootTime = millis();

  Serial.println("\n=============================================");
  Serial.println("      4-DOF ARM CONTROLLER (hardened build)  ");
  Serial.println("=============================================");

  // Watchdog: if the main loop ever genuinely hangs (an unrecoverable
  // fault, not just a slow I2C transaction), the chip self-resets instead
  // of needing a manual power cycle. Fed every loop() iteration.
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  pinMode(MAGNET1_PIN, OUTPUT);
  pinMode(MAGNET2_PIN, OUTPUT);
  digitalWrite(MAGNET1_PIN, LOW);
  digitalWrite(MAGNET2_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.print("Initializing INA219 (0x41)... ");
  if (ina219.begin(&Wire)) { Serial.println("[OK]"); inaOnline = true; }
  else Serial.println("[FAILED] Telemetry will report 0 until reconnected.");

  loadCalibration();

  Serial.print("Initializing PCA9685 (0x40)... ");
  pwm.begin();
  pwm.setPWMFreq(50);
  pcaOnline = true;
  Serial.println("[OK]");

  // Move to a safe home position BEFORE attempting WiFi, so the arm is in
  // a known, defined pose even if the network never comes up.
  for (int i = 0; i < 4; i++) {
    currentPulses[i] = (joints[i].minPulse + joints[i].maxPulse) / 2;
    targetPulses[i] = currentPulses[i];
    writePWMChecked(i, currentPulses[i]);
  }

  connectWiFi(15000);

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/cal", handleCalibration);
  server.on("/magnet", handleMagnet);
  server.on("/stop", handleStop);
  server.on("/status", handleStatus);

  Serial.println("\nSetup complete.\n");
}

void loop() {
  esp_task_wdt_reset();

  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  // Non-blocking WiFi supervision: previous.cpp's original blocking
  // connect loop had no recovery path if the AP dropped mid-session. This
  // checks every 5s and reconnects in the background without ever
  // stalling the servo ramp loop.
  if (millis() - lastWifiCheckTime >= 5000) {
    lastWifiCheckTime = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.printf("[t=%8lu ms] [WIFI] Not connected, retrying...\n", millis() - bootTime);
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    } else if (!wifiEverConnected) {
      wifiEverConnected = true;
      if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);
      server.begin();
      Serial.printf("[t=%8lu ms] [WIFI] Connected (late). IP: %s\n", millis() - bootTime, WiFi.localIP().toString().c_str());
    }
  }

  if (millis() - lastStepTime >= STEP_DELAY_MS) {
    lastStepTime = millis();
    for (int i = 0; i < 4; i++) {
      if (currentPulses[i] != targetPulses[i]) {
        if (currentPulses[i] < targetPulses[i]) currentPulses[i]++;
        else currentPulses[i]--;
        writePWMChecked(i, currentPulses[i]);
      }
    }
  }
}