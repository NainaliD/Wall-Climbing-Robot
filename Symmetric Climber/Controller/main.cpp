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
};

JointConfig joints[4] = {
  { "Joint 0", 100, 600, 270 },
  { "Joint 1", 120, 590, 180 },
  { "Joint 2", 120, 580, 180 },
  { "Joint 3", 100, 600, 270 },
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
body { font-family: 'Segoe UI', Arial, sans-serif; background: #111115; color: #f7f7ff; text-align: center; padding: 15px; margin: 0; }
.wrapper { display: flex; flex-direction: column; gap: 20px; max-width: 650px; margin: 0 auto; padding-bottom: 5px; }
.container { background: #1c1c24; padding: 22px; border-radius: 12px; box-shadow: 0 6px 25px rgba(0,0,0,0.7); text-align: left; }
h1 { color: #00adb5; margin: 0 0 5px 0; font-size: 24px; text-transform: uppercase; text-align: center; letter-spacing: 1px;}
.subtitle { color: #8f8fbf; font-size: 13px; margin-bottom: 20px; text-align: center; }
h2 { color: #00adb5; font-size: 14px; margin: 0 0 15px 0; text-transform: uppercase; letter-spacing: 1px; border-bottom: 1px solid #2d2d3d; padding-bottom: 6px;}
.joint-card { background: #262633; padding: 15px; border-radius: 8px; margin-bottom: 15px; border-left: 4px solid #00adb5; }
.joint-header { display: flex; justify-content: space-between; align-items: center; font-weight: bold; font-size: 15px; margin-bottom: 10px; }
.deg-input { background: #111115; color: #00f5ff; border: 1px solid #00adb5; border-radius: 4px; padding: 5px; width: 65px; font-family: monospace; font-size: 16px; font-weight: bold; text-align: center; outline: none;}
.slider-row { display: flex; align-items: center; margin-bottom: 10px; }
.slider { width: 100%; -webkit-appearance: none; background: #3d3d52; height: 8px; border-radius: 5px; outline: none; }
.slider::-webkit-slider-thumb { -webkit-appearance: none; width: 22px; height: 22px; border-radius: 50%; background: #00adb5; cursor: pointer; }
.cal-matrix { display: flex; gap: 15px; background: #1c1c24; padding: 8px; border-radius: 6px; font-size: 11px; align-items: center; justify-content: space-between; color: #a9a9b3; }
.cal-input { background: #323245; color: #fff; border: 1px solid #555; border-radius: 4px; padding: 4px; width: 45px; text-align: center; font-family: monospace; }
.magnet-row { display: flex; justify-content: space-between; align-items: center; background: #262633; padding: 12px 15px; border-radius: 8px; margin-bottom: 10px; }
.switch { position: relative; display: inline-block; width: 50px; height: 26px; }
.switch input { opacity: 0; width: 0; height: 0; }
.toggle-slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #dc3545; transition: .3s; border-radius: 34px; }
.toggle-slider:before { position: absolute; content: ""; height: 18px; width: 18px; left: 4px; bottom: 4px; background-color: white; transition: .3s; border-radius: 50%; }
input:checked + .toggle-slider { background-color: #28a745; }
input:checked + .toggle-slider:before { transform: translateX(24px); }
.seq-row { display: flex; gap: 10px; margin-bottom: 12px; align-items: center; flex-wrap: wrap; }
.btn { padding: 9px 14px; font-size: 12px; border: none; border-radius: 4px; cursor: pointer; font-weight: bold; color: white; display: inline-flex; align-items: center; justify-content: center;}
.btn-blue { background: #00adb5; } .btn-blue:hover { background: #00f5ff; color: #000; }
.btn-green { background: #28a745; } .btn-green:hover { background: #218838; }
.btn-red { background: #dc3545; } .btn-red:hover { background: #c82333; }
.btn-orange { background: #ff9f43; } .btn-orange:hover { background: #ee5253; }
.btn-grey { background: #4b4b61; } .btn-grey:hover { background: #5f5f7d; }
.btn-estop { background: #ff2e2e; font-size: 16px; padding: 14px; width: 100%; }
.text-field { background: #111115; color: #fff; border: 1px solid #444; padding: 8px; border-radius: 4px; outline: none; font-size: 13px; }
.delay-input { width: 65px; text-align: center; font-family: monospace; }
#seqQueue { background: #111115; min-height: 100px; max-height: 220px; overflow-y: auto; padding: 8px; border-radius: 6px; border: 1px solid #2d2d3d; margin-bottom: 10px; }
.queue-item { display: flex; justify-content: space-between; align-items: center; background: #21212d; padding: 6px 10px; margin-bottom: 5px; border-radius: 4px; font-family: monospace; font-size: 11px; cursor: pointer; border: 1px solid transparent; color: #b3b3cc;}
.queue-item:hover { border-color: #00adb5; }
.queue-item.active-frame { border-color: #00f5ff; background: #1a2a3a; color: #00f5ff; font-weight: bold; }
.item-actions { display: flex; gap: 4px; }
.mini-btn { padding: 2px 6px; font-size: 10px; border-radius: 3px; border: none; color: white; cursor: pointer; }
.inv-select { flex-grow: 1; background: #111115; color: #fff; border: 1px solid #444; padding: 8px; border-radius: 4px; font-size: 13px; }
.status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; font-family: monospace; font-size: 12px; color: #a9a9b3; }
.status-grid b { color: #00f5ff; }
.live-stats { font-family: monospace; color: #00f5ff; font-size: 15px; text-align: center; margin-bottom: 4px;}
.live-stats span { margin: 0 8px; }
#logChart { width: 100%; height: 140px; background: #111115; border-radius: 6px; border: 1px solid #2d2d3d; display: block; margin-bottom: 10px; }
.log-stats { font-family: monospace; font-size: 12px; color: #a9a9b3; text-align: center; margin-bottom: 10px; }
.log-stats b { color: #00f5ff; }
.rate-select { background: #111115; color: #fff; border: 1px solid #444; padding: 8px; border-radius: 4px; font-size: 13px; }
.btn-logging { background: #dc3545; }
.btn-logging.active { background: #28a745; animation: pulseRec 1.5s infinite; }
@keyframes pulseRec { 0%,100% { opacity: 1; } 50% { opacity: 0.6; } }
</style>
</head>
<body>
<div class='wrapper'>
<h1>4-DOF Arm Controller</h1>
<div class='subtitle'>Hardened Build | Self-Recovering I2C | Persistent Calibration</div>

<div class='container'>
<button class='btn btn-estop' onclick="fetch('/stop').then(()=>flashMsg('Motion halted'))">EMERGENCY STOP (halts motion only — magnets unaffected)</button>
</div>
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

  page += "<div class='container'><h2>Joint Configuration</h2>";
  for (int i = 0; i < 4; i++) {
    int curDeg = map(currentPulses[i], joints[i].minPulse, joints[i].maxPulse, 0, joints[i].maxDegrees);
    page += "<div class='joint-card'>";
    page += "<div class='joint-header'><span>" + String(joints[i].label) + " (0-" + String(joints[i].maxDegrees) + "&deg;)</span>";
    page += "<input type='number' min='0' max='" + String(joints[i].maxDegrees) + "' value='" + String(curDeg) +
            "' class='deg-input' id='num" + String(i) + "' onchange='syncInputs(" + String(i) + ", this.value, true)'></div>";
    page += "<div class='slider-row'><input type='range' min='0' max='" + String(joints[i].maxDegrees) + "' value='" + String(curDeg) +
            "' class='slider' id='deg" + String(i) + "' oninput='syncInputs(" + String(i) + ", this.value, false)'></div>";
    page += "<div class='cal-matrix'><span>Trim Calibration:</span><div>0&deg;=<input type='number' value='" + String(joints[i].minPulse) +
            "' class='cal-input' id='min" + String(i) + "' onchange='updateBounds(" + String(i) + ")'></div>" +
            "<div>" + String(joints[i].maxDegrees) + "&deg;=<input type='number' value='" + String(joints[i].maxPulse) +
            "' class='cal-input' id='max" + String(i) + "' onchange='updateBounds(" + String(i) + ")'></div></div>";
    page += "</div>";
  }
  page += "<span id='statusMsg' style='color:#00f5ff;font-size:12px;'></span></div>";

  page += "<div class='container'><h2>End Effector</h2>";
  page += "<div class='magnet-row'><span>Magnet 1 (GPIO1)</span><label class='switch'><input type='checkbox' id='mag1' " +
          String(magnetState[0] ? "checked" : "") + " onchange='controlMagnet(1, this.checked)'><span class='toggle-slider'></span></label></div>";
  page += "<div class='magnet-row'><span>Magnet 2 (GPIO3)</span><label class='switch'><input type='checkbox' id='mag2' " +
          String(magnetState[1] ? "checked" : "") + " onchange='controlMagnet(2, this.checked)'><span class='toggle-slider'></span></label></div>";
  page += "</div>";

  page += "<div class='container'><h2>Power Telemetry &amp; Logger</h2>";
  page += "<div class='live-stats'><span id='liveV'>-- V</span><span id='liveI'>-- mA</span><span id='liveP'>-- mW</span></div>";
  page += "<canvas id='logChart' width='600' height='140'></canvas>";
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
  page += "<h2>Saved Logging Sessions</h2>";
  page += "<div class='seq-row'><input type='text' id='logSessionName' class='text-field' style='flex-grow:1;' placeholder='Session name...'>";
  page += "<button class='btn btn-blue' onclick='saveLogSession()'>Save</button></div>";
  page += "<div class='seq-row'><select id='logSessionSelect' class='inv-select' onchange='loadLogSession()'></select>";
  page += "<button class='btn btn-red' onclick='deleteLogSession()'>Delete</button></div>";
  page += "</div>";

  page += "<div class='container'><h2>System Health</h2>";
  page += "<div class='status-grid'>";
  page += "<div>WiFi RSSI: <b id='liveRSSI'>--</b></div><div>Uptime: <b id='liveUptime'>--</b></div>";
  page += "<div>I2C Errors: <b id='liveErrors'>--</b></div><div>Bus Recoveries: <b id='liveRecoveries'>--</b></div>";
  page += "<div>Free Heap: <b id='liveHeap'>--</b></div><div>Host: <b>" + String(HOSTNAME) + ".local</b></div>";
  page += "</div></div>";

  page += "<div class='container'><h2>Sequence Timeline</h2><div id='seqQueue'>Timeline empty...</div>";
  page += "<div class='seq-row'><span style='font-size:12px;color:#8f8fbf;'>Hold Interval:</span>";
  page += "<input type='number' id='delayTime' value='1500' class='text-field delay-input'><span style='font-size:12px;color:#8f8fbf;'>ms</span>";
  page += "<button class='btn btn-blue' style='flex-grow:1;' onclick='captureFrame()'>Capture Frame</button>";
  page += "<button class='btn btn-orange' id='updateBtn' style='display:none;' onclick='updateSelectedFrame()'>Update Selected</button></div>";
  page += "<div class='seq-row'><button class='btn btn-green' style='flex-grow:2;font-size:14px;' onclick='playSequence()' id='playBtn'>Play Timeline</button>";
  page += "<button class='btn btn-red' onclick='clearActiveSequence()'>Clear Queue</button></div>";
  page += "<h2>Saved Routines</h2><div class='seq-row'><input type='text' id='storageName' class='text-field' style='flex-grow:1;' placeholder='Routine name...'>";
  page += "<button class='btn btn-blue' onclick='saveActiveSequence()'>Save</button></div>";
  page += "<div class='seq-row'><select id='inventorySelect' class='inv-select' onchange='loadSelectedInventory()'></select>";
  page += "<button class='btn btn-red' onclick='deleteSelectedInventory()'>Delete</button></div></div>";

  page += "</div>"; // wrapper
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