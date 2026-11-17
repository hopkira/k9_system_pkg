/*
K9 Espruino rotating ears — bounded on-demand measurements and independent movement.
Based on the original ears.js (Richard Hopkins, 7 February 2021).
Published under The Unlicense.

USB JSON:
  {"type":"LIDAR","sensor":"l_ear","distance":0.6,"angle":0.2,"seq":7}
  {"type":"POSITION","sensor":"l_ear","angle":0.2,"seq":7} // no valid range
  {"type":"DONE","seq":7}
Pi sends read:<sequence_number>\n; max one reading pair is produced per request.

Movement commands: follow(), stop(), scan(), fast(), think().
All modes allow sensing requests. No periodic sensor-output timer or unbounded queue.
stop() returns servos to neutral; scan(), fast(), think() only change movement.
*/

var PWM_l = 0.15;
var PWM_r = 0.8;
var PIN_PWM_l = B15;
var PIN_PWM_r = B14;
var PIN_LIDARX_l = B4;
var PIN_LIDARX_r = A8;
var PIN_pot_l = B1;
var PIN_pot_r = A7;
var PIN_sda = B3;
var PIN_scl = B10;

var num_steps = 50;
var step = 0;
var direction = 1;
var LIDAR_l, LIDAR_r;
var vRef = E.getAnalogVRef();
var num_readings = 1;
var transmit = false;
var move_callback;
var vref_callback;
var boot_callback;
var command_buffer = "";
var measuring = false;
var last_seq = -1;
var step_lookup = [];

function setServo(pin, pos) {
  if (pos < 0) pos = 0;
  if (pos > 1) pos = 1;
  analogWrite(pin, (1 + pos) / 50.0, {freq:20});
}

function initHW() {
  digitalWrite(PIN_LIDARX_l, 0);
  digitalWrite(PIN_LIDARX_r, 0);
  I2C2.setup({sda:PIN_sda, scl:PIN_scl, bitrate:100000});
}

function initLIDAR() {
  digitalWrite(PIN_LIDARX_l, 1);
  LIDAR_l = require("VL53L0X").connect(I2C2, {address:0x54});
  digitalWrite(PIN_LIDARX_r, 1);
  LIDAR_r = require("VL53L0X").connect(I2C2, {address:0x56});
}

function refine_vRef() {
  num_readings += 1;
  var reading = E.getAnalogVRef();
  vRef = (vRef * (num_readings - 1) + reading) / num_readings;
}

// USB is a dedicated ROS protocol, NOT a JavaScript console.
// Commands are newline-delimited and are never eval()ed.
function onUsbData(data) {
  command_buffer += data;
  if (command_buffer.length > 256) {
    command_buffer = ""; // Discard malformed input; never grow without bound.
    return;
  }
  var newline;
  while ((newline = command_buffer.indexOf("\n")) >= 0) {
    var cmd = command_buffer.slice(0, newline).trim();
    command_buffer = command_buffer.slice(newline + 1);
    if (cmd === "scan();" || cmd === "scan()" || cmd === "scan") scan();
    else if (cmd === "fast();" || cmd === "fast()" || cmd === "fast") fast();
    else if (cmd === "think();" || cmd === "think()" || cmd === "think") think();
    else if (cmd === "stop();" || cmd === "stop()" || cmd === "stop") stop();
    else if (cmd === "follow();" || cmd === "follow()" || cmd === "follow") follow();
    else if (cmd.slice(0, 5) === "read:") {
      var token = cmd.slice(5);
      // No unbounded queue: ignore requests if we're already taking readings.
      if (token.length >= 1 && token.length <=6 && /^[0-9]+$/.test(token) && !measuring) {
        var seq = parseInt(token, 10);
        // Do the measurements in the next event-loop turn. Commands remain
        // independent of the 20-75Hz servo movement timer.
        measuring = true;
        setTimeout(function() { takeReading(seq); }, 0);
      }
    }
  }
}

function switchConsole() {
  // Force the REPL onto Serial1 to prevent USB reconnection taking it back.
  Serial1.setConsole(true);
  USB.removeAllListeners("data");
  command_buffer = "";
  USB.on("data", onUsbData);
  transmit = true;
  stop(); // Neutral at startup. Readings are produced only when requested.
}

function follow() { setMovement(0); }
function stop()   { setMovement(0); }
function scan()   { setMovement(20); }
function fast()   { setMovement(75); }
function think()  { setMovement(50); }

function onInit() {
  // Stop only timers owned by this script, so repeated onInit is predictable.
  if (move_callback) clearInterval(move_callback);
  if (vref_callback) clearInterval(vref_callback);
  if (boot_callback) clearTimeout(boot_callback);
  move_callback = undefined;
  vref_callback = undefined;

  transmit = false;
  measuring = false;
  command_buffer = "";
  USB.setup(115200, {bytesize:8, stopbits:1});
  initHW();
  initLIDAR();

  vref_callback = setInterval(refine_vRef, 1000);
  // Retain the original USB/console boot delay.
  boot_callback = setTimeout(switchConsole, 30000);
}

function calculateServoPos(stepValue) {
  var angle = stepValue / num_steps * Math.PI;
  return (angle - Math.cos(angle) * Math.sin(angle)) / Math.PI;
}

function scaleServoPos(position) {
  return PWM_l + ((PWM_r - PWM_l) * position);
}

function sendMsg(message) {
  if (transmit) USB.println(JSON.stringify(message));
}

function volts2angle(volts, ear) {
  var MIN_V = 1.33;
  var MAX_V = 2.10;
  var MAX_ANG = Math.PI / 4;
  var angle = Math.min(Math.max((volts - MIN_V) / (MAX_V - MIN_V), 0), 1);
  if (ear == "l_ear") angle = 1 - angle;
  if (ear == "r_ear") angle = -angle;
  return angle * MAX_ANG;
}

// Only one poll is permitted at a time. Two readings plus one DONE, no backlog.
// Each measurement is synchronous on this Espruino API; polling is deliberately
// limited by the Pi to reduce event-loop and USB pressure.
function takeReading(seq) {
  try {
    takeEarReading(LIDAR_l, PIN_pot_l, "l_ear", seq);
    takeEarReading(LIDAR_r, PIN_pot_r, "r_ear", seq);
    sendMsg({type:"DONE", seq:seq});
  } finally {
    measuring = false;
  }
}

function takeEarReading(read_lidar, read_pin, ear, seq) {
  var volts, angle, dist;
  try {
    volts = analogRead(read_pin) * vRef;
    angle = volts2angle(volts, ear);
    if (!isFinite(angle)) return;
  } catch (e) {
    return;
  }
  try {
    dist = read_lidar.performSingleMeasurement().distance;
    if (isFinite(dist) && dist > 20 && dist < 8000) {
      sendMsg({type:"LIDAR", sensor:ear, distance:dist/1000,
               angle:angle, seq:seq});
      return;
    }
  } catch (e) {
    // A failed range measurement is not free space.
  }
  sendMsg({type:"POSITION", sensor:ear, angle:angle, seq:seq});
}

function moveEars() {
  step += direction;
  if (step > num_steps) {
    direction = -1;
    step = num_steps - 1;
  }
  if (step < 0) {
    direction = 1;
    step = 1;
  }
  var scaled_pos = step_lookup[step];
  setServo(PIN_PWM_l, scaled_pos);
  setServo(PIN_PWM_r, 1 - scaled_pos);
}

function resetEars() {
  var scaled_pos = step_lookup[num_steps - 2];
  setServo(PIN_PWM_l, scaled_pos);
  setServo(PIN_PWM_r, 1 - scaled_pos);
}

function setMovement(freq) {
  if (move_callback) clearInterval(move_callback);
  move_callback = undefined;
  if (freq > 0) {
    move_callback = setInterval(moveEars, 1000.0 / freq);
  } else {
    resetEars();
  }
  // Sensing is separately requested by the Pi; no sensor timer to alter.
}

for (var i = 0; i <= num_steps; i++) {
  step_lookup[i] = scaleServoPos(calculateServoPos(i));
}
