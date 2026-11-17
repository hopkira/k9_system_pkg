
/*
 * K9 Espruino rotating ears — diagnostic build v2 (2026-11-17).
 * Based on the user's bounded, on-demand ears.js.
 *
 * USB commands (newline terminated): scan, fast, think, follow, stop,
 * read:42, ping, status, debug:0, debug:1.
 *
 * Normal responses: LIDAR / POSITION (each with seq), DONE (with seq).
 * Diagnostic responses: {"type":"DEBUG","at":"...",...}
 * NOTE: DEBUG responses are intentionally verbose. Stop ROS ear driver for tests.
 * Wait 30s after boot for USB protocol to start; USB JavaScript console moves
 * to Serial1 when the protocol starts.
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
var debug_enabled = true;
var init_stage = "not_started";
var init_error = "";
var command_count = 0;
var read_count = 0;
var last_read_ms = 0;
var current_mode = "not_started";

function sendMsg(message) {
  // No REPL writes to USB after switchConsole(); USB is protocol-only.
  if (transmit) USB.println(JSON.stringify(message));
}

function debugAt(where, detail) {
  if (!debug_enabled || !transmit) return;
  var msg = {type:"DEBUG", at:where};
  if (detail !== undefined) msg.detail = detail;
  sendMsg(msg);
}

function errText(e) {
  try { return String(e); } catch (ignored) { return "unknown error"; }
}

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

function statusReport() {
  sendMsg({type:"STATUS", version:"ears_debug_v2", init:init_stage,
           error:init_error, mode:current_mode, measuring:measuring,
           rx_count:command_count, read_count:read_count, last_seq:last_seq,
           last_read_ms:last_read_ms, debug:debug_enabled,
           left_ready:!!LIDAR_l, right_ready:!!LIDAR_r});
}

function onUsbData(data) {
  // Direct output before parsing: distinguishes RX from parsing/dispatch errors.
  debugAt("rx", String(data));
  command_buffer += data;
  if (command_buffer.length > 256) {
    debugAt("overflow", command_buffer.length);
    command_buffer = "";
    return;
  }
  var newline;
  while ((newline = command_buffer.indexOf("\n")) >= 0) {
    var cmd = command_buffer.slice(0, newline).trim();
    command_buffer = command_buffer.slice(newline + 1);
    command_count++;
    debugAt("command", cmd);
    try {
      if (cmd === "scan();" || cmd === "scan()" || cmd === "scan") scan();
      else if (cmd === "fast();" || cmd === "fast()" || cmd === "fast") fast();
      else if (cmd === "think();" || cmd === "think()" || cmd === "think") think();
      else if (cmd === "stop();" || cmd === "stop()" || cmd === "stop") stop();
      else if (cmd === "follow();" || cmd === "follow()" || cmd === "follow") follow();
      else if (cmd === "ping") sendMsg({type:"PONG", version:"ears_debug_v2"});
      else if (cmd === "status") statusReport();
      else if (cmd === "debug:0") {
        debugAt("debug_off");
        debug_enabled = false;
        sendMsg({type:"DEBUG_STATE", enabled:false});
      } else if (cmd === "debug:1") {
        debug_enabled = true;
        debugAt("debug_on");
        sendMsg({type:"DEBUG_STATE", enabled:true});
      } else if (cmd.slice(0, 5) === "read:") {
        var token = cmd.slice(5);
        debugAt("read_token", token);
        if (token.length < 1 || token.length > 6 || !/^[0-9]+$/.test(token)) {
          debugAt("read_rejected", "invalid_sequence");
          sendMsg({type:"ERROR", error:"invalid_sequence"});
        } else if (measuring) {
          debugAt("read_rejected", "busy");
          sendMsg({type:"ERROR", error:"busy"});
        } else {
          var seq = parseInt(token, 10);
          last_seq = seq;
          measuring = true;
          debugAt("read_accepted", seq);
          // 1 ms rather than 0 ms: avoids a possible zero-delay edge case.
          var tid = setTimeout(function() {
            debugAt("timer_fired", seq);
            takeReading(seq);
          }, 1);
          debugAt("timer_scheduled", tid);
        }
      } else {
        debugAt("unknown_command", cmd);
      }
    } catch (e) {
      debugAt("command_exception", errText(e));
      measuring = false;
      sendMsg({type:"ERROR", error:"command_exception", detail:errText(e)});
    }
  }
}

function switchConsole() {
  // Keep USB dedicated to the ROS/data protocol.
  Serial1.setConsole(true);
  USB.removeAllListeners("data");
  command_buffer = "";
  USB.on("data", onUsbData);
  transmit = true;
  // Initial READY is deliberate; makes startup and firmware identity visible.
  sendMsg({type:"READY", version:"ears_debug_v2", init:init_stage,
           error:init_error});
  stop();
  statusReport();
}

function follow() { current_mode = "follow"; setMovement(0); }
function stop()   { current_mode = "stop";   setMovement(0); }
function scan()   { current_mode = "scan";   setMovement(20); }
function fast()   { current_mode = "fast";   setMovement(75); }
function think()  { current_mode = "think";  setMovement(50); }

function onInit() {
  if (move_callback) clearInterval(move_callback);
  if (vref_callback) clearInterval(vref_callback);
  if (boot_callback) clearTimeout(boot_callback);
  move_callback = undefined;
  vref_callback = undefined;
  boot_callback = undefined;
  transmit = false;
  measuring = false;
  command_buffer = "";
  init_error = "";
  init_stage = "usb_setup";
  current_mode = "initialising";
  // Continue boot even if a sensor fails; report initialization status on USB.
  try { USB.setup(115200, {bytesize:8, stopbits:1}); }
  catch (e) { init_error = "USB: " + errText(e); }
  init_stage = "hardware";
  try { initHW(); }
  catch (e2) { init_error += " HW: " + errText(e2); }
  init_stage = "lidar";
  try { initLIDAR(); }
  catch (e3) { init_error += " LIDAR: " + errText(e3); }
  init_stage = "boot_delay";
  vref_callback = setInterval(refine_vRef, 1000);
  boot_callback = setTimeout(function() {
    init_stage = init_error ? "degraded" : "ready";
    switchConsole();
  }, 30000);
}

function calculateServoPos(stepValue) {
  var angle = stepValue / num_steps * Math.PI;
  return (angle - Math.cos(angle) * Math.sin(angle)) / Math.PI;
}
function scaleServoPos(position) {
  return PWM_l + ((PWM_r - PWM_l) * position);
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

function takeReading(seq) {
  read_count++;
  var start = getTime();
  debugAt("reading_started", seq);
  try {
    takeEarReading(LIDAR_l, PIN_pot_l, "l_ear", seq);
    debugAt("left_finished", seq);
    takeEarReading(LIDAR_r, PIN_pot_r, "r_ear", seq);
    debugAt("right_finished", seq);
  } catch (e) {
    debugAt("reading_exception", errText(e));
    sendMsg({type:"ERROR", seq:seq, error:"reading_exception", detail:errText(e)});
  } finally {
    last_read_ms = Math.round((getTime() - start) * 1000);
    debugAt("reading_finished_ms", last_read_ms);
    sendMsg({type:"DONE", seq:seq});
    measuring = false;
  }
}

function takeEarReading(read_lidar, read_pin, ear, seq) {
  var volts, angle, dist;
  debugAt("ear_started", ear);
  try {
    volts = analogRead(read_pin) * vRef;
    angle = volts2angle(volts, ear);
    if (!isFinite(angle)) {
      debugAt("angle_invalid", ear);
      sendMsg({type:"ERROR", seq:seq, sensor:ear, error:"angle_invalid"});
      return;
    }
    debugAt("angle_ok", ear);
  } catch (e) {
    debugAt("angle_exception", ear + ": " + errText(e));
    sendMsg({type:"ERROR", seq:seq, sensor:ear, error:"angle_exception"});
    return;
  }
  try {
    if (!read_lidar) throw new Error("LIDAR not initialized");
    debugAt("lidar_call", ear);
    dist = read_lidar.performSingleMeasurement().distance;
    debugAt("lidar_return", ear + ":" + dist);
    if (isFinite(dist) && dist > 20 && dist < 8000) {
      sendMsg({type:"LIDAR", sensor:ear, distance:dist / 1000,
               angle:angle, seq:seq});
      return;
    }
    debugAt("lidar_invalid", ear);
  } catch (e2) {
    debugAt("lidar_exception", ear + ": " + errText(e2));
  }
  sendMsg({type:"POSITION", sensor:ear, angle:angle, seq:seq});
}

function moveEars() {
  step += direction;
  if (step > num_steps) { direction = -1; step = num_steps - 1; }
  if (step < 0) { direction = 1; step = 1; }
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
  if (freq > 0) move_callback = setInterval(moveEars, 1000.0 / freq);
  else resetEars();
  debugAt("movement", current_mode);
}

for (var i = 0; i <= num_steps; i++) {
  step_lookup[i] = scaleServoPos(calculateServoPos(i));
}
