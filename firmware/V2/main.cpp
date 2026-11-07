// ARM-E: WiFi web control for 6 servos (PCA9685) and 4 DC motors (2x TB6612FNG)
// with a mecanum drive joystick.
// Board: ESP32 Dev Module (classic ESP32). Framework: Arduino (PlatformIO).
//
// Open the page at http://<ip shown in Serial Monitor>  or  http://arme.local
// If your WiFi can't be joined within 15 s, the ESP32 makes its own network
// (name ARM-E-Robot, password robot1234) and the page is at http://192.168.4.1
//
// DRIVING IS ROBOT-CENTRIC: "up" on the joystick always means "the way the
// robot's front is facing right now". After you rotate the robot, up still
// moves it toward its new front. (Field-centric driving would need a gyro/IMU.)


#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// ====================== SETTINGS YOU EDIT ======================


// ---- WiFi (2.4 GHz only) ----
const char* WIFI_SSID = "rapid";
const char* WIFI_PASS = "4Fagro@2023";
const char* AP_SSID   = "ARM-E-Robot";   // fallback network
const char* AP_PASS   = "robot1234";     // at least 8 characters
const char* HOSTNAME  = "arme";          // -> http://arme.local


// ---- I2C / PCA9685 ----
#define PIN_SDA    21
#define PIN_SCL    22
#define PIN_PCA_OE 5     // PCA9685 OE pin (HIGH = servo outputs off). Optional wire.


// ---- TB6612 shared standby (HIGH = drivers enabled) ----
#define PIN_STBY   4


// ---- Servos (PCA9685 channels 0..5) ----
const int NUM_SERVOS = 6;
int servoMinUs[NUM_SERVOS] = {600, 600, 600, 600, 600, 600};
int servoMaxUs[NUM_SERVOS] = {2400, 2400, 2400, 2400, 2400, 2400};
int servoLimLo[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};
int servoLimHi[NUM_SERVOS] = {180, 180, 180, 180, 180, 180};
int servoInit[NUM_SERVOS]  = {90, 90, 90, 90, 90, 90};
const float SERVO_DEG_PER_SEC = 90.0f;


// ---- Motors (physical channels) ----
// Motor 0 = driver 1, channel A      Motor 1 = driver 1, channel B
// Motor 2 = driver 2, channel A      Motor 3 = driver 2, channel B
// Which wheel each motor drives, and which motors are reversed, is set on the
// web page (Wheel setup) and saved in the ESP32's flash. The values below are
// only the defaults used before you save anything.
struct Motor {
  uint8_t in1;
  uint8_t in2;
  uint8_t pwmPin;
  uint8_t ledcCh;   // used only on older Arduino-ESP32 cores (2.x)
};
const int NUM_MOTORS = 4;
Motor motors[NUM_MOTORS] = {
  // in1 in2 pwm ch
  { 13, 14, 16, 0 },   // M0: AIN1, AIN2, PWMA of driver 1
  { 17, 18, 19, 1 },   // M1: BIN1, BIN2, PWMB of driver 1
  { 23, 25, 26, 2 },   // M2: AIN1, AIN2, PWMA of driver 2
  { 27, 32, 33, 3 },   // M3: BIN1, BIN2, PWMB of driver 2
};
// Wheel order everywhere: 0 = Front-Left, 1 = Front-Right, 2 = Rear-Left, 3 = Rear-Right
const uint8_t DEFAULT_WHEEL_MOTOR[4] = {0, 1, 2, 3};   // wheel -> motor
const bool    DEFAULT_REVERSE[4]     = {false, true, false, true}; // per motor


const int      PWM_FREQ_HZ         = 20000;  // above hearing range, no whine
const int      PWM_RES_BITS        = 8;      // duty 0..255
const float    MOTOR_RAMP_PER_TICK = 8.0f;   // percent per 20 ms = gentle start/stop
const uint32_t MOTOR_TIMEOUT_MS    = 1500;   // motors stop if the page stops sending commands


// ================================================================


Adafruit_PWMServoDriver pwm(0x40);
WebServer server(80);
Preferences prefs;


float servoCur[NUM_SERVOS];
int   servoTarget[NUM_SERVOS];
float motorCur[NUM_MOTORS];
int   motorTarget[NUM_MOTORS];


uint8_t wheelMotor[4];     // wheel -> motor index (saved)
bool    motorReverse[4];   // per motor (saved)


uint32_t lastMotorCmd = 0;
uint32_t lastTick = 0;
bool staMode = false;


// ---------------- Web page ----------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ARM-E Control</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;padding:12px;background:#10141a;color:#e8edf3;max-width:720px;margin:auto;-webkit-user-select:none;user-select:none}
h1{font-size:20px;margin:6px 0 10px}
h2{margin:18px 0 8px;font-size:17px;color:#9fb3c8}
.card{background:#1a212b;border-radius:10px;padding:8px 12px;margin-bottom:8px}
.row{display:flex;align-items:center;gap:10px}
.row label{width:120px;font-size:14px}
input[type=range]{flex:1;height:34px}
select{flex:1;padding:8px;background:#2d3a4a;color:#fff;border:0;border-radius:8px;font-size:14px}
.val{width:44px;text-align:right;font-variant-numeric:tabular-nums}
button{background:#2d3a4a;color:#fff;border:0;border-radius:8px;padding:8px 12px;font-size:14px;touch-action:none}
button.stop{background:#c0392b;font-weight:700}
button.on{background:#3b82f6}
#stopall{width:100%;padding:16px;font-size:18px;margin-bottom:8px}
.note{font-size:13px;color:#9fb3c8}
#padwrap{text-align:center}
#pad{width:220px;height:220px;box-sizing:border-box;border-radius:50%;background:#1a212b;border:2px solid #2d3a4a;position:relative;margin:4px auto;touch-action:none}
#pad:before{content:"";position:absolute;left:50%;top:8px;bottom:8px;width:1px;background:#2d3a4a}
#pad:after{content:"";position:absolute;top:50%;left:8px;right:8px;height:1px;background:#2d3a4a}
#knob{width:70px;height:70px;border-radius:50%;background:#3b82f6;position:absolute;left:50%;top:50%;margin:-35px 0 0 -35px;pointer-events:none;z-index:2}
.readout{text-align:center;font-size:15px;margin:4px 0}
.turn{display:flex;gap:10px;justify-content:center;margin:8px 0}
.turn button{flex:1;padding:16px;font-size:16px;max-width:200px}
.chk{font-size:14px;margin-right:12px;white-space:nowrap}
</style></head><body>
<h1>ARM-E Control</h1>
<button class="stop" id="stopall">STOP ALL MOTORS</button>


<h2>Drive (mecanum)</h2>
<div class="card">
  <div class="note" style="text-align:center">FRONT of robot = up</div>
  <div id="padwrap"><div id="pad"><div id="knob"></div></div></div>
  <div class="readout">Angle: <b id="ang">-</b>&deg; &nbsp; Speed: <b id="spd">0</b>%</div>
  <div class="turn">
    <button id="rotL">&#9664; Rotate left</button>
    <button id="rotR">Rotate right &#9654;</button>
  </div>
  <div class="row"><label>Max speed</label><input type="range" id="maxsp" min="10" max="100" value="60"><span class="val" id="maxspv">60</span></div>
  <div class="row"><label>Turn speed</label><input type="range" id="turnsp" min="10" max="100" value="60"><span class="val" id="turnspv">60</span></div>
  <div class="note">Angle 0 = straight ahead, 90 = right, 180 = back, 270 = left, always relative to the way the robot is currently facing. Joystick distance sets speed. Hold a rotate button (or use two fingers) to turn while moving. Keyboard: W A S D to move, &larr; &rarr; (or Q E) to rotate.</div>
</div>


<h2>Wheel setup</h2>
<div class="card note">Pick which physical motor drives each wheel, tick Reverse for any motor whose wheel spins backward, then press Save. "Test" nudges the selected motor forward for a moment (wheels off the ground!).</div>
<div id="wheelsetup"></div>
<div class="card" id="revrow"></div>
<button id="savecfg">Save wheel setup</button>
<span class="note" id="cfgmsg"></span>


<h2>Servos (angle in degrees)</h2>
<div id="servos"></div>


<h2>Single motor test (-100 reverse ... +100 forward)</h2>
<label class="note"><input type="checkbox" id="latch"> Latch motors (keep running after I let go)</label>
<div id="motors"></div>


<script>
const MOTOR_NAMES=["M0 (driver 1, ch A)","M1 (driver 1, ch B)","M2 (driver 2, ch A)","M3 (driver 2, ch B)"];
const WHEEL_NAMES=["Front-Left","Front-Right","Rear-Left","Rear-Right"];
const SERVO_NAMES=["Servo 0","Servo 1","Servo 2","Servo 3","Servo 4","Servo 5"];
const $=function(id){return document.getElementById(id);};
function send(u){fetch(u).catch(function(){});}
function throttle(fn,ms){let t=0;return function(){const n=Date.now();if(n-t>=ms){t=n;fn();}};}
document.addEventListener('contextmenu',function(e){e.preventDefault();});


// ---------- Drive joystick ----------
const pad=('pad'),knob=('knob');
const joy={x:0,y:0,active:false};
const rotState={L:false,R:false};
const kb={x:0,y:0,r:0};
const keys={};


function driveCmd(){
  let x=0,y=0;
  if(joy.active){x=joy.x;y=joy.y;}
  else{x=kb.x;y=kb.y;const m=Math.hypot(x,y);if(m>1){x/=m;y/=m;}}
  const rdir=Math.max(-1,Math.min(1,(rotState.R?1:0)-(rotState.L?1:0)+kb.r));
  const tp=+$('turnsp').value;
  return {x:Math.round(x*100),y:Math.round(y*100),r:Math.round(rdir*tp)};
}
function updateReadout(){
  const d=driveCmd();
  const mag=Math.min(1,Math.hypot(d.x,d.y)/100);
  ('spd').textContent=Math.round(mag*(+('maxsp').value));
  if(mag>0.001){let a=Math.atan2(d.x,d.y)*180/Math.PI;if(a<0)a+=360;$('ang').textContent=Math.round(a);}
  else{$('ang').textContent='-';}
}
let lastKey='';
function pushDrive(){
  const d=driveCmd();const lim=+$('maxsp').value;
  const key=d.x+','+d.y+','+d.r+','+lim;
  const moving=d.x||d.y||d.r;
  if(moving||key!==lastKey){send('/drive?x='+d.x+'&y='+d.y+'&r='+d.r+'&lim='+lim);lastKey=key;}
  updateReadout();
}
setInterval(pushDrive,120);


function moveJoy(e){
  const r=pad.getBoundingClientRect();
  const R=r.width/2-35;
  let dx=e.clientX-(r.left+r.width/2),dy=e.clientY-(r.top+r.height/2);
  const dist=Math.hypot(dx,dy);
  if(dist>R){dx=dx*R/dist;dy=dy*R/dist;}
  knob.style.transform='translate('+dx+'px,'+dy+'px)';
  let x=dx/R,y=-dy/R;
  if(Math.hypot(x,y)<0.1){x=0;y=0;}
  joy.x=x;joy.y=y;
}
function endJoy(){joy.active=false;joy.x=0;joy.y=0;knob.style.transform='translate(0,0)';}
pad.addEventListener('pointerdown',function(e){e.preventDefault();pad.setPointerCapture(e.pointerId);joy.active=true;moveJoy(e);});
pad.addEventListener('pointermove',function(e){if(joy.active)moveJoy(e);});
pad.addEventListener('pointerup',endJoy);
pad.addEventListener('pointercancel',endJoy);


function hold(id,key){
  const b=$(id);
  const up=function(){rotState[key]=false;b.classList.remove('on');};
  b.addEventListener('pointerdown',function(e){e.preventDefault();b.setPointerCapture(e.pointerId);rotState[key]=true;b.classList.add('on');});
  b.addEventListener('pointerup',up);
  b.addEventListener('pointercancel',up);
  b.addEventListener('lostpointercapture',up);
}
hold('rotL','L');hold('rotR','R');


function kbUpdate(){
  kb.y=(keys['w']?1:0)-(keys['s']?1:0);
  kb.x=(keys['d']?1:0)-(keys['a']?1:0);
  kb.r=((keys['arrowright']||keys['e'])?1:0)-((keys['arrowleft']||keys['q'])?1:0);
}
window.addEventListener('keydown',function(e){
  const t=document.activeElement?document.activeElement.tagName:'';
  if(t==='SELECT'||t==='INPUT'||t==='TEXTAREA')return;
  const k=e.key.toLowerCase();
  if(k==='arrowleft'||k==='arrowright')e.preventDefault();
  keys[k]=true;kbUpdate();
});
window.addEventListener('keyup',function(e){keys[e.key.toLowerCase()]=false;kbUpdate();});
window.addEventListener('blur',function(){for(const k in keys)keys[k]=false;kbUpdate();endJoy();});


$('maxsp').addEventListener('input',function(){$('maxspv').textContent=this.value;});
$('turnsp').addEventListener('input',function(){$('turnspv').textContent=this.value;});


// ---------- Wheel setup ----------
const wb=$('wheelsetup');
WHEEL_NAMES.forEach(function(name,k){
  const d=document.createElement('div');d.className='card row';
  let opts='';
  MOTOR_NAMES.forEach(function(m,i){opts+='<option value="'+i+'">'+m+'</option>';});
  d.innerHTML='<label>'+name+'</label><select id="wsel'+k+'">'+opts+'</select><button>Test</button>';
  wb.appendChild(d);
  d.querySelector('button').onclick=function(){
    const id=+$('wsel'+k).value;
    send('/motor?id='+id+'&speed=40');
    setTimeout(function(){send('/motor?id='+id+'&speed=0');},700);
  };
});
let revHtml='';
MOTOR_NAMES.forEach(function(m,i){revHtml+='<label class="chk"><input type="checkbox" id="rev'+i+'"> Reverse M'+i+'</label>';});
$('revrow').innerHTML=revHtml;


function loadConfig(){
  fetch('/config').then(function(r){return r.json();}).then(function(c){
    for(let k=0;k<4;k++){$('wsel'+k).value=c.map[k];}
    for(let i=0;i<4;i++){$('rev'+i).checked=!!c.rev[i];}
  }).catch(function(){});
}
$('savecfg').onclick=function(){
  const m=[];for(let k=0;k<4;k++)m.push(+$('wsel'+k).value);
  if(new Set(m).size!==4){$('cfgmsg').textContent=' Each motor can only be used for one wheel.';return;}
  let rev='';for(let i=0;i<4;i++)rev+=$('rev'+i).checked?'1':'0';
  fetch('/setconfig?fl='+m[0]+'&fr='+m[1]+'&rl='+m[2]+'&rr='+m[3]+'&rev='+rev)
    .then(function(r){return r.text();})
    .then(function(t){$('cfgmsg').textContent=' '+t;loadConfig();})
    .catch(function(){$('cfgmsg').textContent=' Save failed.';});
};
loadConfig();


// ---------- Servos ----------
const sSl=[],sVal=[];
const sc=$('servos');
SERVO_NAMES.forEach(function(name,i){
  const d=document.createElement('div');d.className='card row';
  d.innerHTML='<label>'+name+'</label><input type="range" min="0" max="180" value="90"><span class="val">90</span><button>90</button>';
  sc.appendChild(d);
  const sl=d.querySelector('input'),v=d.querySelector('.val'),b=d.querySelector('button');
  const snd=throttle(function(){send('/servo?ch='+i+'&angle='+sl.value);},50);
  sl.addEventListener('input',function(){v.textContent=sl.value;snd();});
  sl.addEventListener('change',function(){send('/servo?ch='+i+'&angle='+sl.value);});
  b.onclick=function(){sl.value=90;v.textContent='90';send('/servo?ch='+i+'&angle=90');};
  sSl.push(sl);sVal.push(v);
});


// ---------- Single motor test ----------
const mSl=[],mVal=[];
const mc=$('motors');
const latch=$('latch');
MOTOR_NAMES.forEach(function(name,i){
  const d=document.createElement('div');d.className='card row';
  d.innerHTML='<label>'+name+'</label><input type="range" min="-100" max="100" value="0"><span class="val">0</span><button>0</button>';
  mc.appendChild(d);
  const sl=d.querySelector('input'),v=d.querySelector('.val'),b=d.querySelector('button');
  const snd=throttle(function(){send('/motor?id='+i+'&speed='+sl.value);},70);
  sl.addEventListener('input',function(){v.textContent=sl.value;snd();});
  sl.addEventListener('change',function(){
    if(!latch.checked){sl.value=0;v.textContent='0';}
    send('/motor?id='+i+'&speed='+sl.value);
  });
  b.onclick=function(){sl.value=0;v.textContent='0';send('/motor?id='+i+'&speed=0');};
  mSl.push(sl);mVal.push(v);
});
setInterval(function(){
  mSl.forEach(function(sl,i){if(+sl.value!==0){send('/motor?id='+i+'&speed='+sl.value);}});
},400);


$('stopall').onclick=function(){
  mSl.forEach(function(sl,i){sl.value=0;mVal[i].textContent='0';});
  endJoy();rotState.L=false;rotState.R=false;
  send('/stop');
};


fetch('/state').then(function(r){return r.json();}).then(function(s){
  s.servos.forEach(function(a,i){if(sSl[i]){sSl[i].value=a;sVal[i].textContent=a;}});
}).catch(function(){});
</script></body></html>
)rawliteral";


// ---------------- Saved configuration ----------------
void saveConfig() {
  String m = "";
  for (int k = 0; k < 4; k++) m += (char)('0' + wheelMotor[k]);
  uint8_t mask = 0;
  for (int i = 0; i < 4; i++) if (motorReverse[i]) mask |= (1 << i);
  prefs.begin("arme", false);
  prefs.putString("map", m);
  prefs.putUChar("rev", mask);
  prefs.end();
}


void loadConfig() {
  // defaults
  for (int k = 0; k < 4; k++) wheelMotor[k] = DEFAULT_WHEEL_MOTOR[k];
  for (int i = 0; i < 4; i++) motorReverse[i] = DEFAULT_REVERSE[i];


  prefs.begin("arme", true);
  String m = prefs.getString("map", "");
  bool haveRev = prefs.isKey("rev");
  uint8_t mask = prefs.getUChar("rev", 0);
  prefs.end();


  if (m.length() == 4) {
    bool used[4] = {false, false, false, false};
    bool ok = true;
    uint8_t tmp[4];
    for (int k = 0; k < 4; k++) {
      int v = m[k] - '0';
      if (v < 0 || v > 3 || used[v]) { ok = false; break; }
      used[v] = true;
      tmp[k] = (uint8_t)v;
    }
    if (ok) for (int k = 0; k < 4; k++) wheelMotor[k] = tmp[k];
  }
  if (haveRev) {
    for (int i = 0; i < 4; i++) motorReverse[i] = (mask >> i) & 1;
  }
}


// ---------------- Servo helpers ----------------
void writeServo(int ch, float angle) {
  float a = constrain(angle, 0.0f, 180.0f);
  int us = servoMinUs[ch] + (int)((servoMaxUs[ch] - servoMinUs[ch]) * a / 180.0f);
  pwm.writeMicroseconds(ch, us);
}


void tickServos() {
  const float step = SERVO_DEG_PER_SEC * 0.02f;   // per 20 ms tick
  for (int i = 0; i < NUM_SERVOS; i++) {
    float diff = servoTarget[i] - servoCur[i];
    if (fabsf(diff) < 0.01f) continue;
    if (fabsf(diff) <= step) servoCur[i] = servoTarget[i];
    else servoCur[i] += (diff > 0) ? step : -step;
    writeServo(i, servoCur[i]);
  }
}


// ---------------- Motor helpers ----------------
void motorPwmWrite(int i, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(motors[i].pwmPin, duty);
#else
  ledcWrite(motors[i].ledcCh, duty);
#endif
}


// speed: -100..100 (percent). 0 = short brake.
void applyMotor(int i, int speed) {
  const Motor &m = motors[i];
  if (motorReverse[i]) speed = -speed;
  if (speed == 0) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, HIGH);
    motorPwmWrite(i, 0);
    return;
  }
  bool fwd = speed > 0;
  digitalWrite(m.in1, fwd ? HIGH : LOW);
  digitalWrite(m.in2, fwd ? LOW : HIGH);
  uint32_t duty = (uint32_t)(abs(speed) * 255 / 100);
  motorPwmWrite(i, duty);
}


void tickMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    float diff = motorTarget[i] - motorCur[i];
    if (fabsf(diff) < 0.01f) continue;
    if (fabsf(diff) <= MOTOR_RAMP_PER_TICK) motorCur[i] = motorTarget[i];
    else motorCur[i] += (diff > 0) ? MOTOR_RAMP_PER_TICK : -MOTOR_RAMP_PER_TICK;
    applyMotor(i, (int)roundf(motorCur[i]));
  }
}


void stopAllMotorsNow() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motorTarget[i] = 0;
    motorCur[i] = 0;
    applyMotor(i, 0);
  }
}


// ---------------- HTTP handlers ----------------
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }


void handleServo() {
  if (!server.hasArg("ch") || !server.hasArg("angle")) { server.send(400, "text/plain", "missing"); return; }
  int ch = server.arg("ch").toInt();
  int angle = server.arg("angle").toInt();
  if (ch < 0 || ch >= NUM_SERVOS) { server.send(400, "text/plain", "bad channel"); return; }
  servoTarget[ch] = constrain(angle, servoLimLo[ch], servoLimHi[ch]);
  server.send(200, "text/plain", "ok");
}


void handleMotor() {
  if (!server.hasArg("id") || !server.hasArg("speed")) { server.send(400, "text/plain", "missing"); return; }
  int id = server.arg("id").toInt();
  int speed = server.arg("speed").toInt();
  if (id < 0 || id >= NUM_MOTORS) { server.send(400, "text/plain", "bad motor"); return; }
  motorTarget[id] = constrain(speed, -100, 100);
  lastMotorCmd = millis();
  server.send(200, "text/plain", "ok");
}


// Mecanum drive. x = sideways (+ right), y = forward (+ forward), r = rotation (+ clockwise),
// all -100..100. lim = max wheel speed in percent (0..100).
// Directions are relative to the robot's own front (robot-centric).
void handleDrive() {
  if (!server.hasArg("x") || !server.hasArg("y") || !server.hasArg("r")) { server.send(400, "text/plain", "missing"); return; }
  float x = constrain(server.arg("x").toInt(), -100, 100);
  float y = constrain(server.arg("y").toInt(), -100, 100);
  float r = constrain(server.arg("r").toInt(), -100, 100);
  float lim = server.hasArg("lim") ? constrain(server.arg("lim").toInt(), 0, 100) : 100;


  // wheel order: 0 FL, 1 FR, 2 RL, 3 RR
  float w[4] = { y + x + r, y - x - r, y - x + r, y + x - r };
  float m = 0;
  for (int k = 0; k < 4; k++) if (fabsf(w[k]) > m) m = fabsf(w[k]);
  float scale = (m > 100.0f) ? (100.0f / m) : 1.0f;


  for (int k = 0; k < 4; k++) {
    float v = w[k] * scale * lim / 100.0f;
    motorTarget[wheelMotor[k]] = (int)roundf(v);
  }
  lastMotorCmd = millis();
  server.send(200, "text/plain", "ok");
}


void handleStop() {
  stopAllMotorsNow();
  server.send(200, "text/plain", "stopped");
}


void handleConfig() {
  String s = "{\"map\":[";
  for (int k = 0; k < 4; k++) { s += wheelMotor[k]; if (k < 3) s += ","; }
  s += "],\"rev\":[";
  for (int i = 0; i < 4; i++) { s += motorReverse[i] ? 1 : 0; if (i < 3) s += ","; }
  s += "]}";
  server.send(200, "application/json", s);
}


void handleSetConfig() {
  const char* names[4] = {"fl", "fr", "rl", "rr"};
  int m[4];
  bool used[4] = {false, false, false, false};
  for (int k = 0; k < 4; k++) {
    if (!server.hasArg(names[k])) { server.send(400, "text/plain", "missing wheel"); return; }
    int v = server.arg(names[k]).toInt();
    if (v < 0 || v > 3 || used[v]) { server.send(400, "text/plain", "Each motor can only be used once."); return; }
    used[v] = true;
    m[k] = v;
  }
  String rev = server.arg("rev");
  if (rev.length() != 4) { server.send(400, "text/plain", "bad reverse flags"); return; }


  stopAllMotorsNow();
  for (int k = 0; k < 4; k++) wheelMotor[k] = (uint8_t)m[k];
  for (int i = 0; i < 4; i++) motorReverse[i] = (rev[i] == '1');
  saveConfig();
  server.send(200, "text/plain", "Saved.");
}


void handleState() {
  String s = "{\"servos\":[";
  for (int i = 0; i < NUM_SERVOS; i++) { s += servoTarget[i]; if (i < NUM_SERVOS - 1) s += ","; }
  s += "],\"motors\":[";
  for (int i = 0; i < NUM_MOTORS; i++) { s += motorTarget[i]; if (i < NUM_MOTORS - 1) s += ","; }
  s += "]}";
  server.send(200, "application/json", s);
}


void handleNotFound() { server.send(404, "text/plain", "not found"); }


// ---------------- WiFi ----------------
void startWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    staMode = true;
    WiFi.setSleep(false);   // faster response
    Serial.print("Connected. Open http://");
    Serial.println(WiFi.localIP());
  } else {
    staMode = false;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("WiFi failed. Started own network '");
    Serial.print(AP_SSID);
    Serial.print("'. Open http://");
    Serial.println(WiFi.softAPIP());
  }
  if (MDNS.begin(HOSTNAME)) {
    Serial.print("Also reachable at http://");
    Serial.print(HOSTNAME);
    Serial.println(".local");
  }
}


// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(300);


  loadConfig();


  // Keep everything off until configured
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);
  pinMode(PIN_PCA_OE, OUTPUT);
  digitalWrite(PIN_PCA_OE, HIGH);   // servo outputs disabled


  // Motor pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(motors[i].pwmPin, PWM_FREQ_HZ, PWM_RES_BITS);
#else
    ledcSetup(motors[i].ledcCh, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(motors[i].pwmPin, motors[i].ledcCh);
#endif
    motorTarget[i] = 0;
    motorCur[i] = 0;
    applyMotor(i, 0);
  }


  // PCA9685
  Wire.begin(PIN_SDA, PIN_SCL);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);
  delay(10);


  for (int i = 0; i < NUM_SERVOS; i++) {
    servoTarget[i] = constrain(servoInit[i], servoLimLo[i], servoLimHi[i]);
    servoCur[i] = servoTarget[i];
    writeServo(i, servoCur[i]);
  }
  delay(50);
  digitalWrite(PIN_PCA_OE, LOW);    // enable servo outputs
  digitalWrite(PIN_STBY, HIGH);     // enable motor drivers


  startWifi();


  server.on("/", handleRoot);
  server.on("/servo", handleServo);
  server.on("/motor", handleMotor);
  server.on("/drive", handleDrive);
  server.on("/stop", handleStop);
  server.on("/config", handleConfig);
  server.on("/setconfig", handleSetConfig);
  server.on("/state", handleState);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Web server started.");
  lastTick = millis();
}


void loop() {
  server.handleClient();
  uint32_t now = millis();


  if (now - lastTick >= 20) {
    lastTick = now;
    tickServos();
    tickMotors();
  }


  // Failsafe: stop motors if commands stop arriving or WiFi drops
  bool anyMotor = false;
  for (int i = 0; i < NUM_MOTORS; i++) if (motorTarget[i] != 0) anyMotor = true;
  if (anyMotor) {
    bool timedOut = (now - lastMotorCmd) > MOTOR_TIMEOUT_MS;
    bool wifiLost = staMode && WiFi.status() != WL_CONNECTED;
    if (timedOut || wifiLost) stopAllMotorsNow();
  }
}

