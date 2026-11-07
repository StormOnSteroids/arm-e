// ARM-E: WiFi web control for 6 servos (PCA9685) and 4 DC motors (2x TB6612FNG)
// + mecanum joystick + camera tracking over ESP-NOW + AI detection + OLED.
// Board: ESP32 Dev Module (classic ESP32). Framework: Arduino (PlatformIO).
//
// Open http://<ip in Serial Monitor> or http://arme.local
// If WiFi can't be joined in 15 s, it makes its own network ARM-E-Robot / robot1234
// (page at http://192.168.4.1)
//
// OLED (SSD1306 128x64, I2C): VCC->3V3, GND->GND, SDA->GPIO21, SCL->GPIO22

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_now.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ====================== SETTINGS YOU EDIT ======================

// ---- WiFi (2.4 GHz only) ----
const char* WIFI_SSID = "rapid";
const char* WIFI_PASS = "4Fagro@2023";
const char* AP_SSID   = "ARM-E-Robot";
const char* AP_PASS   = "robot1234";
const char* HOSTNAME  = "arme";

// ---- I2C / PCA9685 / OLED ----
#define PIN_SDA    21
#define PIN_SCL    22
#define PIN_PCA_OE 5
#define PIN_STBY   4
#define OLED_ADDR  0x3C      // try 0x3D if the screen stays blank

// ---- Servos (PCA9685 channels 0..5) ----
const int NUM_SERVOS = 6;
int servoMinUs[NUM_SERVOS] = {600, 600, 600, 600, 600, 600};
int servoMaxUs[NUM_SERVOS] = {2400, 2400, 2400, 2400, 2400, 2400};
int servoLimLo[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};
int servoLimHi[NUM_SERVOS] = {180, 180, 180, 180, 180, 180};
int servoInit[NUM_SERVOS]  = {90, 90, 90, 90, 90, 90};
const float SERVO_DEG_PER_SEC = 90.0f;

// ---- Tracking ----
const int   PAN_SERVO  = 0;        // turns the camera left/right
const int   TILT_SERVO = 1;        // tilts the camera up/down (use wrist pitch if the camera is on the wrist)
const float PAN_DIR  = -1.0f;      // flip sign if the camera pans AWAY from the target
const float TILT_DIR =  1.0f;      // flip sign if it tilts the wrong way
const float PAN_DEG_PER_SEC  = 60.0f;
const float TILT_DEG_PER_SEC = 45.0f;
const float DEADBAND = 0.10f;      // ignore errors under 10% of half-frame
const float TURN_KP  = 90.0f;      // body rotation: % power at full left/right offset
const float TURN_MIN = 28.0f;      // wheels stall below this
const float TURN_MAX = 60.0f;      // cap rotation speed
const uint32_t TRACK_STALE_MS = 400;

// ---- Motors ----
struct Motor {
  uint8_t in1;
  uint8_t in2;
  uint8_t pwmPin;
  uint8_t ledcCh;
};
const int NUM_MOTORS = 4;
Motor motors[NUM_MOTORS] = {
  { 13, 14, 16, 0 },
  { 17, 18, 19, 1 },
  { 23, 25, 26, 2 },
  { 27, 32, 33, 3 },
};
const uint8_t DEFAULT_WHEEL_MOTOR[4] = {0, 1, 2, 3};
const bool    DEFAULT_REVERSE[4]     = {false, true, false, true};

const int      PWM_FREQ_HZ         = 20000;
const int      PWM_RES_BITS        = 8;
const float    MOTOR_RAMP_PER_TICK = 8.0f;
const uint32_t MOTOR_TIMEOUT_MS    = 1500;

// ================================================================

// ---------- ESP-NOW messages (identical on the camera board) ----------
enum : uint8_t { MSG_TRACK = 1, MSG_CMD = 2 };
enum : uint8_t { CMD_PICK = 1, CMD_CLEAR = 2, CMD_TOL = 3 };

typedef struct __attribute__((packed)) {
  uint8_t type, x, y, size, found;
  uint8_t ip[4];
} TrackMsg;

typedef struct __attribute__((packed)) {
  uint8_t type, cmd, a, b;
} CmdMsg;

enum Mode : uint8_t { MODE_MANUAL = 0, MODE_BODY = 1, MODE_AIM = 2 };

Adafruit_PWMServoDriver pwm(0x40);
WebServer server(80);
Preferences prefs;

float servoCur[NUM_SERVOS];
int   servoTarget[NUM_SERVOS];
float motorCur[NUM_MOTORS];
int   motorTarget[NUM_MOTORS];

uint8_t wheelMotor[4];
bool    motorReverse[4];

uint32_t lastMotorCmd = 0;
uint32_t lastTick = 0;
bool staMode = false;

// tracking state
Mode mode = MODE_MANUAL;
float panF = 90, tiltF = 90;
volatile uint8_t trkX = 0, trkY = 0, trkSize = 0, trkFound = 0;
volatile uint32_t trkStamp = 0;
uint8_t camIp[4] = {0, 0, 0, 0};
uint8_t camMac[6];
volatile bool camMacNew = false;
bool camKnown = false;

// OLED + AI detection
Adafruit_SSD1306 oled(128, 64, &Wire, -1, 400000UL, 400000UL);
bool oledOk = false;
bool oledDirty = true;
uint32_t lastOled = 0;
bool aiSource = false;          // true = browser AI detection drives tracking
char aiName[24] = "";
int  aiPct = 0;
uint32_t aiStamp = 0;

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
input[type=text]{flex:1;padding:8px;background:#2d3a4a;color:#fff;border:0;border-radius:8px;font-size:14px;-webkit-user-select:text;user-select:text}
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
.stream{position:relative;max-width:480px;margin:6px auto;border:2px solid #2d3a4a;border-radius:10px;overflow:hidden;min-height:90px;background:#000}
#cam{width:100%;display:block;cursor:crosshair}
#ov,#ov2{position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none}
</style></head><body>
<h1>ARM-E Control</h1>
<button class="stop" id="stopall">STOP ALL MOTORS</button>

<h2>Camera tracking</h2>
<div class="card">
  <div class="stream"><img id="cam" alt=""><canvas id="ov"></canvas><canvas id="ov2"></canvas></div>
  <div class="note" id="tst">Waiting for camera...</div>
  <div class="turn" id="modes">
    <button data-m="0">Manual</button>
    <button data-m="1">Turn body</button>
    <button data-m="2">Aim camera</button>
  </div>
  <div class="turn" id="srcs">
    <button id="srcBlob" class="on">Colour blob</button>
    <button id="srcAI">AI detect</button>
  </div>
  <div class="row"><label>Detect</label><input type="text" id="cls" value="person"></div>
  <div class="note"><b id="aiinfo"></b> Blank = biggest object of any type. Uses COCO names such as person, cup, bottle, cell phone, chair.</div>
  <div class="row"><label>Tolerance</label><input type="range" id="tol" min="2" max="20" value="7"><span class="val" id="tolv">7</span></div>
  <button id="clr">Clear target</button>
  <div class="note">Colour blob: click an object in the video to track it. <b>Turn body</b> rotates the robot to centre the target. <b>Aim camera</b> moves the pan/tilt servos. Drive controls are ignored outside Manual.</div>
</div>

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
  <div class="note">Angle 0 = straight ahead, 90 = right, 180 = back, 270 = left, relative to the way the robot is facing. Keyboard: W A S D to move, &larr; &rarr; (or Q E) to rotate.</div>
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
const pad=$('pad'),knob=$('knob');
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
  $('spd').textContent=Math.round(mag*(+$('maxsp').value));
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

// ---------- Camera tracking ----------
const cam=$('cam'),ov=$('ov'),tst=$('tst');
cam.crossOrigin='anonymous';
let camIp='',curMode=0,polling=false;
const modeBtns=Array.prototype.slice.call(document.querySelectorAll('#modes button'));
function setModeUI(m){curMode=m;modeBtns.forEach(function(b){b.classList.toggle('on',+b.dataset.m===m);});}
modeBtns.forEach(function(b){b.onclick=function(){send('/tmode?m='+b.dataset.m);setModeUI(+b.dataset.m);};});
$('clr').onclick=function(){send('/tclear');};
$('tol').addEventListener('input',function(){$('tolv').textContent=this.value;});
$('tol').addEventListener('input',throttle(function(){send('/ttol?v='+$('tol').value);},100));
cam.addEventListener('click',function(e){
  const r=cam.getBoundingClientRect();
  const x=Math.round((e.clientX-r.left)/r.width*255),y=Math.round((e.clientY-r.top)/r.height*255);
  send('/pick?x='+x+'&y='+y);
});
cam.onerror=function(){setTimeout(function(){if(camIp)cam.src='http://'+camIp+':81/stream?'+Date.now();},1500);};

function poll(){
  if(polling)return;polling=true;
  fetch('/tstate').then(function(r){return r.json();}).then(function(s){
    if(s.cam&&s.cam!==camIp){camIp=s.cam;cam.src='http://'+camIp+':81/stream';}
    if(!s.cam)tst.textContent='Waiting for camera...';
    setModeUI(s.mode);
    const w=ov.width=cam.clientWidth,h=ov.height=cam.clientHeight;
    const c=ov.getContext('2d');c.clearRect(0,0,w,h);
    c.strokeStyle='rgba(255,255,255,0.25)';c.lineWidth=1;
    c.beginPath();c.moveTo(w/2,0);c.lineTo(w/2,h);c.moveTo(0,h/2);c.lineTo(w,h/2);c.stroke();
    if(s.found){
      const px=s.x/255*w,py=s.y/255*h;
      c.strokeStyle='#00ff88';c.lineWidth=2;c.beginPath();
      c.arc(px,py,12,0,6.283);
      c.moveTo(px-18,py);c.lineTo(px+18,py);c.moveTo(px,py-18);c.lineTo(px,py+18);c.stroke();
      tst.textContent='Target x='+s.x+' y='+s.y+' size='+s.size;
    }else if(s.cam){tst.textContent='No target';}
    if(s.mode===2&&s.servos){
      s.servos.forEach(function(a,i){if(sSl[i]&&document.activeElement!==sSl[i]){sSl[i].value=a;sVal[i].textContent=a;}});
    }
  }).catch(function(){}).finally(function(){polling=false;});
}
setInterval(poll,150);

// ---------- AI detection (COCO-SSD, runs in this browser) ----------
const ov2=$('ov2'),o2=ov2.getContext('2d'),work=document.createElement('canvas'),wctx=work.getContext('2d');
const aiinfo=$('aiinfo');
let aiModel=null,aiOn=false,aiRun=false,lastDet=0;
function loadScript(src){return new Promise(function(ok,bad){const s=document.createElement('script');s.src=src;s.onload=ok;s.onerror=bad;document.head.appendChild(s);});}
function srcUI(on){aiOn=on;$('srcBlob').classList.toggle('on',!on);$('srcAI').classList.toggle('on',on);send('/tsrc?v='+(on?1:0));}
async function setSrc(on){
  srcUI(on);
  if(!on){o2.clearRect(0,0,ov2.width,ov2.height);aiinfo.textContent='';return;}
  if(!aiModel){
    aiinfo.textContent='Loading AI model (this device needs internet)...';
    try{
      await loadScript('https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@4.22.0');
      await loadScript('https://cdn.jsdelivr.net/npm/@tensorflow-models/coco-ssd@2.2.3');
      aiModel=await cocoSsd.load({base:'lite_mobilenet_v2'});
    }catch(e){srcUI(false);aiinfo.textContent='Model failed to load - is this device online?';return;}
  }
  if(aiOn&&!aiRun)aiLoop();
}
async function aiLoop(){
  aiRun=true;
  while(aiOn){
    if(!cam.naturalWidth){await new Promise(function(r){setTimeout(r,100);});continue;}
    const W=cam.naturalWidth,H=cam.naturalHeight;
    work.width=ov2.width=W;work.height=ov2.height=H;
    wctx.drawImage(cam,0,0,W,H);
    let preds=[];
    try{preds=await aiModel.detect(work,10,0.5);}
    catch(e){aiinfo.textContent='Cannot read video pixels: '+e.message;await new Promise(function(r){setTimeout(r,1000);});continue;}
    if(!aiOn)break;
    const want=$('cls').value.trim().toLowerCase();
    o2.clearRect(0,0,W,H);o2.font='14px sans-serif';o2.lineWidth=2;
    let best=null;
    preds.forEach(function(p){
      const b=p.bbox,isT=!want||p.class===want;
      o2.strokeStyle=o2.fillStyle=isT?'#0f0':'#888';
      o2.strokeRect(b[0],b[1],b[2],b[3]);
      o2.fillText(p.class+' '+Math.round(p.score*100)+'%',b[0]+3,b[1]+15);
      const a=b[2]*b[3];
      if(isT&&(!best||a>best.a))best={a:a,cls:p.class,pct:Math.round(p.score*100),cx:b[0]+b[2]/2,cy:b[1]+b[3]/2};
    });
    const now=Date.now();
    if(best){
      o2.fillStyle='#f00';o2.beginPath();o2.arc(best.cx,best.cy,5,0,6.283);o2.fill();
      aiinfo.textContent=best.cls+' '+best.pct+'%';
      if(now-lastDet>100){lastDet=now;send('/det?f=1&n='+encodeURIComponent(best.cls)+'&p='+best.pct+'&x='+Math.round(best.cx/W*255)+'&y='+Math.round(best.cy/H*255));}
    }else{
      aiinfo.textContent='Nothing matching';
      if(now-lastDet>100){lastDet=now;send('/det?f=0');}
    }
  }
  aiRun=false;
}
$('srcBlob').onclick=function(){setSrc(false);};
$('srcAI').onclick=function(){setSrc(true);};
setSrc(false);   // resets the board's source on page load

$('stopall').onclick=function(){
  mSl.forEach(function(sl,i){sl.value=0;mVal[i].textContent='0';});
  endJoy();rotState.L=false;rotState.R=false;
  send('/stop');setModeUI(0);
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
  const float step = SERVO_DEG_PER_SEC * 0.02f;
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

// Mecanum mix. x = sideways (+ right), y = forward, r = rotation (+ clockwise), -100..100.
// Wheel order: 0 FL, 1 FR, 2 RL, 3 RR.
void mecanumMix(float x, float y, float r, float lim) {
  float w[4] = { y + x + r, y - x - r, y - x + r, y + x - r };
  float m = 0;
  for (int k = 0; k < 4; k++) if (fabsf(w[k]) > m) m = fabsf(w[k]);
  float scale = (m > 100.0f) ? (100.0f / m) : 1.0f;
  for (int k = 0; k < 4; k++) {
    float v = w[k] * scale * lim / 100.0f;
    motorTarget[wheelMotor[k]] = (int)roundf(v);
  }
}

// ---------------- ESP-NOW ----------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *src = info->src_addr;
#else
void onRecv(const uint8_t *src, const uint8_t *data, int len) {
#endif
  if (len != sizeof(TrackMsg) || data[0] != MSG_TRACK) return;
  const TrackMsg *t = (const TrackMsg *)data;
  if (!aiSource) {   // blob data is ignored while AI detection is the source
    trkX = t->x; trkY = t->y; trkSize = t->size; trkFound = t->found;
    trkStamp = millis();
  }
  memcpy(camIp, t->ip, 4);
  if (!camKnown && !camMacNew) { memcpy(camMac, src, 6); camMacNew = true; }
}

void initEspNow() {
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); return; }
  esp_now_register_recv_cb(onRecv);
  Serial.printf("ESP-NOW ready, channel %d\n", WiFi.channel());
}

void sendCmd(uint8_t cmd, uint8_t a, uint8_t b) {
  if (!camKnown) return;
  CmdMsg m = { MSG_CMD, cmd, a, b };
  esp_now_send(camMac, (uint8_t *)&m, sizeof(m));
}

bool trackFresh() {
  return trkFound && (millis() - trkStamp) < TRACK_STALE_MS;
}

// ---------------- Tracking modes ----------------
void setMode(Mode m) {
  oledDirty = true;
  mode = m;
  stopAllMotorsNow();
  panF  = servoTarget[PAN_SERVO];
  tiltF = servoTarget[TILT_SERVO];
}

void trackControl(float dt) {
  if (mode == MODE_MANUAL) return;
  bool fresh = trackFresh();
  float ex = fresh ? ((int)trkX - 127.5f) / 127.5f : 0;   // -1 left .. +1 right
  float ey = fresh ? ((int)trkY - 127.5f) / 127.5f : 0;   // -1 top  .. +1 bottom

  if (mode == MODE_BODY) {
    if (fresh && fabsf(ex) > DEADBAND) {
      float r = constrain(TURN_KP * ex, -TURN_MAX, TURN_MAX);
      if (fabsf(r) < TURN_MIN) r = (r > 0) ? TURN_MIN : -TURN_MIN;
      mecanumMix(0, 0, r, 100);        // target on the right -> rotate clockwise
      lastMotorCmd = millis();
    } else {
      mecanumMix(0, 0, 0, 100);
    }
  } else if (mode == MODE_AIM) {
    if (fresh) {
      if (fabsf(ex) > DEADBAND)
        panF = constrain(panF + PAN_DIR * PAN_DEG_PER_SEC * ex * dt, (float)servoLimLo[PAN_SERVO], (float)servoLimHi[PAN_SERVO]);
      if (fabsf(ey) > DEADBAND)
        tiltF = constrain(tiltF + TILT_DIR * TILT_DEG_PER_SEC * ey * dt, (float)servoLimLo[TILT_SERVO], (float)servoLimHi[TILT_SERVO]);
      servoTarget[PAN_SERVO]  = (int)lroundf(panF);
      servoTarget[TILT_SERVO] = (int)lroundf(tiltF);
    }
  }
}

// ---------------- OLED ----------------
void drawOled() {
  static const char* modeTxt[] = {"Manual", "Turn body", "Aim cam"};
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  if (!aiSource) {
    oled.setTextSize(1); oled.setCursor(0, 0);
    oled.println("AI detect is off");
    oled.println("Pick 'AI detect' on");
    oled.println("the web page.");
  } else if (aiName[0]) {
    oled.setTextSize(strlen(aiName) <= 10 ? 2 : 1);
    oled.setCursor(0, 0); oled.print(aiName);
    oled.setTextSize(3); oled.setCursor(0, 26);
    oled.print(aiPct); oled.print('%');
  } else {
    oled.setTextSize(1); oled.setCursor(0, 0);
    oled.print("Nothing detected");
  }
  oled.setTextSize(1); oled.setCursor(0, 56);
  oled.print(modeTxt[mode]);
  oled.display();
}

// ---------------- HTTP handlers ----------------
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleServo() {
  if (!server.hasArg("ch") || !server.hasArg("angle")) { server.send(400, "text/plain", "missing"); return; }
  int ch = server.arg("ch").toInt();
  int angle = server.arg("angle").toInt();
  if (ch < 0 || ch >= NUM_SERVOS) { server.send(400, "text/plain", "bad channel"); return; }
  servoTarget[ch] = constrain(angle, servoLimLo[ch], servoLimHi[ch]);
  if (ch == PAN_SERVO)  panF  = servoTarget[ch];
  if (ch == TILT_SERVO) tiltF = servoTarget[ch];
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

void handleDrive() {
  if (!server.hasArg("x") || !server.hasArg("y") || !server.hasArg("r")) { server.send(400, "text/plain", "missing"); return; }
  if (mode != MODE_MANUAL) { server.send(200, "text/plain", "ignored (tracking mode)"); return; }
  float x = constrain(server.arg("x").toInt(), -100, 100);
  float y = constrain(server.arg("y").toInt(), -100, 100);
  float r = constrain(server.arg("r").toInt(), -100, 100);
  float lim = server.hasArg("lim") ? constrain(server.arg("lim").toInt(), 0, 100) : 100;
  mecanumMix(x, y, r, lim);
  lastMotorCmd = millis();
  server.send(200, "text/plain", "ok");
}

void handleStop() {
  setMode(MODE_MANUAL);
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

// ---- tracking endpoints ----
void handleTMode() {
  int m = server.arg("m").toInt();
  if (m < 0 || m > 2) { server.send(400, "text/plain", "bad mode"); return; }
  setMode((Mode)m);
  server.send(200, "text/plain", "ok");
}

void handlePick() {
  if (!camKnown) { server.send(503, "text/plain", "camera not seen yet"); return; }
  sendCmd(CMD_PICK, constrain(server.arg("x").toInt(), 0, 255), constrain(server.arg("y").toInt(), 0, 255));
  server.send(200, "text/plain", "ok");
}

void handleTClear() {
  sendCmd(CMD_CLEAR, 0, 0);
  trkFound = 0;
  server.send(200, "text/plain", "ok");
}

void handleTTol() {
  sendCmd(CMD_TOL, constrain(server.arg("v").toInt(), 2, 25), 0);
  server.send(200, "text/plain", "ok");
}

void handleTState() {
  bool fresh = trackFresh();
  char ip[20] = "";
  if (camIp[0] | camIp[1] | camIp[2] | camIp[3]) snprintf(ip, sizeof(ip), "%d.%d.%d.%d", camIp[0], camIp[1], camIp[2], camIp[3]);
  String s = "{\"mode\":" + String((int)mode) + ",\"found\":" + String(fresh ? 1 : 0) +
             ",\"x\":" + String((int)trkX) + ",\"y\":" + String((int)trkY) + ",\"size\":" + String((int)trkSize) +
             ",\"cam\":\"" + String(ip) + "\",\"servos\":[";
  for (int i = 0; i < NUM_SERVOS; i++) { s += servoTarget[i]; if (i < NUM_SERVOS - 1) s += ","; }
  s += "]}";
  server.send(200, "application/json", s);
}

// ---- AI detection endpoints (browser -> board) ----
void handleDet() {   // best detection: name, percent, centre x/y 0-255
  if (server.arg("f").toInt()) {
    String n = server.arg("n");
    strncpy(aiName, n.c_str(), sizeof(aiName) - 1);
    aiName[sizeof(aiName) - 1] = 0;
    aiPct = constrain(server.arg("p").toInt(), 0, 100);
    aiStamp = millis();
    if (aiSource) {
      trkX = constrain(server.arg("x").toInt(), 0, 255);
      trkY = constrain(server.arg("y").toInt(), 0, 255);
      trkFound = 1;
      trkStamp = millis();
    }
  } else {
    aiName[0] = 0;
    if (aiSource) trkFound = 0;
  }
  oledDirty = true;
  server.send(200, "text/plain", "ok");
}

void handleTSrc() {
  aiSource = server.arg("v").toInt() == 1;
  trkFound = 0;
  aiName[0] = 0;
  oledDirty = true;
  server.send(200, "text/plain", "ok");
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
    WiFi.setSleep(false);
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

  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);
  pinMode(PIN_PCA_OE, OUTPUT);
  digitalWrite(PIN_PCA_OE, HIGH);

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

  Wire.begin(PIN_SDA, PIN_SCL);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);

  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false);   // false = don't re-init Wire
  if (oledOk) {
    oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2); oled.setCursor(0, 0); oled.println("ARM-E");
    oled.setTextSize(1); oled.println("Starting...");
    oled.display();
  } else {
    Serial.println("OLED not found (check wiring, or try address 0x3D)");
  }
  delay(10);

  for (int i = 0; i < NUM_SERVOS; i++) {
    servoTarget[i] = constrain(servoInit[i], servoLimLo[i], servoLimHi[i]);
    servoCur[i] = servoTarget[i];
    writeServo(i, servoCur[i]);
  }
  panF  = servoTarget[PAN_SERVO];
  tiltF = servoTarget[TILT_SERVO];
  delay(50);
  digitalWrite(PIN_PCA_OE, LOW);
  digitalWrite(PIN_STBY, HIGH);

  startWifi();
  initEspNow();

  server.on("/", handleRoot);
  server.on("/servo", handleServo);
  server.on("/motor", handleMotor);
  server.on("/drive", handleDrive);
  server.on("/stop", handleStop);
  server.on("/config", handleConfig);
  server.on("/setconfig", handleSetConfig);
  server.on("/state", handleState);
  server.on("/tmode", handleTMode);
  server.on("/pick", handlePick);
  server.on("/tclear", handleTClear);
  server.on("/ttol", handleTTol);
  server.on("/tstate", handleTState);
  server.on("/det", handleDet);
  server.on("/tsrc", handleTSrc);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Web server started.");
  lastTick = millis();
}

void loop() {
  server.handleClient();
  uint32_t now = millis();

  // Register the camera as an ESP-NOW peer the first time we hear from it
  if (camMacNew && !camKnown) {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, camMac, 6);
    p.channel = 0;
    p.ifidx   = staMode ? WIFI_IF_STA : WIFI_IF_AP;
    p.encrypt = false;
    if (esp_now_add_peer(&p) == ESP_OK) { camKnown = true; Serial.println("Camera found over ESP-NOW"); }
  }

  if (now - lastTick >= 20) {
    lastTick = now;
    trackControl(0.02f);
    tickServos();
    tickMotors();
  }

  if (aiName[0] && now - aiStamp > 1500) { aiName[0] = 0; oledDirty = true; }   // detection went stale
  if (oledOk && oledDirty && now - lastOled >= 150) {
    lastOled = now; oledDirty = false;
    drawOled();
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