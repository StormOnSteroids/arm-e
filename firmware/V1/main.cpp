// ESP32 + PCA9685 6-servo WiFi controller
// Libraries (Arduino Library Manager): "Adafruit PWM Servo Driver Library"
// Board: "ESP32 Dev Module"
//
// Wiring:
//   ESP32 3V3 -> PCA9685 VCC   (logic power)
//   ESP32 GND -> PCA9685 GND   (also join the servo supply GND here)
//   ESP32 GPIO21 -> PCA9685 SDA
//   ESP32 GPIO22 -> PCA9685 SCL
//   External 5-6V supply -> PCA9685 V+ terminal block (servo power, NOT from the ESP32)
//   Servos on PCA9685 channels 0-5


#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


const char* WIFI_SSID = "rapid";
const char* WIFI_PASS = "4Fagro@2023";


const int NUM_SERVOS = 6;


// Pulse range in PCA9685 ticks at 50 Hz (4096 ticks per 20 ms).
// ~150 = 0.6 ms, ~600 = 2.4 ms. Tune per servo so they don't hit end stops.
const int SERVO_MIN = 150;
const int SERVO_MAX = 600;


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
WebServer server(80);


int angles[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};


void setServo(int ch, int angle) {
  angle = constrain(angle, 0, 180);
  angles[ch] = angle;
  int ticks = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(ch, 0, ticks);
}


const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Servo Control</title>
<style>
  body { font-family: sans-serif; max-width: 480px; margin: 20px auto; padding: 0 16px; background:#111; color:#eee; }
  h2 { text-align:center; }
  .row { margin: 18px 0; }
  .row label { display:flex; justify-content:space-between; margin-bottom:6px; }
  input[type=range] { width:100%; }
  button { width:100%; padding:12px; font-size:16px; margin-top:10px; border:0; border-radius:8px; background:#3a7; color:#fff; }
</style>
</head>
<body>
<h2>ESP32 Servo Control</h2>
<div id="sliders"></div>
<button onclick="centerAll()">Center all (90°)</button>
<script>
const N = 6;
const box = document.getElementById('sliders');
let pending = {};


for (let i = 0; i < N; i++) {
  box.insertAdjacentHTML('beforeend',
    `<div class="row">
       <label>Servo i<spanid="v{i}">90°</span></label>
       <input type="range" id="s${i}" min="0" max="180" value="90">
     </div>`);
  document.getElementById('s' + i).addEventListener('input', e => {
    const a = e.target.value;
    document.getElementById('v' + i).textContent = a + '°';
    send(i, a);
  });
}


// Throttle: at most one request per servo every 40 ms, always sending the latest value
function send(ch, angle) {
  if (pending[ch]) { pending[ch].latest = angle; return; }
  pending[ch] = { latest: null };
  fetch(`/set?ch=ch&angle={angle}`).catch(()=>{});
  setTimeout(() => {
    const l = pending[ch].latest;
    pending[ch] = null;
    if (l !== null) send(ch, l);
  }, 40);
}


function centerAll() {
  for (let i = 0; i < N; i++) {
    document.getElementById('s' + i).value = 90;
    document.getElementById('v' + i).textContent = '90°';
    fetch(`/set?ch=${i}&angle=90`);
  }
}


// Load current positions on page open
fetch('/state').then(r => r.json()).then(a => {
  a.forEach((v, i) => {
    document.getElementById('s' + i).value = v;
    document.getElementById('v' + i).textContent = v + '°';
  });
});
</script>
</body>
</html>
)rawliteral";


void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}


void handleSet() {
  if (!server.hasArg("ch") || !server.hasArg("angle")) {
    server.send(400, "text/plain", "missing ch or angle");
    return;
  }
  int ch = server.arg("ch").toInt();
  int angle = server.arg("angle").toInt();
  if (ch < 0 || ch >= NUM_SERVOS) {
    server.send(400, "text/plain", "bad channel");
    return;
  }
  setServo(ch, angle);
  server.send(200, "text/plain", "ok");
}


void handleState() {
  String json = "[";
  for (int i = 0; i < NUM_SERVOS; i++) {
    json += String(angles[i]);
    if (i < NUM_SERVOS - 1) json += ",";
  }
  json += "]";
  server.send(200, "application/json", json);
}


void setup() {
  Serial.begin(115200);


  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // analog servos: 50 Hz
  delay(10);


  for (int i = 0; i < NUM_SERVOS; i++) setServo(i, 90);


  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Open this in your browser: http://");
  Serial.println(WiFi.localIP());


  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/state", handleState);
  server.begin();
}


void loop() {
  server.handleClient();
}



[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
    adafruit/Adafruit PWM Servo Driver Library @ ^3.0.2
monitor_speed = 115200


