#include "MultiMAX6675.h"

int evapPWRPin = 22;
int upPWRPin = 18;
int downPWRPin = 4;
int fanRelayPin = 16;
int pressurePin = 35;

int evapPWRRating = 168;
int reactorPWRRating = 470;
int halfZonePWRRating = reactorPWRRating / 2;
bool evapStable = false;
bool constantPower = false;
bool oneZone = false; // Måste ändras manuellt

unsigned long lastSent = 0;
unsigned long lastPID = 0;

struct PIDController {
  float kP, kI, kD;
  float integral = 0;
  float lastMeasurement = NAN;
  unsigned long lastTime = 0;
  float integralLimit = 100;

  float compute(float error, float measurement) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (lastTime == 0 || dt <= 0) {lastTime = now; lastMeasurement = measurement; return 0; }
    lastTime = now;
    integral = constrain(integral + error * dt, -integralLimit, integralLimit);
    float derivative = 0;
    if (!isnan(lastMeasurement)) derivative = -(measurement - lastMeasurement) / dt;
    lastMeasurement = measurement;
    return kP * error + kI * integral + kD * derivative;
  }

  void reset() { integral = 0; lastMeasurement = NAN; lastTime = 0; }
};
PIDController meanPID = {2.5, 0.07, 3.5};
PIDController deltaPID = {1.0, 0.07, 0};

struct opParams {
  bool valid;
  int stage;
  String msg;
  float refEvap;
  float refReactor = -1;
  float evapTemp;
  float upTemp;
  float downTemp;
  float evapPWR;
  float upPWR;
  float downPWR;
  bool fanOn;
  float pressure;
  float runTime;
  float deltaTemp;
  float avgTemp;
  float evapStableTemp = NAN;
  float rawEvapTemp;
  float rawUpTemp;
  float rawDownTemp;
};
opParams op;

enum processState {
  START_STANDBY = 1,
  PREHEAT_STANDBY = 4,
  PREHEAT = 5,
  OPERATION = 6,
  END_SUMMARY = 7,
};
processState currentState = START_STANDBY;

MultiMAX6675 tc;

void setup() {
  pinMode(evapPWRPin, OUTPUT);
  pinMode(upPWRPin, OUTPUT);
  pinMode(downPWRPin, OUTPUT);
  pinMode(fanRelayPin, OUTPUT);
  pinMode(pressurePin, INPUT);

  digitalWrite(evapPWRPin, HIGH);
  digitalWrite(upPWRPin, HIGH);
  digitalWrite(downPWRPin, HIGH);

  SPI.begin(33, 26);  // 33-CLKPin, 26-MISOPin
  tc.bind(27, &op.rawEvapTemp, &SPI);  // 27-evapTempPin
  tc.bind(25, &op.rawUpTemp, &SPI);  // 25-upTempPin
  tc.bind(32, &op.rawDownTemp, &SPI);  // 32-downTempPin
  
  Serial.begin(115200);
  Serial.setTimeout(10);
}

void loop() {
  switch (currentState) {
    case START_STANDBY: StartStandby(); break;
    case PREHEAT: Preheat(); break;
    case OPERATION: Operation(); break;
  }
  delay(5);
}

void GetStatus() {
  op.valid = false;
  if (!Serial.available()) return;
  String msg = Serial.readStringUntil('\n');
  msg.trim();
  if (!msg.startsWith("<") || !msg.endsWith(">")) return;
  int s = 1, e;
  #define NEXT(delim) e = msg.indexOf(delim, s); if (e == -1) return; 
  NEXT(';') op.stage = msg.substring(s, e).toInt(); s = e+1;
  NEXT(';') op.refEvap = msg.substring(s, e).toFloat(); s = e+1;
  NEXT(';') op.refReactor = msg.substring(s, e).toFloat(); s = e+1;
  NEXT('>') constantPower = msg.substring(s, e).toInt() == 1; s = e+1;
  op.valid = true;
}

void GetPressure() {
  int samples = 10;
  long sum = 0;
  for (int i = 0; i < samples; i++) {sum += analogRead(pressurePin); delay(5); }
  float voltage = (sum / samples) * 3.3 / 4095.0;
  op.pressure = voltage * 79.52 - 14;
}

bool IsCancel() {
  if (op.stage == -1) {
    op.msg = "Canceled by User";
    currentState = END_SUMMARY;
    SendStatus();
    currentState = START_STANDBY;
    return true; 
  }
  return false;
}

bool IsOverpressure() {
  if (op.pressure > 50) {
    op.msg = "Overpressure ("+String(op.pressure,1)+" kPa)";
    currentState = END_SUMMARY;
    SendStatus();
    currentState = START_STANDBY;
    return true;
  }
  return false;
}

bool IsOvertemp() {
  float maxReactorTemp = max({op.upTemp, op.downTemp});
  bool reactorHot = ((!constantPower && maxReactorTemp > op.refReactor*1.5 && op.refReactor >= 50.0) || maxReactorTemp > 450);
  bool evapHot = (op.evapTemp > 110);
  if (reactorHot || evapHot) {
    float temp = reactorHot ? maxReactorTemp : op.evapTemp;
    String source = reactorHot ? "Reactor" : "Evaporator";
    op.msg = source+" too Hot ("+String(temp,1)+" C)";
    currentState = END_SUMMARY;
    SendStatus();
    currentState = START_STANDBY;
    return true;
  }
  else return false;
}

void SendStatus() {
  op.runTime++;
  Serial.println("<" + 
    String(currentState) + ";" +
    String(op.msg) + ";" +
    String(op.evapTemp) + ";" +
    String(op.pressure) + ";" +
    String(op.upTemp) + ";" +
    String(op.downTemp) + ";" +
    String(100*op.evapPWR/evapPWRRating) + ";" +
    String(100*op.upPWR/(halfZonePWRRating)) + ";" +
    String(100*op.downPWR/(halfZonePWRRating)) + ";" +
    String(op.fanOn) + ";" +
    String(op.runTime) + ">");
}

void CheckEvapStable(bool reset = false) {
  static float buf[30];
  static uint8_t head = 0, count = 0;
  static unsigned long lastSample = 0;
  if (reset) {head = 0; count = 0; lastSample = 0; return; }
  if (millis() - lastSample < 3500) return;
  lastSample = millis();
  buf[head] = op.evapTemp;
  head = (head + 1) % 30;
  if (count < 30) {count++; return; }
  float lo = buf[0], hi = buf[0];
  for (uint8_t i = 1; i < 30; i++) {
    if (buf[i] < lo) lo = buf[i];
    if (buf[i] > hi) hi = buf[i];
  }
  if ((hi - lo) < 2.1 && lo > 50) {
    op.evapStableTemp = (lo + hi) / 2;
    op.evapPWR = 10;
  }
}

void CorrectTemps() {
  if (op.rawEvapTemp != 0) op.evapTemp = op.rawEvapTemp * 1.58 - 14.4;
  if (op.rawUpTemp != 0) op.upTemp   = op.rawUpTemp * 1.65 - 17.8;
  if (op.rawDownTemp != 0) op.downTemp = op.rawDownTemp * 1.15 - 5.5;
}

bool ProcessLoop() {
  op.avgTemp = (op.upTemp + op.downTemp) / 2;
  op.deltaTemp = op.upTemp - op.downTemp;
  GetStatus();
  GetPressure();
  tc.loop();
  CorrectTemps();
  unsigned long now = millis();
  if (now - lastSent >= 1000) {lastSent = now; SendStatus(); }
  if (IsCancel()) return false;
  if (IsOverpressure()) return false;
  if (IsOvertemp()) return false;
  if (!constantPower && now - lastPID >= 100) {lastPID = now; ReactorPID(); }
  PWM(evapPWRPin, op.evapPWR/evapPWRRating);
  PWM(upPWRPin, op.upPWR/(halfZonePWRRating));
  PWM(downPWRPin, op.downPWR/(halfZonePWRRating));
  return true;
}

void StartStandby() {
  op.evapStableTemp = NAN;
  op.refReactor = -1;
  op.msg = "";
  op.fanOn = false;
  CheckEvapStable(true);
  meanPID.reset();
  deltaPID.reset();
  digitalWrite(evapPWRPin, HIGH);
  digitalWrite(upPWRPin, HIGH);
  digitalWrite(downPWRPin, HIGH);
  digitalWrite(fanRelayPin, LOW);
  while (true) {
    GetStatus();
    if (op.valid && op.stage == PREHEAT_STANDBY) break;
    delay(5);
  }
  while (op.evapTemp == 0 || op.upTemp == 0 || op.downTemp == 0) {tc.loop(); CorrectTemps(); delay(5); }
  if (constantPower) op.upPWR = op.downPWR = op.refReactor/2;
  op.runTime = 0;
  currentState = PREHEAT;
  return;
}

void Preheat() {
  if (constantPower) op.evapPWR = op.refEvap;
  else op.evapPWR = evapPWRRating;
  op.fanOn = true;
  digitalWrite(fanRelayPin, HIGH);
  while (true) {
    if (!ProcessLoop()) return;
    if (isnan(op.evapStableTemp)) CheckEvapStable();
    else if (op.avgTemp > op.refReactor*0.95 || constantPower) {currentState = OPERATION; return;}
    delay(5);
  }
} 

void Operation() {
  op.evapPWR = op.refEvap;
  while (true) {
    if (!ProcessLoop()) return;
    if (op.evapTemp > 1.1*op.evapStableTemp) {
      int totalSeconds = op.runTime;
      int hours = totalSeconds / 3600;
      int minutes = (totalSeconds % 3600) / 60;
      int seconds = totalSeconds % 60;
      String time = (hours < 10 ? "0" : "") + String(hours) + ":" +
                    (minutes < 10 ? "0" : "") + String(minutes) + ":" +
                    (seconds < 10 ? "0" : "") + String(seconds);
      op.msg = "Completed in " + time;
      currentState = END_SUMMARY;
      SendStatus();
      currentState = START_STANDBY;
      return;
    }
    delay(5);
  }
}

void PWM(int pin, float duty) {
  static unsigned long cycleStart[40] = {};
  unsigned long now = millis();
  if (now - cycleStart[pin] >= 5000) cycleStart[pin] = now;
  duty = constrain(duty, 0.0, 1.0);
  digitalWrite(pin, (now - cycleStart[pin]) < (unsigned long)(5000 * duty) ? LOW : HIGH);
}

void ReactorPID() {
  float basePID = meanPID.compute(op.refReactor - op.avgTemp, op.avgTemp);
  float basePWR = constrain(basePID, 0, halfZonePWRRating);
  if (oneZone) { op.upPWR = op.downPWR = basePWR; return; }
  float trimPID = deltaPID.compute(op.deltaTemp, op.deltaTemp);
  float trimPWR = constrain(trimPID, -halfZonePWRRating, halfZonePWRRating);
  op.upPWR = constrain(basePWR - trimPWR, 0, halfZonePWRRating);
  op.downPWR = constrain(basePWR + trimPWR, 0, halfZonePWRRating);
}