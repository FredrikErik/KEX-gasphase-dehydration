#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();
int evapPower = -1;
int reactorTemp = -1;
volatile int moveDir = 0;
bool constantPower = false;
bool showCancel = false;
bool showDiagnostics = false;

enum uiState {
  START_MENU,
  EVAP_POWER,
  REACTOR_TEMP,
  CONFIRM_START,
  PREHEAT_STANDBY,
  PREHEAT,
  OPERATION,
  END_SUMMARY,
};
uiState currentState = START_MENU;

struct opData {
  bool valid;
  int stage;
  String msg;
  float evapTemp;
  float pressure;
  float upTemp;
  float downTemp;
  float evapP;
  float upP;
  float downP;
  int fanOn;
  float runTime;
};
opData od;

bool btnReleased(int pin) {
  static bool lastState[40];
  static bool init = false;
  if (!init) { memset(lastState, HIGH, sizeof(lastState)); init = true; }
  bool cur = digitalRead(pin);
  bool released = (lastState[pin] == LOW && cur == HIGH);
  lastState[pin] = cur;
  return released;
}

void IRAM_ATTR encoderISR() {
  static uint8_t lastState = 0;
  uint8_t a = digitalRead(TFT_A);
  uint8_t b = digitalRead(TFT_B);
  uint8_t state = (a << 1) | b;

  if      (lastState == 0b10 && state == 0b00) moveDir--;
  else if (lastState == 0b00 && state == 0b01) moveDir--;
  else if (lastState == 0b01 && state == 0b11) moveDir--;
  else if (lastState == 0b11 && state == 0b10) moveDir--;

  else if (lastState == 0b01 && state == 0b00) moveDir++;
  else if (lastState == 0b00 && state == 0b10) moveDir++;
  else if (lastState == 0b10 && state == 0b11) moveDir++;
  else if (lastState == 0b11 && state == 0b01) moveDir++;

  lastState = state;
}

void setup() {
  pinMode(TFT_A, INPUT);  // Encoder vridsignal
  pinMode(TFT_B, INPUT);  // Encoder vridsignal
  pinMode(TFT_PUSH, INPUT_PULLUP);  // Encoder (bekräfta) knapp
  pinMode(TFT_K0, INPUT_PULLUP);  // Nedre (tillbaka) knapp
  attachInterrupt(digitalPinToInterrupt(TFT_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(TFT_B), encoderISR, CHANGE);

  tft.init();
  tft.setRotation(1);

  Serial.begin(115200);
  Serial.setTimeout(10);
}

void loop() {
  switch (currentState) {
    case START_MENU:
      StartMenu();
      break;

    case EVAP_POWER:
      EvapPowerAdj();
      break;

    case REACTOR_TEMP:
      ReactorTempAdj();
      break;

    case CONFIRM_START:
      ConfirmStart();
      break;

    case PREHEAT_STANDBY:
      PreheatStandby();
      break;

    case PREHEAT:
      Preheat();
      break;

    case OPERATION:
      Operation();
      break;

    case END_SUMMARY:
      EndSummary();
      break;
  }
  delay(5);
  moveDir = 0;
}

void waitForRelease() {
  while (digitalRead(TFT_PUSH) == LOW || digitalRead(TFT_K0) == LOW) delay(5);
}

void StartMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  int selMenuIdx = 1;
  drawStartMenuItems(selMenuIdx);
  unsigned long pressTime = 0;
  unsigned long lastMoveTime = 0;

  while (true) {
    if (abs(moveDir) >= 3) {
      if (millis() - lastMoveTime >= 100) {
        selMenuIdx += (moveDir > 0 ? 1 : -1);
        lastMoveTime = millis();
        if (selMenuIdx < 1) selMenuIdx = 1;
        if (selMenuIdx > 3) selMenuIdx = 3;
        drawStartMenuItems(selMenuIdx);
      }
      moveDir = 0;
    }
    if (digitalRead(TFT_K0) == LOW) {
      if (pressTime == 0) pressTime = millis();
      if (millis() - pressTime > 2000) {
        constantPower = !constantPower;
        tft.fillScreen(TFT_BLACK);
        drawStartMenuItems(selMenuIdx);
        pressTime = 0;
        waitForRelease();
      }
    }
    else pressTime = 0;
    if (digitalRead(TFT_PUSH) == LOW) break;
    delay(5);
  }
  if (selMenuIdx == 1) currentState = CONFIRM_START;
  else if (selMenuIdx == 2) currentState = EVAP_POWER;
  else if (selMenuIdx == 3) currentState = REACTOR_TEMP;
  waitForRelease();
  return;
}

void drawStartMenuItems(int selMenuIdx) {
  if (selMenuIdx == 1) tft.setTextColor(TFT_BLACK, TFT_WHITE);
  else tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(" Start Process ", tft.width()/2, tft.height()/4, 4);

  if (selMenuIdx == 2) tft.setTextColor(TFT_BLACK, TFT_WHITE);
  else tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(" Evaporation Power ", tft.width()/2, tft.height()/2, 4);

  if (selMenuIdx == 3) tft.setTextColor(TFT_BLACK, TFT_WHITE);
  else tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (constantPower) tft.drawString(" Reactor Power ", tft.width()/2, tft.height()*3/4, 4);
  else tft.drawString(" Reactor Temperature ", tft.width()/2, tft.height()*3/4, 4);
}

void EvapPowerAdj() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Evaporation Power:", tft.width()/2, tft.height()/4, 4);

  int selPower = 0;
  if (evapPower == -1) selPower = 100;
  else selPower = evapPower;

  auto drawBar = [&](int v) {
    int w = tft.width(), h = tft.height();
    int x = 20, y = h/2, bw = w-40, bh = 20, fw = map(v, 0, 200, 0, bw-2);

    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("     " + String(v) + " W", w/2 + 30, h*3/4, 4);
    tft.drawRect(x, y, bw, bh, TFT_WHITE);
    tft.fillRect(x+1, y+1, fw, bh-2, TFT_WHITE);
    tft.fillRect(x+1+fw, y+1, bw-2-fw, bh-2, TFT_BLACK);
  };
  drawBar(selPower);

  while (true) {
    if (moveDir != 0) {
      selPower += moveDir;
      moveDir = 0;
      if (selPower < 0)   selPower = 0;
      if (selPower > 200) selPower = 200;
      drawBar(selPower);
    }
    if (digitalRead(TFT_K0) == LOW || digitalRead(TFT_PUSH) == LOW) {
      if (digitalRead(TFT_PUSH) == LOW) evapPower = selPower;
      currentState = START_MENU;
      waitForRelease();
      return;
    }
  }
}

void ReactorTempAdj() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  if (constantPower) tft.drawString("Reactor Power:", tft.width()/2, tft.height()/4, 4);
  else tft.drawString("Reactor Temperature:", tft.width()/2, tft.height()/4, 4);

  int selTemp = 0;
  if (reactorTemp == -1) selTemp = 250;
  else selTemp = reactorTemp;

  auto drawBar = [&](int v) {
    int w = tft.width(), h = tft.height();
    int x = 20, y = h/2, bw = w-40, bh = 20, fw = map(v, 0, 400, 0, bw-2);

    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    if (constantPower) tft.drawString("     " + String(v) + " W", w/2 + 30, h*3/4, 4);
    else tft.drawString("     " + String(v) + " C", w/2 + 30, h*3/4, 4);
    tft.drawRect(x, y, bw, bh, TFT_WHITE);
    tft.fillRect(x+1, y+1, fw, bh-2, TFT_WHITE);
    tft.fillRect(x+1+fw, y+1, bw-2-fw, bh-2, TFT_BLACK);
  };
  drawBar(selTemp);

  while (true) {
    if (moveDir != 0) {
      selTemp += moveDir;
      moveDir = 0;
      if (selTemp < 0) selTemp = 0;
      if (selTemp > 400) selTemp = 400;
      drawBar(selTemp);
    }
    if (digitalRead(TFT_K0) == LOW || digitalRead(TFT_PUSH) == LOW) {
      if (digitalRead(TFT_PUSH) == LOW) reactorTemp = selTemp;
      currentState = START_MENU;
      waitForRelease();
      return;
    }
  }
}

void ConfirmStart() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  if (evapPower == -1 || reactorTemp == -1) {
    tft.drawString("Provide Operating", tft.width()/2, tft.height()/2 - 20, 4);
    tft.drawString("Parameters to Continue", tft.width()/2, tft.height()/2 + 20, 4);
    while (digitalRead(TFT_PUSH) != LOW && digitalRead(TFT_K0) != LOW) delay(5);  // Loopar tills bekräfta eller back-knappen trycks
    currentState = START_MENU;
    waitForRelease();
    return;
  }
  else {
    tft.drawString("Apply Operating", tft.width()/2, tft.height()/3 - 20, 4);
    tft.drawString("Parameters and Start?", tft.width()/2, tft.height()/3 + 20, 4);
    tft.drawString("Evaporation Power: " + String(evapPower) + " W", tft.width()/2, tft.height()*2/3 - 15, 2);
    if (constantPower) tft.drawString("Reactor Power: " + String(reactorTemp) + " W", tft.width()/2, tft.height()*2/3 + 15, 2);
    else tft.drawString("Reactor Temperature: " + String(reactorTemp) + " C", tft.width()/2, tft.height()*2/3 + 15, 2);
    while (true) {
      if (digitalRead(TFT_PUSH) == LOW) {currentState = PREHEAT_STANDBY; break; }
      if (digitalRead(TFT_K0) == LOW) {currentState = START_MENU; break; }
      delay(5);
    }
    waitForRelease();
    return;
  }
}

void drawCancel() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Hold Confirm to", tft.width()/2, tft.height()/3 - 20, 4);
  tft.drawString("Cancel Operation", tft.width()/2, tft.height()/3 + 20, 4);
}

void GetOpData() {
  od.valid = false;
  if (!Serial.available()) return;
  String msg = Serial.readStringUntil('\n');
  msg.trim();
  if (!msg.startsWith("<") || !msg.endsWith(">")) return;

  int s = 1, e;
  #define NEXT(delim) e = msg.indexOf(delim, s); if (e == -1) return; 
  NEXT(';') od.stage = msg.substring(s, e).toInt(); s = e+1;
  NEXT(';') od.msg = msg.substring(s, e); s = e+1;
  NEXT(';') od.evapTemp = msg.substring(s, e).toFloat(); s = e+1;
  NEXT(';') od.pressure = msg.substring(s, e).toFloat(); s = e+1;
  NEXT(';') od.upTemp = msg.substring(s, e).toFloat(); s = e+1;
  NEXT(';') od.downTemp = msg.substring(s, e).toFloat(); s = e+1;
  NEXT(';') od.evapP = msg.substring(s, e).toFloat(); s = e+1;
  NEXT(';') od.upP = msg.substring(s, e).toFloat(); s = e+1;
  NEXT(';') od.downP = msg.substring(s, e).toFloat(); s = e+1;
  NEXT(';') od.fanOn = msg.substring(s, e).toInt(); s = e+1;
  NEXT('>') od.runTime = msg.substring(s, e).toFloat();
  od.valid = true;
}

void PreheatStandby() {
  Serial.println("<"+String(currentState)+";"+String(evapPower)+";"+String(reactorTemp)+";"+String(constantPower)+">");
  while (true) {
    GetOpData();
    if (od.valid && od.stage == PREHEAT) break;
    delay(5);
  }
  currentState = PREHEAT;
  return;
}

void Preheat() {
  bool needsDraw = true;
  unsigned long pressTime = 0;

  auto drawPreheat = [&]() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Heating in Progress", tft.width()/2, tft.height()/6, 4);
    tft.setTextDatum(MR_DATUM);
    tft.drawString("Evaporator:", tft.width()/2, tft.height()*2/4, 4);
    tft.drawString("Reactor:", tft.width()/2, tft.height()*3/4, 4);
  };
  auto updatePreheat = [&]() {
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("   " + String(od.evapTemp, 1) + " C", tft.width()/2 + 115, tft.height()*2/4, 4);
    tft.drawString("   " + String((od.upTemp + od.downTemp)/2, 1) + " C", tft.width()/2 + 115, tft.height()*3/4, 4);
  }; 
  while (true) {
    GetOpData();
    if (od.valid) {
      if (od.stage == OPERATION) {currentState = OPERATION; return; }
      else if (showDiagnostics) updateDiagnostics();
      else if (!showCancel && od.stage == PREHEAT) updatePreheat();
      else if (od.stage == END_SUMMARY) {currentState = END_SUMMARY; return; }
    }
    if (needsDraw) {
      if (showCancel) drawCancel();
      else if (showDiagnostics) {drawDiagnostics(); updateDiagnostics(); }
      else {drawPreheat(); updatePreheat(); }
      needsDraw = false;
    }
    if (showCancel) {
      if (btnReleased(TFT_K0)) {
        showCancel = false; needsDraw = true;
        pressTime = 0; btnReleased(TFT_PUSH);
      }
      else if (digitalRead(TFT_PUSH) == LOW) {
        if (pressTime == 0) pressTime = millis();
        int elapsedTime = millis() - pressTime;
        int w = tft.width(), h = tft.height();
        int x = 20, bw = w-40, bh = 20, fw = map(elapsedTime, 0, 2000, 0, bw-2);
        tft.drawRect(x, h*2/3, bw, bh, TFT_WHITE);
        tft.fillRect(x+1, h*2/3+1, fw, bh-2, TFT_WHITE);
        tft.fillRect(x+1+fw, h*2/3+1, bw-2-fw, bh-2, TFT_BLACK);
        if (elapsedTime > 2000) {
          Serial.println("<-1;0;0;0>");
          while (true) {
            GetOpData();
            if (od.valid && od.stage == END_SUMMARY) {currentState = END_SUMMARY; waitForRelease(); return; }
            delay(5);
          }
        }
      }
      else {
        if (pressTime != 0) tft.fillRect(0, tft.height()*2/3, tft.width(), tft.height()/3, TFT_BLACK);
        pressTime = 0;
      }
    }
    else if (showDiagnostics) {
      if (btnReleased(TFT_K0)) {
        showDiagnostics = false; needsDraw = true; 
        pressTime = 0; btnReleased(TFT_PUSH);
      }
    }
    else {
      if (btnReleased(TFT_K0)) {
        showCancel = true; needsDraw = true;
        pressTime = 0; btnReleased(TFT_PUSH);
      }
      else if (btnReleased(TFT_PUSH)) {
        showDiagnostics = true; needsDraw = true;
        pressTime = 0; btnReleased(TFT_K0);
      }
    }
    delay(5);
  }
}

void Operation() {
  bool needsDraw = true;
  unsigned long pressTime = 0;

  auto drawOperation = [&]() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Operation Overview", tft.width()/2, tft.height()/6, 4);
    tft.setTextDatum(MR_DATUM);
    tft.drawString("Evaporator:", tft.width()/2, tft.height()*2/4, 4);
    tft.drawString("Reactor:", tft.width()/2, tft.height()*3/4, 4);
  };
  auto updateOperation = [&]() {
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("   " + String(od.evapTemp, 1) + " C", tft.width()/2 + 115, tft.height()*2/4, 4);
    tft.drawString("   " + String((od.upTemp + od.downTemp)/2, 1) + " C", tft.width()/2 + 115, tft.height()*3/4, 4);
  };
  while (true) {
    GetOpData();
    if (od.valid) {
      if (od.stage == END_SUMMARY) {currentState = END_SUMMARY; return; }
      else if (showDiagnostics) updateDiagnostics();
      else if (!showCancel && od.stage == OPERATION) updateOperation();
    }
    if (needsDraw) {
      if (showCancel) drawCancel();
      else if (showDiagnostics) {drawDiagnostics(); updateDiagnostics(); }
      else {drawOperation(); updateOperation(); }
      needsDraw = false;
    }
    if (showCancel) {
      if (btnReleased(TFT_K0)) {
        showCancel = false; needsDraw = true;
        pressTime = 0; btnReleased(TFT_PUSH);
      }
      else if (digitalRead(TFT_PUSH) == LOW) {
        if (pressTime == 0) pressTime = millis();
        int elapsedTime = millis() - pressTime;
        int w = tft.width(), h = tft.height();
        int x = 20, bw = w-40, bh = 20, fw = map(elapsedTime, 0, 2000, 0, bw-2);
        tft.drawRect(x, h*2/3, bw, bh, TFT_WHITE);
        tft.fillRect(x+1, h*2/3+1, fw, bh-2, TFT_WHITE);
        tft.fillRect(x+1+fw, h*2/3+1, bw-2-fw, bh-2, TFT_BLACK);
        if (elapsedTime > 2000) {
          Serial.println("<-1;0;0;0>");
          while (true) {
            GetOpData();
            if (od.valid && od.stage == END_SUMMARY) {currentState = END_SUMMARY; waitForRelease(); return; }
            delay(5);
          }
        }
      }
      else {
        if (pressTime != 0) tft.fillRect(0, tft.height()*2/3, tft.width(), tft.height()/3, TFT_BLACK);
        pressTime = 0;
      }
    }
    else if (showDiagnostics) {
      if (btnReleased(TFT_K0)) {
        showDiagnostics = false; needsDraw = true; 
        pressTime = 0; btnReleased(TFT_PUSH);
      }
    }
    else {
      if (btnReleased(TFT_K0)) {
        showCancel = true; needsDraw = true;
        pressTime = 0; btnReleased(TFT_PUSH);
      }
      else if (btnReleased(TFT_PUSH)) {
        showDiagnostics = true; needsDraw = true;
        pressTime = 0; btnReleased(TFT_K0);
      }
    }
    delay(5);
  }
}

void drawDiagnostics() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Diagnostic View", tft.width()/2, tft.height()/6, 4);
  tft.setTextDatum(MR_DATUM);
  int w = tft.width()/3+10, h = tft.height()/11, a = -30, b = 0;
  tft.drawString("stage", w+a, h*3, 2);
  tft.drawString("msg", w*2+b, h*3, 2);
  tft.drawString("evapTemp", w+a, h*4, 2);
  tft.drawString("refEvap", w*2+b, h*4, 2);
  tft.drawString("evapPWR", w+a, h*5, 2);
  tft.drawString("pressure", w*2+b, h*5, 2);
  tft.drawString("upTemp", w+a, h*6, 2);
  tft.drawString("downTemp", w*2+b, h*6, 2);
  tft.drawString("avgTemp", w+a, h*7, 2);
  tft.drawString("deltaTemp", w*2+b, h*7, 2);
  tft.drawString("upPWR", w+a, h*8, 2);
  tft.drawString("downPWR", w*2+b, h*8, 2);
  tft.drawString("refReactor", w+a, h*9, 2);
  tft.drawString("fanOn", w*2+b, h*9, 2);
  tft.drawString("runTime", tft.width()/2-5, h*10+5, 2);
}

void updateDiagnostics() {
  String stage = "";
  if (od.stage == PREHEAT) stage = "Preheat";
  else if (od.stage == OPERATION)  stage = "Operation";
  int totalSeconds = (int)od.runTime;
  int hours = totalSeconds / 3600;
  int minutes = (totalSeconds % 3600) / 60;
  int seconds = totalSeconds % 60;
  String time = (hours < 10 ? "0" : "") + String(hours) + ":" +
                (minutes < 10 ? "0" : "") + String(minutes) + ":" +
                (seconds < 10 ? "0" : "") + String(seconds);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(ML_DATUM);
  int w = tft.width()/3+10, h = tft.height()/11, a = -20, b = 10;
  tft.drawString(stage+"    ", w+a, h*3, 2);
  tft.drawString(od.msg+"    ", w*2+b, h*3, 2);
  tft.drawString(String(od.evapTemp, 1)+" C   ", w+a, h*4, 2);
  tft.drawString(String(float(evapPower), 1)+" W   ", w*2+b, h*4, 2);
  tft.drawString(String(od.evapP, 1)+" %    ", w+a, h*5, 2);
  tft.drawString(String(od.pressure, 1)+" kPa    ", w*2+b, h*5, 2);
  tft.drawString(String(od.upTemp, 1)+" C    ", w+a, h*6, 2);
  tft.drawString(String(od.downTemp, 1)+" C   ", w*2+b, h*6, 2);
  tft.drawString(String((od.upTemp + od.downTemp)/2, 1)+" C   ", w+a, h*7, 2);
  tft.drawString(String((od.upTemp - od.downTemp), 1)+" C   ", w*2+b, h*7, 2);
  tft.drawString(String(od.upP, 1)+" %    ", w+a, h*8, 2);
  tft.drawString(String(od.downP, 1)+" %   ", w*2+b, h*8, 2);
  if (constantPower) tft.drawString(String(float(reactorTemp), 1)+" W   ", w+a, h*9, 2);
  else tft.drawString(String(float(reactorTemp), 1)+" C   ", w+a, h*9, 2);
  tft.drawString(String(od.fanOn)+"    ", w*2+b, h*9, 2);
  tft.drawString(time+"   ", tft.width()/2+5, h*10+5, 2);
}

void EndSummary() {
  showCancel = false;
  showDiagnostics = false;
  unsigned long pressTime = 0;
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Operation Ended", tft.width()/2, tft.height()/3 - 20, 4);
  tft.drawString(od.msg, tft.width()/2, tft.height()/3 + 20, 4);
  while (true) {
    if (digitalRead(TFT_PUSH) == LOW) {
      if (pressTime == 0) pressTime = millis();
      int elapsedTime = millis() - pressTime;
      int w = tft.width(), h = tft.height();
      int x = 20, bw = w-40, bh = 20, fw = map(elapsedTime, 0, 2000, 0, bw-2);
      tft.drawRect(x, h*2/3, bw, bh, TFT_WHITE);
      tft.fillRect(x+1, h*2/3+1, fw, bh-2, TFT_WHITE);
      tft.fillRect(x+1+fw, h*2/3+1, bw-2-fw, bh-2, TFT_BLACK);
      if (elapsedTime > 2000) break;
    }
    else if (pressTime != 0) {
      tft.fillRect(0, tft.height()*2/3, tft.width(), tft.height()/3, TFT_BLACK);
      pressTime = 0;
    }
    delay(5);
  }
  currentState = START_MENU;
  waitForRelease();
  return;
}