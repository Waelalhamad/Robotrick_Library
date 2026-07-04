// =====================================================
//  RobotControl.ino — USB serial bridge for Robotrick
//  Arduino Mega 2560 + Robotrick Mega Shield
//
//  Lets a desktop app drive the robot over USB serial and
//  stream live sensor telemetry.
//
//  ──────────────────────────────────────────────────────
//  SERIAL PROTOCOL  (115200 baud, lines end with "\n")
//  ──────────────────────────────────────────────────────
//  Commands are case-insensitive, tokens space-separated.
//
//  PC -> Robot:
//    PING            -> ACK PING, then DONE PING
//    FWD  <cm>       -> bot.forward(cm)        (blocking)
//    BACK <cm>       -> bot.backward(cm)       (blocking)
//    TL   <deg>      -> bot.turnLeft(deg)      (blocking)
//    TR   <deg>      -> bot.turnRight(deg)     (blocking)
//    STOP            -> bot.stop()
//    CAL             -> bot.calibrateGyro()    (blocking ~3s)
//    CALSPEED        -> bot.calibrateSpeed()   (blocking; needs ~1m clear)
//    LINECAL         -> bot.lineCalibrate(5)   (blocking 5s; wiggle robot over line by hand)
//    LINEJ <n>       -> bot.followLineToJunction(n)  (blocking; DONE on success, ERR if lost)
//    LINECM <cm>     -> bot.followLineForCM(cm)      (blocking; DONE on success, ERR if lost)
//    RESETH          -> bot.resetHeading()
//    MOTORS <l> <r>  -> bot.setMotors(l,r)     (non-blocking raw drive, -255..255)
//    M4 <speed>      -> bot.motor4(speed)      (lift/mechanism motor, -255..255)
//    M4FOR <sp> <ms> -> bot.motor4For(sp,ms)   (run motor 4 for a time, then stop)
//    M4STOP          -> bot.motor4Stop()
//    MAP             -> demo: TL90, FWD30, TL90, FWD50, STOP (blocking)
//    TELEM <0|1>     -> disable/enable telemetry streaming (default ON)
//    GET             -> print one telemetry line immediately
//
//  Robot -> PC:
//    ACK  <CMD>      when a command starts
//    DONE <CMD>      when a blocking command finishes
//    ERR  <message>  on bad / unknown command
//    READY Robotrick v1                      printed once at boot
//    TLM heading=<f1dp> encL=<long> encR=<long> busy=<0|1>
//        streamed every 100 ms while telemetry is ON and no
//        blocking command is executing.
//
//  Notes:
//    - bot.update() is called every loop so heading stays live when idle.
//    - The drive base avoids M4 (encoder on D0/D1 = Serial pins),
//      so Serial at 115200 is safe.
// =====================================================

#include <Robotrick.h>

Robotrick bot;

// ── line parser state ──────────────────────────────────
const uint8_t BUF_SIZE = 48;
char     lineBuf[BUF_SIZE];
uint8_t  lineLen = 0;
bool     overflow = false;

// ── telemetry state ────────────────────────────────────
bool     telemEnabled = true;
uint32_t lastTelem = 0;
const uint32_t TELEM_PERIOD_MS = 100;

// ── busy flag (true only while a blocking command runs) ─
bool busy = false;

// -------------------------------------------------------
void setup() {
  bot.begin(115200);          // inits Serial, motors, encoders, gyro + calibrates
  if (bot.lineLoadCalibration())
    Serial.println(F("LINECAL loaded from EEPROM"));   // معايرة الخط محفوظة
  Serial.println(F("READY Robotrick v1"));
  lastTelem = millis();
}

// -------------------------------------------------------
void loop() {
  bot.update();               // keep heading integrating while idle
  readSerial();
  streamTelemetry();
}

// -------------------------------------------------------
//  Telemetry: one line every TELEM_PERIOD_MS when enabled.
//  (busy commands block loop(), so this never fires during them;
//   busy is reflected correctly around the blocking call anyway.)
// -------------------------------------------------------
void streamTelemetry() {
  if (!telemEnabled) return;
  uint32_t now = millis();
  if (now - lastTelem < TELEM_PERIOD_MS) return;
  lastTelem = now;
  printTelemetry();
}

void printTelemetry() {
  Serial.print(F("TLM heading="));
  Serial.print(bot.heading(), 1);
  Serial.print(F(" encL="));
  Serial.print(bot.encoderLeft());
  Serial.print(F(" encR="));
  Serial.print(bot.encoderRight());
  Serial.print(F(" busy="));
  Serial.println(busy ? 1 : 0);
}

// -------------------------------------------------------
//  Read chars until '\n'. Ignore '\r'. Guard overflow.
// -------------------------------------------------------
void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (overflow) {
        Serial.println(F("ERR line too long"));
      } else if (lineLen > 0) {
        handleLine(lineBuf);
      }
      lineLen = 0;
      overflow = false;
      continue;
    }

    if (lineLen < BUF_SIZE - 1) {
      lineBuf[lineLen++] = c;
    } else {
      overflow = true;       // keep draining until newline, then report
    }
  }
}

// -------------------------------------------------------
//  Parse + dispatch one complete line.
// -------------------------------------------------------
void handleLine(char* line) {
  char* cmd = strtok(line, " \t");
  if (cmd == NULL) return;

  toUpper(cmd);

  // ── PING ──
  if (strcmp(cmd, "PING") == 0) {
    Serial.println(F("ACK PING"));
    Serial.println(F("DONE PING"));
    return;
  }

  // ── FWD <cm> ──
  if (strcmp(cmd, "FWD") == 0) {
    float v;
    if (!nextFloat(&v)) { Serial.println(F("ERR FWD needs <cm>")); return; }
    Serial.println(F("ACK FWD"));
    runBlocking_forward(v);
    Serial.println(F("DONE FWD"));
    return;
  }

  // ── BACK <cm> ──
  if (strcmp(cmd, "BACK") == 0) {
    float v;
    if (!nextFloat(&v)) { Serial.println(F("ERR BACK needs <cm>")); return; }
    Serial.println(F("ACK BACK"));
    runBlocking_backward(v);
    Serial.println(F("DONE BACK"));
    return;
  }

  // ── TL <deg> ──
  if (strcmp(cmd, "TL") == 0) {
    float v;
    if (!nextFloat(&v)) { Serial.println(F("ERR TL needs <deg>")); return; }
    Serial.println(F("ACK TL"));
    runBlocking_turnLeft(v);
    Serial.println(F("DONE TL"));
    return;
  }

  // ── TR <deg> ──
  if (strcmp(cmd, "TR") == 0) {
    float v;
    if (!nextFloat(&v)) { Serial.println(F("ERR TR needs <deg>")); return; }
    Serial.println(F("ACK TR"));
    runBlocking_turnRight(v);
    Serial.println(F("DONE TR"));
    return;
  }

  // ── PIVOT <L|R> <deg> ── (one wheel moves, other stopped; +fwd/-back)
  if (strcmp(cmd, "PIVOT") == 0) {
    char* w = strtok(NULL, " \t");
    float d;
    if (w == NULL || !nextFloat(&d)) { Serial.println(F("ERR PIVOT needs <L|R> <deg>")); return; }
    Serial.println(F("ACK PIVOT"));
    busy = true;
    bot.pivot(w[0], d);
    busy = false;
    Serial.println(F("DONE PIVOT"));
    return;
  }

  // ── STOP ── (non-blocking)
  if (strcmp(cmd, "STOP") == 0) {
    Serial.println(F("ACK STOP"));
    bot.stop();
    return;
  }

  // ── CAL ── (blocking ~3s)
  if (strcmp(cmd, "CAL") == 0) {
    Serial.println(F("ACK CAL"));
    busy = true;
    bot.calibrateGyro();
    busy = false;
    Serial.println(F("DONE CAL"));
    return;
  }

  // ── CALSPEED ── (blocking; needs ~1m clear space in front)
  if (strcmp(cmd, "CALSPEED") == 0) {
    Serial.println(F("ACK CALSPEED"));
    busy = true;
    bot.calibrateSpeed();       // prints RT_PWM_STATIC and RT_KV
    busy = false;
    Serial.println(F("DONE CALSPEED"));
    return;
  }

  // ── LINECAL ── (blocking 5s; wiggle robot over the line by hand)
  if (strcmp(cmd, "LINECAL") == 0) {
    Serial.println(F("ACK LINECAL"));
    busy = true;
    bot.lineCalibrate(5);
    busy = false;
    Serial.println(F("DONE LINECAL"));
    return;
  }

  // ── LINEJ <n> ── (blocking; follow line to nth junction)
  if (strcmp(cmd, "LINEJ") == 0) {
    long n;
    if (!nextLong(&n) || n < 1) { Serial.println(F("ERR LINEJ needs <n> >= 1")); return; }
    Serial.println(F("ACK LINEJ"));
    busy = true;
    bool ok = bot.followLineToJunction((uint8_t)n);
    busy = false;
    if (ok) Serial.println(F("DONE LINEJ"));
    else    Serial.println(F("ERR LINEJ lost or timeout"));
    return;
  }

  // ── LINECM <cm> ── (blocking; follow line a set distance)
  if (strcmp(cmd, "LINECM") == 0) {
    float v;
    if (!nextFloat(&v)) { Serial.println(F("ERR LINECM needs <cm>")); return; }
    Serial.println(F("ACK LINECM"));
    busy = true;
    bool ok = bot.followLineForCM(v);
    busy = false;
    if (ok) Serial.println(F("DONE LINECM"));
    else    Serial.println(F("ERR LINECM lost or timeout"));
    return;
  }

  // ── QTRRAW ── (one raw sensor dump — no calibration needed)
  if (strcmp(cmd, "QTRRAW") == 0) {
    uint16_t v[RT_QTR_N];
    bot.lineReadRaw(v);
    uint32_t sum = 0;
    Serial.print(F("QTRRAW "));
    for (uint8_t i = 0; i < RT_QTR_N; i++) {
      Serial.print(v[i]);
      if (i < RT_QTR_N - 1) Serial.print(',');
      sum += v[i];
    }
    Serial.print(F(" sum=")); Serial.println(sum);
    return;
  }

  // ── QTRDBG ── (calibrated values + line position)
  if (strcmp(cmd, "QTRDBG") == 0) {
    uint16_t v[RT_QTR_N];
    uint16_t pos = bot.lineRead(v);
    if (pos == 65535) { Serial.println(F("ERR QTRDBG not calibrated — run LINECAL")); return; }
    uint32_t sum = 0;
    Serial.print(F("QTRDBG "));
    for (uint8_t i = 0; i < RT_QTR_N; i++) {
      Serial.print(v[i]);
      if (i < RT_QTR_N - 1) Serial.print(',');
      sum += v[i];
    }
    Serial.print(F(" pos=")); Serial.print(pos);
    Serial.print(F(" sum=")); Serial.println(sum);
    return;
  }

  // ── M4 <speed> ── (non-blocking raw drive of motor 4, -255..255)
  if (strcmp(cmd, "M4") == 0) {
    long s;
    if (!nextLong(&s)) { Serial.println(F("ERR M4 needs <speed>")); return; }
    bot.motor4((int)s);
    Serial.println(F("ACK M4"));
    return;
  }

  // ── M4FOR <speed> <ms> ── (run motor 4 for a time, then stop; blocking)
  if (strcmp(cmd, "M4FOR") == 0) {
    long s, ms;
    if (!nextLong(&s) || !nextLong(&ms)) { Serial.println(F("ERR M4FOR needs <speed> <ms>")); return; }
    Serial.println(F("ACK M4FOR"));
    busy = true;
    bot.motor4For((int)s, (uint32_t)ms);
    busy = false;
    Serial.println(F("DONE M4FOR"));
    return;
  }

  // ── M4STOP ──
  if (strcmp(cmd, "M4STOP") == 0) {
    bot.motor4Stop();
    Serial.println(F("ACK M4STOP"));
    return;
  }

  // ── RESETH ── (non-blocking)
  if (strcmp(cmd, "RESETH") == 0) {
    Serial.println(F("ACK RESETH"));
    bot.resetHeading();
    return;
  }

  // ── MOTORS <l> <r> ── (non-blocking raw drive)
  if (strcmp(cmd, "MOTORS") == 0) {
    long l, r;
    if (!nextLong(&l) || !nextLong(&r)) {
      Serial.println(F("ERR MOTORS needs <l> <r>"));
      return;
    }
    Serial.println(F("ACK MOTORS"));
    bot.setMotors((int)l, (int)r);
    return;
  }

  // ── MAP ── (blocking demo)
  if (strcmp(cmd, "MAP") == 0) {
    Serial.println(F("ACK MAP"));
    busy = true;
    bot.turnLeft(90);
    bot.forward(30);
    bot.turnLeft(90);
    bot.forward(50);
    bot.stop();
    busy = false;
    Serial.println(F("DONE MAP"));
    return;
  }

  // ── TELEM <0|1> ──
  if (strcmp(cmd, "TELEM") == 0) {
    long v;
    if (!nextLong(&v)) { Serial.println(F("ERR TELEM needs <0|1>")); return; }
    telemEnabled = (v != 0);
    Serial.println(F("ACK TELEM"));
    return;
  }

  // ── GET ── (one telemetry line now)
  if (strcmp(cmd, "GET") == 0) {
    printTelemetry();
    return;
  }

  // ── unknown ──
  Serial.print(F("ERR unknown command: "));
  Serial.println(cmd);
}

// -------------------------------------------------------
//  Blocking wrappers — set busy around each blocking call.
// -------------------------------------------------------
void runBlocking_forward(float cm)  { busy = true; bot.forward(cm);   busy = false; }
void runBlocking_backward(float cm) { busy = true; bot.backward(cm);  busy = false; }
void runBlocking_turnLeft(float d)  { busy = true; bot.turnLeft(d);   busy = false; }
void runBlocking_turnRight(float d) { busy = true; bot.turnRight(d);  busy = false; }

// -------------------------------------------------------
//  Token helpers — operate on strtok's running state.
// -------------------------------------------------------
bool nextFloat(float* out) {
  char* tok = strtok(NULL, " \t");
  if (tok == NULL) return false;
  *out = atof(tok);
  return true;
}

bool nextLong(long* out) {
  char* tok = strtok(NULL, " \t");
  if (tok == NULL) return false;
  *out = atol(tok);
  return true;
}

void toUpper(char* s) {
  for (; *s; s++) *s = toupper((unsigned char)*s);
}
