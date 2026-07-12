// =====================================================
//  Robotrick.cpp — implementation
//  كل المعايرة في Robotrick.h — هون المنطق فقط.
// =====================================================
#include "Robotrick.h"

// L3GD20H registers
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1    0x20
#define REG_CTRL4    0x23
#define REG_OUT_X_L  0x28
#define WHO_AM_I_VAL 0xD7

Robotrick::Robotrick()
    : _encL(RT_ENC_L_A, RT_ENC_L_B),
      _encR(RT_ENC_R_A, RT_ENC_R_B),
      _gzBias(0), _heading(0), _gyroRate(0), _lastMicros(0),
      _driveSpeed(RT_STRAIGHT_SPEED), _driveAccel(RT_STRAIGHT_ACCEL),
      _distKp(RT_DIST_KP), _distKd(RT_DIST_KD), _distKi(RT_DIST_KI),
      _hdgKp(RT_STRAIGHT_KP), _hdgKd(RT_STRAIGHT_KD), _hdgDeadband(RT_STRAIGHT_DEADBAND),
      _lineBase(RT_LINE_BASE), _lineMax(RT_LINE_MAX), _lineMin(RT_LINE_MIN),
      _lineSlow(RT_LINE_SLOW), _lineDeadband(RT_LINE_DEADBAND),
      _lineKp(RT_LINE_KP), _lineKd(RT_LINE_KD), _lineKpBoost(RT_LINE_KP_BOOST),
      _turnFast(RT_TURN_FAST), _turnSlowMax(RT_TURN_SLOW_MAX), _turnSlowMin(RT_TURN_SLOW_MIN),
      _turnKp(RT_TURN_KP), _turnFastDeg(RT_TURN_FAST_DEG),
      _pivotFast(_pivotFast), _pivotSlow(_pivotSlow), _pivotMin(_pivotMin) {}

// ─────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────
bool Robotrick::begin(uint32_t baud) {
    if (baud) Serial.begin(baud);
    Wire.begin();

    pinMode(RT_L_PWM, OUTPUT); pinMode(RT_L_DIR, OUTPUT); pinMode(RT_L_EN, OUTPUT);
    pinMode(RT_R_PWM, OUTPUT); pinMode(RT_R_DIR, OUTPUT); pinMode(RT_R_EN, OUTPUT);
    digitalWrite(RT_L_EN, HIGH);   // فعّل الدرايفرات
    digitalWrite(RT_R_EN, HIGH);
    stop();

    // Motor 4 (رافعة/آلية)
    pinMode(RT_M4_PWM, OUTPUT); pinMode(RT_M4_DIR, OUTPUT); pinMode(RT_M4_EN, OUTPUT);
    digitalWrite(RT_M4_EN, HIGH);
    analogWrite(RT_M4_PWM, 0);

    _servoBegin();   // سجّل بنات السيرفو (يتفعّلوا أول ما تستعملهم)
    _colorBegin();   // بنات + مراجع حساسات الألوان (بعد Wire.begin)

    bool ok = _gyroInit();
    if (!ok) {
        Serial.println(F("[Robotrick] ERROR: gyro not found! check wiring (0x6B)"));
        return false;
    }
    delay(200);          // warm-up
    calibrateGyro();
    Serial.println(F("[Robotrick] ready."));
    return true;
}

void Robotrick::calibrateGyro() {
    Serial.print(F("[Robotrick] calibrating gyro (keep STILL)"));
    int32_t sum = 0;
    int16_t mn = 32767, mx = -32768;
    for (int i = 0; i < 300; i++) {
        int16_t g = _readGz();
        sum += g;
        if (g < mn) mn = g;
        if (g > mx) mx = g;
        if (i % 30 == 0) Serial.print('.');
        delay(10);
    }
    _gzBias = sum / 300.0f;
    _heading = 0;
    _lastMicros = micros();
    Serial.print(F(" done. bias=")); Serial.print(_gzBias, 2);
    Serial.print(F("  spread=")); Serial.println(mx - mn);
    // health check: a live gyro has small nonzero noise; dead/loose = 0 spread
    if (mx == mn) {
        Serial.println(F("  !!! GYRO NOT RESPONDING — check I2C: SDA=D20, SCL=D21, 3.3V, GND !!!"));
    } else if (fabs(_gzBias) > 400) {
        Serial.println(F("  !! big bias — was the robot moving during calibration? re-do still."));
    }
}

// ─────────────────────────────────────────────────────
//  SPEED CALIBRATION — يقيس RT_PWM_STATIC و RT_KV
//  يمشي دفعتين بسرعتين ويقيس cm/s، بعدين يطبع القيم المقترحة.
//  ⚠️ لازم ~1 متر مساحة فاضية قدام الروبوت.
// ─────────────────────────────────────────────────────
void Robotrick::calibrateSpeed() {
    float cmPerCount = (3.14159f * RT_WHEEL_DIAMETER_MM / RT_COUNTS_PER_REV) / 10.0f;
    Serial.println(F("[Robotrick] SPEED CAL — needs ~1m clear space! starting in 2s"));
    delay(2000);

    // Run 1: PWM 120 for 1.5s
    int p1 = 120; float dur1 = 1.5f;
    resetEncoders();
    _spinL(p1); _spinR(p1);
    uint32_t end = millis() + (uint32_t)(dur1 * 1000);
    while (millis() < end) { /* drive */ }
    long c1 = abs(_encL.read());   // اليسار فقط (الإنكودرات مختلفة 12 vs 48 CPR)
    stop();
    float v1 = c1 * cmPerCount / dur1;
    delay(1000);

    // Run 2: PWM 220 for 1.0s
    int p2 = 220; float dur2 = 1.0f;
    resetEncoders();
    _spinL(p2); _spinR(p2);
    end = millis() + (uint32_t)(dur2 * 1000);
    while (millis() < end) { /* drive */ }
    long c2 = abs(_encL.read());   // اليسار فقط (الإنكودرات مختلفة 12 vs 48 CPR)
    stop();
    float v2 = c2 * cmPerCount / dur2;

    Serial.println(F("=== SPEED CALIBRATION ==="));
    Serial.print(F("PWM 120 -> ")); Serial.print(v1, 1); Serial.println(F(" cm/s"));
    Serial.print(F("PWM 220 -> ")); Serial.print(v2, 1); Serial.println(F(" cm/s"));

    if (v2 <= v1 + 0.5f) {
        Serial.println(F("ERR: speeds too close — check motors/encoders, retry."));
        return;
    }
    float kv      = (p2 - p1) / (v2 - v1);
    float pstatic = p1 - kv * v1;
    float vmax    = (255 - pstatic) / kv;

    Serial.print(F(">> RT_PWM_STATIC = ")); Serial.println(pstatic, 1);
    Serial.print(F(">> RT_KV         = ")); Serial.println(kv, 2);
    Serial.print(F("   max speed ~"));      Serial.print(vmax, 0);
    Serial.println(F(" cm/s  → set RT_STRAIGHT_SPEED to ~70% of it"));
    Serial.print(F(">> RT_STRAIGHT_SPEED ~ ")); Serial.println(vmax * 0.7f, 0);
}

// ─────────────────────────────────────────────────────
//  PUBLIC MOVEMENT
// ─────────────────────────────────────────────────────
void Robotrick::forward(float cm)  { driveStraight(cm, +1); }
void Robotrick::backward(float cm) { driveStraight(cm, -1); }
void Robotrick::turnLeft(float deg)  { turn(+fabs(deg)); }
void Robotrick::turnRight(float deg) { turn(-fabs(deg)); }

// ── Drive tuning (لايف — بتأثّر على forward/backward مباشرة) ──────
void Robotrick::setDriveSpeed(float cmPerSec) {
    _driveSpeed = constrain(cmPerSec, 1.0f, 100.0f);   // 0 يوقف الحركة → أدنى 1
    Serial.print(F("[Robotrick] driveSpeed = ")); Serial.print(_driveSpeed, 1);
    Serial.println(F(" cm/s"));
}

void Robotrick::setDriveAccel(float cmPerSec2) {
    _driveAccel = constrain(cmPerSec2, 1.0f, 400.0f);
    Serial.print(F("[Robotrick] driveAccel = ")); Serial.print(_driveAccel, 1);
    Serial.println(F(" cm/s^2"));
}

void Robotrick::setDrivePID(float kp, float kd, float ki) {
    _distKp = (kp >= 0) ? kp : _distKp;
    _distKd = (kd >= 0) ? kd : _distKd;
    _distKi = (ki >= 0) ? ki : _distKi;
    Serial.print(F("[Robotrick] drivePID  Kp=")); Serial.print(_distKp, 3);
    Serial.print(F("  Kd=")); Serial.print(_distKd, 3);
    Serial.print(F("  Ki=")); Serial.println(_distKi, 3);
}

// رجّع كل شي لقيم الـ CONFIG (زر "default" للطلاب لو عبثوا بالقيم)
void Robotrick::resetDriveTuning() {
    _driveSpeed = RT_STRAIGHT_SPEED;
    _driveAccel = RT_STRAIGHT_ACCEL;
    _distKp     = RT_DIST_KP;
    _distKd     = RT_DIST_KD;
    _distKi     = RT_DIST_KI;
    _hdgKp       = RT_STRAIGHT_KP;
    _hdgKd       = RT_STRAIGHT_KD;
    _hdgDeadband = RT_STRAIGHT_DEADBAND;
    Serial.println(F("[Robotrick] drive tuning RESET to defaults"));
    printDriveTuning();
}

void Robotrick::setHeadingPD(float kp, float kd, float deadband) {
    _hdgKp       = (kp >= 0) ? kp : _hdgKp;
    _hdgKd       = (kd >= 0) ? kd : _hdgKd;
    _hdgDeadband = (deadband >= 0) ? deadband : _hdgDeadband;
    Serial.print(F("[Robotrick] headingPD  Kp=")); Serial.print(_hdgKp, 3);
    Serial.print(F("  Kd=")); Serial.print(_hdgKd, 3);
    Serial.print(F("  dead=")); Serial.print(_hdgDeadband, 2); Serial.println(F("deg"));
}

// ── Turn tuning (لايف — بتأثّر على turnLeft/turnRight مباشرة) ──────
void Robotrick::setTurnSpeed(int fast, int slowMax, int slowMin) {
    if (fast    >= 0) _turnFast    = constrain(fast,    0, 255);
    if (slowMax >= 0) _turnSlowMax = constrain(slowMax, 0, 255);
    if (slowMin >= 0) _turnSlowMin = constrain(slowMin, 0, 255);
    Serial.print(F("[Robotrick] turnSpeed fast=")); Serial.print(_turnFast);
    Serial.print(F("  slowMax=")); Serial.print(_turnSlowMax);
    Serial.print(F("  slowMin=")); Serial.println(_turnSlowMin);
}

void Robotrick::setTurnTune(float kp, float fastDeg) {
    if (kp      >= 0) _turnKp      = kp;
    if (fastDeg >= 0) _turnFastDeg = fastDeg;
    Serial.print(F("[Robotrick] turnTune Kp=")); Serial.print(_turnKp, 2);
    Serial.print(F("  fastDeg=")); Serial.println(_turnFastDeg, 1);
}

void Robotrick::resetTurnTuning() {
    _turnFast = RT_TURN_FAST; _turnSlowMax = RT_TURN_SLOW_MAX; _turnSlowMin = RT_TURN_SLOW_MIN;
    _turnKp = RT_TURN_KP; _turnFastDeg = RT_TURN_FAST_DEG;
    Serial.println(F("[Robotrick] turn tuning RESET to defaults"));
    printTurnTuning();
}

void Robotrick::printTurnTuning() {
    Serial.println(F("[Robotrick] --- turn tuning ---"));
    Serial.print(F("  fast=")); Serial.print(_turnFast);
    Serial.print(F("  slowMax=")); Serial.print(_turnSlowMax);
    Serial.print(F("  slowMin=")); Serial.println(_turnSlowMin);
    Serial.print(F("  Kp=")); Serial.print(_turnKp, 2);
    Serial.print(F("  fastDeg=")); Serial.println(_turnFastDeg, 1);
}

// ── Pivot speed tuning (لايف — بتأثّر على pivot مباشرة) ──────
void Robotrick::setPivotSpeed(int fast, int slow, int min) {
    if (fast >= 0) _pivotFast = constrain(fast, 0, 255);
    if (slow >= 0) _pivotSlow = constrain(slow, 0, 255);
    if (min  >= 0) _pivotMin  = constrain(min,  0, 255);
    Serial.print(F("[Robotrick] pivotSpeed fast=")); Serial.print(_pivotFast);
    Serial.print(F("  slow=")); Serial.print(_pivotSlow);
    Serial.print(F("  min=")); Serial.println(_pivotMin);
}

void Robotrick::resetPivotTuning() {
    _pivotFast = RT_PIVOT_FAST; _pivotSlow = RT_PIVOT_SLOW; _pivotMin = RT_PIVOT_MIN;
    Serial.println(F("[Robotrick] pivot tuning RESET to defaults"));
    printPivotTuning();
}

void Robotrick::printPivotTuning() {
    Serial.print(F("[Robotrick] --- pivot --- fast=")); Serial.print(_pivotFast);
    Serial.print(F("  slow=")); Serial.print(_pivotSlow);
    Serial.print(F("  min=")); Serial.println(_pivotMin);
}

float Robotrick::getDriveSpeed() { return _driveSpeed; }

void Robotrick::printDriveTuning() {
    Serial.println(F("[Robotrick] --- drive tuning ---"));
    Serial.print(F("  speed = ")); Serial.print(_driveSpeed, 1); Serial.println(F(" cm/s"));
    Serial.print(F("  accel = ")); Serial.print(_driveAccel, 1); Serial.println(F(" cm/s^2"));
    Serial.print(F("  dist Kp=")); Serial.print(_distKp, 3);
    Serial.print(F("  Kd=")); Serial.print(_distKd, 3);
    Serial.print(F("  Ki=")); Serial.println(_distKi, 3);
    Serial.print(F("  hdg  Kp=")); Serial.print(_hdgKp, 3);
    Serial.print(F("  Kd=")); Serial.print(_hdgKd, 3);
    Serial.print(F("  dead=")); Serial.print(_hdgDeadband, 2); Serial.println(F("deg"));
}

// ── Line-follower tuning (لايف — بتأثّر على followLine مباشرة) ────
void Robotrick::setLineSpeed(int base, int maxSpd, int minSpd) {
    if (base   >= 0) _lineBase = constrain(base,   0, 255);
    if (maxSpd >= 0) _lineMax  = constrain(maxSpd, 0, 255);
    if (minSpd >= 0) _lineMin  = constrain(minSpd, 0, 255);
    Serial.print(F("[Robotrick] lineSpeed base=")); Serial.print(_lineBase);
    Serial.print(F("  max=")); Serial.print(_lineMax);
    Serial.print(F("  min=")); Serial.println(_lineMin);
}

void Robotrick::setLinePD(float kp, float kd) {
    if (kp >= 0) _lineKp = kp;
    if (kd >= 0) _lineKd = kd;
    Serial.print(F("[Robotrick] linePD  Kp=")); Serial.print(_lineKp, 4);
    Serial.print(F("  Kd=")); Serial.println(_lineKd, 4);
}

void Robotrick::setLineTune(int deadband, float kpBoost, int slow) {
    if (deadband >= 0) _lineDeadband = deadband;
    if (kpBoost  >= 0) _lineKpBoost  = kpBoost;
    if (slow     >= 0) _lineSlow     = constrain(slow, 0, 255);
    Serial.print(F("[Robotrick] lineTune dead=")); Serial.print(_lineDeadband);
    Serial.print(F("  boost=")); Serial.print(_lineKpBoost, 2);
    Serial.print(F("  slow=")); Serial.println(_lineSlow);
}

void Robotrick::resetLineTuning() {
    _lineBase = RT_LINE_BASE; _lineMax = RT_LINE_MAX; _lineMin = RT_LINE_MIN;
    _lineSlow = RT_LINE_SLOW; _lineDeadband = RT_LINE_DEADBAND;
    _lineKp = RT_LINE_KP; _lineKd = RT_LINE_KD; _lineKpBoost = RT_LINE_KP_BOOST;
    Serial.println(F("[Robotrick] line tuning RESET to defaults"));
    printLineTuning();
}

void Robotrick::printLineTuning() {
    Serial.println(F("[Robotrick] --- line tuning ---"));
    Serial.print(F("  speed base=")); Serial.print(_lineBase);
    Serial.print(F("  max=")); Serial.print(_lineMax);
    Serial.print(F("  min=")); Serial.print(_lineMin);
    Serial.print(F("  slow=")); Serial.println(_lineSlow);
    Serial.print(F("  Kp=")); Serial.print(_lineKp, 4);
    Serial.print(F("  Kd=")); Serial.print(_lineKd, 4);
    Serial.print(F("  boost=")); Serial.print(_lineKpBoost, 2);
    Serial.print(F("  deadband=")); Serial.println(_lineDeadband);
}

void Robotrick::stop() {
    analogWrite(RT_L_PWM, 0);
    analogWrite(RT_R_PWM, 0);
}

float Robotrick::heading()      { return _heading; }
void  Robotrick::resetHeading() { _heading = 0; _lastMicros = micros(); }
void  Robotrick::update()       { _updateHeading(); }

// ─────────────────────────────────────────────────────
//  STRAIGHT DRIVE — v2 PHASE A
//  خطة (trapezoid) + feedforward + tracking PID + وقوف دقيق + أمان
//  المسافة على محور الإنكودر، الاستقامة على الجايرو (heading hold)
// ─────────────────────────────────────────────────────
void Robotrick::driveStraight(float cm, int dir) {
    Serial.print(F("[Robotrick] ")); Serial.print(dir > 0 ? F("FWD ") : F("BACK "));
    Serial.print(cm); Serial.println(F("cm"));

    float cmPerCount = (3.14159f * RT_WHEEL_DIAMETER_MM / RT_COUNTS_PER_REV) / 10.0f;
    float S = fabs(cm);
    if (S < 0.05f) return;

    // ── plan trapezoid (magnitude) ────────────────────
    float vmax = _driveSpeed, a = _driveAccel, d = _driveAccel;
    float accelD = vmax * vmax / (2 * a);
    float decelD = vmax * vmax / (2 * d);
    float vpeak, t1, t2, t3, sT1, sT2;
    if (accelD + decelD <= S) {                 // full trapezoid
        vpeak = vmax;
        t1 = vpeak / a;
        float cruise = S - accelD - decelD;
        t2 = t1 + cruise / vpeak;
        t3 = t2 + vpeak / d;
        sT1 = accelD; sT2 = accelD + cruise;
    } else {                                    // triangle (never reach vmax)
        vpeak = sqrt(a * S);                    // a == d
        t1 = vpeak / a; t2 = t1;
        t3 = t2 + vpeak / d;
        sT1 = vpeak * vpeak / (2 * a); sT2 = sT1;
    }

    resetEncoders();
    resetHeading();

    float integ = 0, velFilt = 0;
    long  lastCounts = 0;
    uint32_t tStart = micros(), tPrev = tStart;
    uint32_t settleStart = 0, stallStart = 0, stuckStart = 0;
    uint32_t hardTimeout = millis() + RT_MOVE_TIMEOUT;

    while (millis() < hardTimeout) {
        _updateHeading();
        uint32_t now = micros();
        float t  = (now - tStart) * 1e-6f;
        float dt = (now - tPrev)  * 1e-6f;
        tPrev = now;
        if (dt <= 0) continue;

        // ── state estimate ────────────────────────────
        long counts = abs(_encL.read());   // اليسار فقط — الإنكودرات مختلفة (12 vs 48 CPR)
        float pos     = counts * cmPerCount;                     // cm (magnitude)
        float velRaw  = (counts - lastCounts) * cmPerCount / dt;  // cm/s
        lastCounts = counts;
        velFilt += 0.3f * (velRaw - velFilt);                    // low-pass

        // ── trajectory reference at t ─────────────────
        float sRef, vRef;
        if      (t < t1) { vRef = a * t;               sRef = 0.5f * a * t * t; }
        else if (t < t2) { vRef = vpeak;               sRef = sT1 + vpeak * (t - t1); }
        else if (t < t3) { float td = t - t2; vRef = vpeak - d * td;
                           sRef = sT2 + vpeak * td - 0.5f * d * td * td; }
        else             { vRef = 0;                   sRef = S; }

        // ── feedforward + tracking PID (distance axis) ─
        float posErr = sRef - pos;
        float velErr = vRef - velFilt;
        float ff = (vRef > 0.01f) ? (RT_PWM_STATIC + RT_KV * vRef) : 0;
        float p  = _distKp * posErr;
        float dd = _distKd * velErr;
        if (_distKi > 0) {                                // anti-windup integrator
            float u_pd = ff + p + dd;
            bool sat = (u_pd >= 255 || u_pd <= -255);
            if (!(sat && (posErr > 0)))
                integ = constrain(integ + posErr * dt, -255.0f / _distKi, 255.0f / _distKi);
        }
        float baseMag = ff + p + _distKi * integ + dd;
        // أرضية: إذا لسا ما وصل، لازم PWM كافٍ يتحرّك (يمنع التوقف القصير)
        if (posErr > RT_POS_TOL && baseMag < RT_PWM_STATIC) baseMag = RT_PWM_STATIC;
        baseMag = constrain(baseMag, 0.0f, 255.0f);

        // ── heading hold (استقامة) — حسب RT_STRAIGHT_MODE ─
        float corr = 0;
        // MODE 1 أو 3 → جايرو PD
        if (RT_STRAIGHT_MODE == 1 || RT_STRAIGHT_MODE == 3) {
            float herr = _heading;
            if (herr > -_hdgDeadband && herr < _hdgDeadband) herr = 0;
            corr += RT_STEER_SIGN * (_hdgKp * herr + _hdgKd * _gyroRate);
        }
        // MODE 2 أو 3 → موازنة الإنكودر (نطبّع اليمين 4× لمقياس اليسار)
        // × dir: تصحيح الإنكودر ينعكس مع اتجاه الحركة (عكس الجايرو المطلق)
        if (RT_STRAIGHT_MODE == 2 || RT_STRAIGHT_MODE == 3) {
            const float encRtoL = 0.25f;         // right counts 4× (2256 vs 564) → normalize to left
            long encRnorm = (long)(abs(_encR.read()) * encRtoL);
            long encErr   = encRnorm - counts;   // counts = abs(encL)
            corr += dir * RT_ENC_KP * encErr;
        }

        int base = (int)(dir * baseMag);
        setMotors(base - (int)corr, base + (int)corr);

        // ── completion: خلصت الخطة + قريب + بطيء + ثابت ─
        if (t >= t3 && fabs(posErr) < RT_POS_TOL && fabs(velFilt) < RT_SPEED_TOL) {
            if (!settleStart) settleStart = millis();
            if (millis() - settleStart >= RT_SETTLE_MS) break;
        } else settleStart = 0;

        // ── stuck-short: خلصت الخطة بس واقف مكانه (طنين بدون حركة) → اعتبره وصل ─
        if (t >= t3 && fabs(velFilt) < RT_STALL_SPEED) {
            if (!stuckStart) stuckStart = millis();
            if (millis() - stuckStart >= RT_STUCK_MS) {
                Serial.println(F("[Robotrick] arrived (stuck-short, no buzz)")); break;
            }
        } else stuckStart = 0;

        // ── stall: يدفع بقوة بس مو ماشي ────────────────
        if (baseMag > 0.8f * 255 && fabs(velFilt) < RT_STALL_SPEED) {
            if (!stallStart) stallStart = millis();
            if (millis() - stallStart >= RT_STALL_MS) {
                Serial.println(F("[Robotrick] STALL — abort")); break;
            }
        } else stallStart = 0;
    }

    stop();
    delay(120);
    Serial.print(F("  done. pos=")); Serial.print(lastCounts * cmPerCount, 1);
    Serial.print(F("cm heading=")); Serial.println(_heading, 1);
}

// ─────────────────────────────────────────────────────
//  TURN — gyro two-phase  (+deg = left, -deg = right)
// ─────────────────────────────────────────────────────
void Robotrick::turn(float deg) {
    if (fabs(deg) < 0.5f) return;
    Serial.print(F("[Robotrick] TURN ")); Serial.print(deg); Serial.println(F("°"));

    resetHeading();
    // نقيس مقدار الدوران (magnitude) — مستقل عن إشارة الجايرو = ما في لف لانهائي
    float targetMag = fabs(deg);
    int   dir       = ((deg >= 0) ? 1 : -1) * RT_TURN_DIR;  // +1 يسار، -1 يمين (فيزيائياً)
    float slowStart = targetMag - _turnFastDeg;
    if (slowStart < 0) slowStart = 0;

    // Phase 1: سرعة كاملة لحد ما نقرب من الهدف
    _spinL(-dir * _turnFast);
    _spinR( dir * _turnFast);

    uint32_t timeout = millis() + RT_TURN_TIMEOUT;
    while (millis() < timeout) {
        _updateHeading();
        if (fabs(_heading) >= slowStart) break;
    }

    // Phase 2: P-control بطيء لحد ما يكمّل الزاوية
    while (millis() < timeout) {
        _updateHeading();
        float err = targetMag - fabs(_heading);   // كم باقي
        if (err <= RT_TURN_TOLERANCE) break;

        int spd = (int)(_turnKp * err);
        spd = constrain(spd, _turnSlowMin, _turnSlowMax);
        _spinL(-dir * spd);
        _spinR( dir * spd);
    }

    // فرملة عكسية قصيرة لقتل القصور الذاتي
    _spinL( dir * 150);
    _spinR(-dir * 150);
    delay(RT_TURN_BRAKE_MS);

    stop();
    delay(150);
    Serial.print(F("  done. final=")); Serial.print(_heading, 1); Serial.println(F("°"));
}

// ─────────────────────────────────────────────────────
//  PIVOT — عجلة وحدة تتحرك، التانية واقفة (بالجايرو)
//  wheel: 'L' أو 'R' = العجلة اللي تتحرك
//  deg  : موجب = تلك العجلة لقدام، سالب = لخلف؛ |deg| = زاوية الدوران
// ─────────────────────────────────────────────────────
void Robotrick::pivot(char wheel, float deg) {
    if (fabs(deg) < 0.5f) return;
    bool useLeft = (wheel == 'L' || wheel == 'l');
    int  sgn = (deg >= 0) ? 1 : -1;          // + = العجلة المتحركة لقدام
    float targetMag = fabs(deg);

    Serial.print(F("[Robotrick] PIVOT ")); Serial.print(useLeft ? F("L") : F("R"));
    Serial.print(sgn > 0 ? F(" FWD ") : F(" BACK ")); Serial.print(targetMag); Serial.println(F("°"));

    resetHeading();
    float slowStart = targetMag - _turnFastDeg;
    if (slowStart < 0) slowStart = 0;

    // Phase 1: سرعة كاملة — عجلة وحدة، التانية = 0
    if (useLeft) { _spinL(sgn * _pivotFast); _spinR(0); }
    else         { _spinR(sgn * _pivotFast); _spinL(0); }

    uint32_t timeout = millis() + RT_TURN_TIMEOUT;
    while (millis() < timeout) {
        _updateHeading();
        if (fabs(_heading) >= slowStart) break;
    }

    // Phase 2: P-control بطيء — يوقف الدفع قبل الهدف بـ RT_PIVOT_STOP_DEG (تعويض الاندفاع)
    while (millis() < timeout) {
        _updateHeading();
        float err = targetMag - fabs(_heading);
        if (err <= RT_PIVOT_STOP_DEG) break;
        int spd = constrain((int)(_turnKp * err), _pivotMin, _pivotSlow);
        if (useLeft) { _spinL(sgn * spd); _spinR(0); }
        else         { _spinR(sgn * spd); _spinL(0); }
    }

    // فرملة قوية: counter-spin بالعجلتين (متل turn) — يقتل اندفاع الدوران
    // العجلة المتحركة تنعكس، والواقفة تدفع بالعكس → عزم مضاد
    if (useLeft) { _spinL(-sgn * RT_PIVOT_BRAKE); _spinR( sgn * RT_PIVOT_BRAKE); }
    else         { _spinR(-sgn * RT_PIVOT_BRAKE); _spinL( sgn * RT_PIVOT_BRAKE); }
    delay(RT_TURN_BRAKE_MS);

    stop();
    delay(150);
    _updateHeading();   // اقرأ الزاوية النهائية بعد ما استقر
    Serial.print(F("  done. heading=")); Serial.print(_heading, 1); Serial.println(F("°"));
}

// ─────────────────────────────────────────────────────
//  LOW-LEVEL MOTORS
// ─────────────────────────────────────────────────────
void Robotrick::setMotors(int left, int right) {
    _spinL(left);
    _spinR(right);
}

// ── Motor 4 (رافعة/آلية) — تحكّم بالسرعة/الوقت ──────
void Robotrick::motor4(int speed) {
    if (RT_M4_REVERSE) speed = -speed;
    speed = constrain(speed, -255, 255);
    digitalWrite(RT_M4_DIR, speed >= 0 ? HIGH : LOW);
    analogWrite(RT_M4_PWM, abs(speed));
}

void Robotrick::motor4Stop() {
    analogWrite(RT_M4_PWM, 0);
    _m4StopAt = 0; _m4Mode = 0;   // ألغِ أي async (وقت أو عدّات)
}

// يوقف الرافعة تلقائياً: بالوقت (mode 1) أو بالعدّات (mode 2). خلفية.
void Robotrick::_serviceMotor4() {
    if (_m4Mode == 1) {                              // بالوقت
        if (millis() >= _m4StopAt) { analogWrite(RT_M4_PWM, 0); _m4Mode = 0; }
    }
#if RT_LIFT_USE_ENCODER
    else if (_m4Mode == 2) {                         // بالانكودر
        if (labs(_liftEnc.read()) >= _m4TargetCount) { analogWrite(RT_M4_PWM, 0); _m4Mode = 0; }
    }
#endif
}

void Robotrick::setLiftUseEncoder(bool on) {
#if RT_LIFT_USE_ENCODER
    _liftUseEnc = on;
    Serial.print(F("[Robotrick] lift encoder mode = ")); Serial.println(on ? F("ON") : F("OFF"));
#else
    (void)on;
    Serial.println(F("[Robotrick] انكودر الرافعة مو مفعّل — خلّي RT_LIFT_USE_ENCODER=1 وأعد الكومبايل"));
#endif
}

long Robotrick::liftEncoder() {
#if RT_LIFT_USE_ENCODER
    return _liftEnc.read();
#else
    return 0;
#endif
}

void Robotrick::resetLiftEncoder() {
#if RT_LIFT_USE_ENCODER
    _liftEnc.write(0);
#endif
}

void Robotrick::motor4For(int speed, uint32_t ms) {
    motor4(speed);
    uint32_t endAt = millis() + ms;
    while (millis() < endAt) {}
    motor4Stop();
}

// رفع/تنزيل الرافعة — الاتجاه من RT_LIFT_UP_SIGN
void Robotrick::liftUp(uint32_t ms) {
    Serial.print(F("[Robotrick] LIFT UP ")); Serial.print(ms); Serial.println(F("ms"));
    motor4For(RT_LIFT_UP_SIGN * RT_LIFT_SPEED, ms);
}
void Robotrick::liftDown(uint32_t ms) {
    Serial.print(F("[Robotrick] LIFT DOWN ")); Serial.print(ms); Serial.println(F("ms"));
    motor4For(-RT_LIFT_UP_SIGN * RT_LIFT_SPEED, ms);
}

// ── نسخ غير حابسة: تشغّل الرافعة وترجع فوراً ──────────────
//  القيمة (value): عادةً ms. لكن لو خيار الانكودر ON → value = عدّات.
//  value=0 → تفضل شغّالة لحد liftStop.
void Robotrick::_liftStartAsync(int speed, uint32_t value, const __FlashStringHelper* tag) {
    motor4(speed);
    if (value == 0) { _m4StopAt = 0; _m4Mode = 0;      // دايم
        Serial.print(F("[Robotrick] ")); Serial.print(tag); Serial.println(F(" (دايم)")); return; }
#if RT_LIFT_USE_ENCODER
    if (_liftUseEnc) {                                   // بالعدّات
        _liftEnc.write(0); _m4TargetCount = (long)value; _m4Mode = 2;
        Serial.print(F("[Robotrick] ")); Serial.print(tag);
        Serial.print(value); Serial.println(F(" counts")); return;
    }
#endif
    _m4StopAt = millis() + value; _m4Mode = 1;          // بالوقت
    Serial.print(F("[Robotrick] ")); Serial.print(tag); Serial.print(value); Serial.println(F("ms"));
}
void Robotrick::liftUpAsync(uint32_t value) {
    _liftStartAsync(RT_LIFT_UP_SIGN * RT_LIFT_SPEED, value, F("LIFT UP async "));
}
void Robotrick::liftDownAsync(uint32_t value) {
    _liftStartAsync(-RT_LIFT_UP_SIGN * RT_LIFT_SPEED, value, F("LIFT DOWN async "));
}
void Robotrick::liftStop() { motor4Stop(); }
bool Robotrick::liftBusy() { return _m4Mode != 0; }

// ─────────────────────────────────────────────────────
//  SERVOS — 3 سيرفو على A2/A3/A4، رقمها 1..3
//  attach كسول: السيرفو ما بينشغل إلا أول ما تكتب له زاوية،
//  عشان ما ياخد timer/عزم قبل ما نحتاجه.
// ─────────────────────────────────────────────────────
void Robotrick::_servoBegin() {
    _servoStopUs = RT_SERVO_STOP_US;
    _servoPin[0] = RT_SERVO1_PIN;
    _servoPin[1] = RT_SERVO2_PIN;
    _servoPin[2] = RT_SERVO3_PIN;
    for (uint8_t i = 0; i < RT_SERVO_N; i++) {
        _servoPos[i]      = -1;      // لسا ما تحرّك
        _servoAttached[i] = false;
    }
}

// فعّل السيرفو i (index داخلي 0..N-1) لو مش مفعّل. true = جاهز
bool Robotrick::_servoEnsure(uint8_t i) {
    if (i >= RT_SERVO_N) return false;
    if (!_servoAttached[i]) {
        _servo[i].attach(_servoPin[i], RT_SERVO_MIN_US, RT_SERVO_MAX_US);
        _servoAttached[i] = true;
    }
    return true;
}

void Robotrick::servoWrite(uint8_t idx, int angle) {
    if (idx < 1 || idx > RT_SERVO_N) {
        Serial.println(F("[Robotrick] servo idx لازم 1..3")); return;
    }
    uint8_t i = idx - 1;
    angle = constrain(angle, RT_SERVO_MIN_DEG, RT_SERVO_MAX_DEG);
    if (!_servoEnsure(i)) return;
    _servo[i].write(angle);
    _servoPos[i] = angle;
    Serial.print(F("[Robotrick] servo ")); Serial.print(idx);
    Serial.print(F(" -> ")); Serial.print(angle); Serial.println(F("°"));
}

void Robotrick::servoMove(uint8_t idx, int angle, uint16_t degPerSec) {
    if (idx < 1 || idx > RT_SERVO_N) {
        Serial.println(F("[Robotrick] servo idx لازم 1..3")); return;
    }
    uint8_t i = idx - 1;
    angle = constrain(angle, RT_SERVO_MIN_DEG, RT_SERVO_MAX_DEG);
    if (!_servoEnsure(i)) return;

    int from = _servoPos[i];
    // أول حركة أو سرعة 0 → روح فوراً
    if (from < 0 || degPerSec == 0) {
        _servo[i].write(angle);
        _servoPos[i] = angle;
        Serial.print(F("[Robotrick] servo ")); Serial.print(idx);
        Serial.print(F(" -> ")); Serial.print(angle); Serial.println(F("° (فوري)"));
        return;
    }

    uint16_t msPerDeg = 1000UL / degPerSec;
    if (msPerDeg < 1) msPerDeg = 1;      // سقف سرعة عملي
    int step = (angle >= from) ? 1 : -1;
    for (int a = from; a != angle; a += step) {
        _servo[i].write(a);
        delay(msPerDeg);
    }
    _servo[i].write(angle);
    _servoPos[i] = angle;
    Serial.print(F("[Robotrick] servo ")); Serial.print(idx);
    Serial.print(F(" ~> ")); Serial.print(angle);
    Serial.print(F("° @ ")); Serial.print(degPerSec); Serial.println(F(" deg/s"));
}

void Robotrick::servoDetach(uint8_t idx) {
    if (idx < 1 || idx > RT_SERVO_N) return;
    uint8_t i = idx - 1;
    if (_servoAttached[i]) {
        _servo[i].detach();
        _servoAttached[i] = false;
        Serial.print(F("[Robotrick] servo ")); Serial.print(idx); Serial.println(F(" detached"));
    }
}

void Robotrick::servoAttachAll() {
    for (uint8_t i = 0; i < RT_SERVO_N; i++) _servoEnsure(i);
}

// سيرفو 360° (دوران مستمر): speed -100..100، 0 = وقوف
// بيكتب نبضة µs: STOP عند 0، وينزاح باتجاه MIN/MAX حسب الإشارة والمقدار.
void Robotrick::servoSpin(uint8_t idx, int speed) {
    if (idx < 1 || idx > RT_SERVO_N) {
        Serial.println(F("[Robotrick] servo idx لازم 1..3")); return;
    }
    uint8_t i = idx - 1;
    speed = constrain(speed, -100, 100);
    if (!_servoEnsure(i)) return;
    long us;
    if (speed >= 0) us = _servoStopUs + (long)speed * (RT_SERVO_MAX_US - _servoStopUs) / 100;
    else            us = _servoStopUs + (long)speed * (_servoStopUs - RT_SERVO_MIN_US) / 100;
    _servo[i].writeMicroseconds((int)us);
    _servoPos[i] = -1;   // مش زاوية — دوران
    Serial.print(F("[Robotrick] servo ")); Serial.print(idx);
    Serial.print(F(" spin ")); Serial.print(speed);
    Serial.print(F("%  (")); Serial.print(us); Serial.println(F("us)"));
}

void Robotrick::servoStop(uint8_t idx) {
    servoSpin(idx, 0);
}

// عاير نبضة الوقوف لسيرفو 360°. بيطبّقها فوراً على أي سيرفو مفعّل
// حتى تشوف مباشرة لمّا يوقف تماماً (زحف للأمام؟ قلّلها. للخلف؟ زوّدها).
void Robotrick::setServoStop(int us) {
    _servoStopUs = constrain(us, 1000, 2000);
    for (uint8_t i = 0; i < RT_SERVO_N; i++)
        if (_servoAttached[i]) _servo[i].writeMicroseconds(_servoStopUs);
    Serial.print(F("[Robotrick] servo stop = ")); Serial.print(_servoStopUs);
    Serial.println(F("us (طبّقها على المفعّلين)"));
}

// افصل كل سيرفو مفعّل قبل تتبع الخط. سبب: قراءة QTR-RC (RC timing) كل لوب
// بتشوّش نبضة السيرفو (Timer5) فيرجف/يتحرّك. بعد الفصل ما في نبضات = ما في تشويش.
// بيرجعوا للعمل أوتوماتيك أول ما تنادي servoWrite/servoSpin بعدها (attach كسول).
void Robotrick::_lineParkServos() {
    bool any = false;
    for (uint8_t i = 0; i < RT_SERVO_N; i++) {
        if (_servoAttached[i]) { _servo[i].detach(); _servoAttached[i] = false; any = true; }
    }
    if (any) Serial.println(F("[Robotrick] servos parked (فُصلوا) أثناء تتبع الخط"));
}

// ═════════════════════════════════════════════════════
//  COLOR SENSORS — 4× TCS34725 على DIY bus-gate mux
//  كلهم 0x29؛ نشغّل بوابة سلوت واحد بلحظة. التصنيف بالـ
//  chromaticity (نِسب مستقلة عن السطوع) + أقرب لون متعلّم.
// ═════════════════════════════════════════════════════
#define TCS_CMD_    0x80
#define TCS_AUTO_   0x20
#define TCS_ENABLE_ 0x00
#define TCS_ATIME_  0x01
#define TCS_CTRL_   0x0F
#define TCS_ID_     0x12
#define TCS_STAT_   0x13
#define TCS_CDATAL_ 0x14
#define TCS_PON_    0x01
#define TCS_AEN_    0x02

// مراجع الألوان الافتراضية (مقاسة على الروبوت) — حسب ترتيب enum RTColor
static const RTColorRef COLOR_DEFAULT[7] = {
    { 0.54f, 0.22f, 0.24f, true  },   // RT_RED
    { 0.23f, 0.43f, 0.34f, true  },   // RT_GREEN
    { 0.18f, 0.36f, 0.47f, true  },   // RT_BLUE
    { 0.47f, 0.34f, 0.20f, true  },   // RT_YELLOW
    { 0.00f, 0.00f, 0.00f, false },   // RT_WHITE (غير مقاس بعد)
    { 0.25f, 0.36f, 0.39f, true  },   // RT_BLACK
    { 0.00f, 0.00f, 0.00f, false },   // RT_UNKNOWN
};

void Robotrick::_colorSelect(int slot) {
    for (uint8_t i = 0; i < RT_COLOR_N; i++)
        digitalWrite(_colorPin[i], (i == slot) ? HIGH : LOW);
    delayMicroseconds(600);          // settle بعد تبديل الـ FET
}
void Robotrick::_colorDeselectAll() {
    for (uint8_t i = 0; i < RT_COLOR_N; i++) digitalWrite(_colorPin[i], LOW);
}
bool Robotrick::_colorAck() {
    Wire.beginTransmission(RT_COLOR_ADDR);
    return Wire.endTransmission() == 0;
}
bool Robotrick::_tcsW(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(RT_COLOR_ADDR);
    Wire.write(TCS_CMD_ | reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}
uint8_t Robotrick::_tcsR(uint8_t reg) {
    Wire.beginTransmission(RT_COLOR_ADDR);
    Wire.write(TCS_CMD_ | reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    Wire.requestFrom((uint8_t)RT_COLOR_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}
void Robotrick::_colorConfig() {
    _tcsW(TCS_ENABLE_, TCS_PON_); delay(3);
    _tcsW(TCS_ATIME_,  RT_COLOR_ATIME);
    _tcsW(TCS_CTRL_,   RT_COLOR_GAIN);
    _tcsW(TCS_ENABLE_, TCS_PON_ | TCS_AEN_);
}

void Robotrick::_colorBegin() {
    _colorPin[0] = RT_COLOR1_PIN; _colorPin[1] = RT_COLOR2_PIN;
    _colorPin[2] = RT_COLOR3_PIN; _colorPin[3] = RT_COLOR4_PIN;
    for (uint8_t i = 0; i < RT_COLOR_N; i++) {
        pinMode(_colorPin[i], OUTPUT); digitalWrite(_colorPin[i], LOW);
        _colorOk[i] = false;
    }
    pinMode(RT_COLOR_LED_PIN, OUTPUT); digitalWrite(RT_COLOR_LED_PIN, HIGH); // إضاءة ON
    for (uint8_t i = 0; i < 7; i++) _colorRef[i] = COLOR_DEFAULT[i];

    // فحص + تهيئة كل سلوت
    uint8_t found = 0;
    for (uint8_t s = 0; s < RT_COLOR_N; s++) {
        _colorSelect(s);
        if (_colorAck()) {
            uint8_t id = _tcsR(TCS_ID_);
            if (id == 0x44 || id == 0x4D || id == 0x10) { _colorConfig(); _colorOk[s] = true; found++; }
        }
    }
    _colorDeselectAll();
    Serial.print(F("[Robotrick] color sensors: ")); Serial.print(found); Serial.println(F("/4 موجود"));
}

bool Robotrick::_colorReadAvg(int slot, RTColorReading &out) {
    _colorSelect(slot);
    if (!_colorAck()) { _colorDeselectAll(); return false; }
    uint32_t sc = 0, sr = 0, sg = 0, sb = 0; uint8_t n = 0;
    for (uint8_t k = 0; k < RT_COLOR_AVG; k++) {
        uint32_t t0 = millis();
        while (millis() - t0 < 250) { if (_tcsR(TCS_STAT_) & 0x01) break; delay(5); }
        Wire.beginTransmission(RT_COLOR_ADDR);
        Wire.write(TCS_CMD_ | TCS_AUTO_ | TCS_CDATAL_);
        if (Wire.endTransmission(false) != 0) continue;
        Wire.requestFrom((uint8_t)RT_COLOR_ADDR, (uint8_t)8);
        if (Wire.available() < 8) continue;
        uint8_t d[8]; for (uint8_t i = 0; i < 8; i++) d[i] = Wire.read();
        sc += (uint16_t)(d[1] << 8) | d[0]; sr += (uint16_t)(d[3] << 8) | d[2];
        sg += (uint16_t)(d[5] << 8) | d[4]; sb += (uint16_t)(d[7] << 8) | d[6];
        n++;
    }
    _colorDeselectAll();
    if (n == 0) return false;
    out.c = sc / n; out.r = sr / n; out.g = sg / n; out.b = sb / n;
    float s = (float)out.r + out.g + out.b; if (s < 1) s = 1;
    out.nr = out.r / s; out.ng = out.g / s; out.nb = out.b / s;
    out.color = _colorClassify(out);
    return true;
}

RTColor Robotrick::_colorClassify(const RTColorReading &rd) {
    if (rd.c < RT_COLOR_BLACK_C) return RT_BLACK;      // معتم = اسود بالسطوع
    int best = -1; float bestD = 1e9;
    for (uint8_t i = 0; i < 6; i++) {
        if (!_colorRef[i].set) continue;
        float d = sq(rd.nr - _colorRef[i].nr) + sq(rd.ng - _colorRef[i].ng) + sq(rd.nb - _colorRef[i].nb);
        if (d < bestD) { bestD = d; best = i; }
    }
    return (best < 0) ? RT_UNKNOWN : (RTColor)best;
}

RTColor Robotrick::readColor(uint8_t sensor) {
    if (sensor < 1 || sensor > RT_COLOR_N) return RT_UNKNOWN;
    RTColorReading rd;
    if (!_colorReadAvg(sensor - 1, rd)) return RT_UNKNOWN;
    return rd.color;
}
void Robotrick::readAllColors(RTColor dst[4]) {
    for (uint8_t s = 0; s < 4; s++) dst[s] = readColor(s + 1);
}
bool Robotrick::readColorRGB(uint8_t sensor, RTColorReading &out) {
    if (sensor < 1 || sensor > RT_COLOR_N) return false;
    return _colorReadAvg(sensor - 1, out);
}
bool Robotrick::colorPresent(uint8_t sensor) {
    return (sensor >= 1 && sensor <= RT_COLOR_N) ? _colorOk[sensor - 1] : false;
}
void Robotrick::teachColor(RTColor color, uint8_t sensor) {
    if ((int)color < 0 || (int)color >= 6) { Serial.println(F("[color] لون غير صالح")); return; }
    if (sensor < 1 || sensor > RT_COLOR_N)  { Serial.println(F("[color] حساس 1..4")); return; }
    RTColorReading rd;
    if (!_colorReadAvg(sensor - 1, rd)) { Serial.println(F("[color] فشلت القراءة")); return; }
    _colorRef[color] = { rd.nr, rd.ng, rd.nb, true };
    Serial.print(F("[color] علّمت ")); Serial.print(colorName(color));
    Serial.print(F(" من S")); Serial.print(sensor); Serial.print(F(": "));
    Serial.print(rd.nr, 3); Serial.print(' '); Serial.print(rd.ng, 3); Serial.print(' '); Serial.println(rd.nb, 3);
}
void Robotrick::resetColorRefs() {
    for (uint8_t i = 0; i < 7; i++) _colorRef[i] = COLOR_DEFAULT[i];
    Serial.println(F("[color] refs reset للقيم الافتراضية"));
}
void Robotrick::printColorRefs() {
    Serial.println(F("[color] المراجع:"));
    for (uint8_t i = 0; i < 6; i++) {
        Serial.print(F("  ")); Serial.print(colorName((RTColor)i)); Serial.print(F(": "));
        if (_colorRef[i].set) {
            Serial.print(_colorRef[i].nr, 3); Serial.print(' ');
            Serial.print(_colorRef[i].ng, 3); Serial.print(' ');
            Serial.println(_colorRef[i].nb, 3);
        } else Serial.println(F("— غير متعلّم"));
    }
}
void Robotrick::setColorLED(bool on) { digitalWrite(RT_COLOR_LED_PIN, on ? HIGH : LOW); }

const char* Robotrick::colorName(RTColor c) {
    static const char* names[7] = { "احمر", "اخضر", "ازرق", "اصفر", "ابيض", "اسود", "؟" };
    return names[((int)c >= 0 && (int)c <= 6) ? (int)c : 6];
}

int Robotrick::servoAngle(uint8_t idx) {
    if (idx < 1 || idx > RT_SERVO_N) return -1;
    return _servoPos[idx - 1];
}

// جهة اليسار: speed موجب = forward
void Robotrick::_spinL(int spd) {
    if (RT_L_REVERSE) spd = -spd;
    spd = constrain(spd, -255, 255);
    digitalWrite(RT_L_DIR, spd >= 0 ? HIGH : LOW);
    analogWrite(RT_L_PWM, abs(spd));
}

// جهة اليمين: speed موجب = forward
void Robotrick::_spinR(int spd) {
    if (RT_R_REVERSE) spd = -spd;
    spd = constrain(spd, -255, 255);
    digitalWrite(RT_R_DIR, spd >= 0 ? HIGH : LOW);
    analogWrite(RT_R_PWM, abs(spd));
}

long Robotrick::encoderLeft()   { return _encL.read(); }
long Robotrick::encoderRight()  { return _encR.read(); }
void Robotrick::resetEncoders() { _encL.write(0); _encR.write(0); }

long Robotrick::countsForCM(float cm) {
    float countsPerMM = RT_COUNTS_PER_REV / (3.14159f * RT_WHEEL_DIAMETER_MM);
    return (long)(fabs(cm) * 10.0f * countsPerMM);
}

// ─────────────────────────────────────────────────────
//  LINE FOLLOWER — الـ 14 حساس الوسطى (index 5..18)
//  نتجنّب الحساسين الميتين (3, 24) والأطراف، emitters 26/27
// ─────────────────────────────────────────────────────
static const uint8_t QTR_PINS[RT_QTR_N] = {
    32, 35, 34, 37, 36, 39, 38, 41, 40, 43, 42, 45, 44, 47
};

void Robotrick::_lineBegin() {
    if (_qtrPinsInit) return;
    _qtr.setTypeRC();
    _qtr.setSensorPins(QTR_PINS, RT_QTR_N);
    _qtr.setEmitterPins(26, 27);
    _qtr.setTimeout(2500);
    pinMode(26, OUTPUT); digitalWrite(26, HIGH);
    pinMode(27, OUTPUT); digitalWrite(27, HIGH);
    _qtrPinsInit = true;
}

void Robotrick::lineReadRaw(uint16_t* dest25) {
    _lineBegin();
    _qtr.read(dest25);
}

uint16_t Robotrick::lineRead(uint16_t* dest25) {
    if (!_qtrReady) return 65535;
    return _qtr.readLineBlack(dest25);
}

// magic يتضمّن عدد الحساسات: لو غيّرت RT_QTR_N، المعايرة القديمة تُرفض تلقائياً
static const uint16_t QTR_CAL_MAGIC = (uint16_t)(0xCA00 | RT_QTR_N);

void Robotrick::lineCalibrate(uint8_t seconds) {
    _lineBegin();
    Serial.println(F("[Robotrick] LINE CAL — حرّك الروبوت يمين/يسار فوق الخط!"));
    delay(800);
    _qtr.resetCalibration();
    uint32_t endAt = millis() + (uint32_t)seconds * 1000UL;
    while (millis() < endAt) {
        _qtr.calibrate();
        if ((millis() % 500) < 15) Serial.print('.');
    }
    _qtrReady = true;
    lineSaveCalibration();                       // احفظ للأبد
    Serial.println(F(" done + SAVED to EEPROM. ما بدك تعيد المعايرة بعد اليوم."));
}

// احفظ min/max لكل حساس بالـ EEPROM
void Robotrick::lineSaveCalibration() {
    if (!_qtr.calibrationOn.initialized) return;
    int addr = RT_QTR_EEPROM_ADDR;
    EEPROM.put(addr, QTR_CAL_MAGIC); addr += 2;
    for (uint8_t i = 0; i < RT_QTR_N; i++) { EEPROM.put(addr, _qtr.calibrationOn.minimum[i]); addr += 2; }
    for (uint8_t i = 0; i < RT_QTR_N; i++) { EEPROM.put(addr, _qtr.calibrationOn.maximum[i]); addr += 2; }
}

// حمّل المعايرة المحفوظة (بدون تحريك). ترجع true إذا في معايرة صالحة.
bool Robotrick::lineLoadCalibration() {
    _lineBegin();
    uint16_t magic; EEPROM.get(RT_QTR_EEPROM_ADDR, magic);
    if (magic != QTR_CAL_MAGIC) return false;     // مافي معايرة محفوظة
    _qtr.calibrate();                             // دورة وحدة عشان يحجز مصفوفات min/max
    int addr = RT_QTR_EEPROM_ADDR + 2;
    for (uint8_t i = 0; i < RT_QTR_N; i++) { uint16_t v; EEPROM.get(addr, v); _qtr.calibrationOn.minimum[i] = v; addr += 2; }
    for (uint8_t i = 0; i < RT_QTR_N; i++) { uint16_t v; EEPROM.get(addr, v); _qtr.calibrationOn.maximum[i] = v; addr += 2; }
    _qtrReady = true;
    return true;
}

uint16_t Robotrick::linePosition() {
    if (!_qtrReady) return 12000;
    return _qtr.readLineBlack(_qtrVals);
}

// قراءة حيّة: حط الروبوت عالخط واقرأ. بيطبع قيمة كل حساس (0..1000)،
// أي حساس هو القمة (*)، والموقع المحسوب مقابل النص النظري.
// الهدف: تشوف لمّا الروبوت متمركز بالزبط، الموقع كم فعلاً.
void Robotrick::lineMonitor(uint32_t ms) {
    _lineBegin();
    if (!_qtrReady && !lineLoadCalibration()) {
        Serial.println(F("[Robotrick] ما في معايرة — اعمل lineCalibrate أول. (بطبع خام)"));
    }
    float mid = (RT_QTR_N - 1) * 1000.0f / 2.0f;   // النص النظري (=6500 لـ14 حساس)
    Serial.print(F("[Robotrick] LINE MONITOR — النص النظري = ")); Serial.println(mid, 0);
    Serial.println(F("  RAW = قراءة خام (RC): ~0-100 أبيض/انعكاس قوي، ~2500 = timeout (ولا انعكاس)"));
    uint16_t raw[RT_QTR_N];
    uint32_t endAt = millis() + ms;
    while (millis() < endAt) {
        // اقرأ الخام دايماً (هاد اللي بيكشف عطل الحساس/الإضاءة)
        _qtr.read(raw);
        if (_qtrReady) _qtr.readCalibrated(_qtrVals);

        // سطر RAW
        uint16_t rawMin = 65535, rawMax = 0;
        Serial.print(F("RAW"));
        for (uint8_t i = 0; i < RT_QTR_N; i++) {
            Serial.print(' '); Serial.print(raw[i]);
            if (raw[i] < rawMin) rawMin = raw[i];
            if (raw[i] > rawMax) rawMax = raw[i];
        }
        Serial.print(F("  | min=")); Serial.print(rawMin);
        Serial.print(F(" max=")); Serial.println(rawMax);

        // سطر CAL (لو في معايرة) + الموقع
        if (_qtrReady) {
            int peak = -1; uint16_t peakV = 0; uint32_t sum = 0;
            for (uint8_t i = 0; i < RT_QTR_N; i++) {
                sum += _qtrVals[i];
                if (_qtrVals[i] > peakV) { peakV = _qtrVals[i]; peak = i; }
            }
            float wsum = 0, tot = 0;
            for (int i = 0; i < RT_QTR_N; i++) {
                if (peak < 0 || abs(i - peak) > RT_LINE_WINDOW) continue;
                float w = (float)_qtrVals[i] * (float)_qtrVals[i];
                wsum += w * (i * 1000.0f); tot += w;
            }
            float pos = (tot > 0) ? (wsum / tot) : -1;
            Serial.print(F("CAL"));
            for (uint8_t i = 0; i < RT_QTR_N; i++) {
                Serial.print(' ');
                if (i == peak) Serial.print('*');
                Serial.print(_qtrVals[i]);
            }
            Serial.print(F("  | peak=idx")); Serial.print(peak);
            Serial.print(F("  pos=")); Serial.print(pos, 0);
            Serial.print(F("  mid=")); Serial.println(mid, 0);
        }
        delay(300);
    }
    Serial.println(F("[Robotrick] خلص المونيتور."));
}

// فحص اتجاه التوجيه — بدون حركة. بيحسب نفس تصحيح المتتبع ويقول:
// وين الخط (idx/pos)، وأي عجلة رح تتسرّع، ولوين رح يميل الروبوت.
// الاستعمال: حرّك الخط لجهة بإيدك، ولازم السهم يشير لنفس الجهة (يصحّح باتجاه الخط).
// لو أشار عكسها → RT_LINE_STEER_SIGN مقلوب.
void Robotrick::lineSteerCheck(uint32_t ms) {
    _lineBegin();
    if (!_qtrReady && !lineLoadCalibration()) {
        Serial.println(F("[Robotrick] ما في معايرة — اعمل k أول.")); return;
    }
    float mid = RT_QTR_CENTER;
    Serial.println(F("[Robotrick] STEER CHECK — حرّك الخط يمين/يسار وراقب السهم"));
    Serial.print(F("  STEER_SIGN=")); Serial.print(RT_LINE_STEER_SIGN);
    Serial.print(F("  CENTER=")); Serial.print((int)mid);
    Serial.print(F("  base=")); Serial.println(_lineBase);
    Serial.println(F("  القاعدة: السهم لازم يشير لنفس جهة الخط (يصحّح نحوه)"));
    uint32_t endAt = millis() + ms;
    while (millis() < endAt) {
        _qtr.readCalibrated(_qtrVals);
        int peak = -1; uint16_t peakV = 0; uint32_t sum = 0;
        for (uint8_t i = 0; i < RT_QTR_N; i++) {
            if (i == RT_QTR_DEAD_1 || i == RT_QTR_DEAD_2) continue;
            sum += _qtrVals[i];
            if (_qtrVals[i] > peakV) { peakV = _qtrVals[i]; peak = i; }
        }
        float wsum = 0, tot = 0;
        for (int i = 0; i < RT_QTR_N; i++) {
            if (i == RT_QTR_DEAD_1 || i == RT_QTR_DEAD_2) continue;
            if (peak < 0 || abs(i - peak) > RT_LINE_WINDOW) continue;
            float w = (float)_qtrVals[i] * (float)_qtrVals[i];
            wsum += w * (i * 1000.0f); tot += w;
        }
        if (sum < RT_LINE_PRESENT || tot <= 0) {
            Serial.println(F("  … لا خط (حط الحساس فوق الخط)"));
            delay(300); continue;
        }
        float pos   = wsum / tot;
        float err   = pos - mid;                         // موجب = خط عند idx أعلى
        float ratio = fabs(err) / mid; if (ratio > 1) ratio = 1;
        float kpEff = _lineKp * (1.0f + _lineKpBoost * ratio * ratio);
        float corr  = RT_LINE_STEER_SIGN * (kpEff * err);
        int l = constrain((int)(_lineBase + corr), _lineMin, _lineMax);
        int r = constrain((int)(_lineBase - corr), _lineMin, _lineMax);

        Serial.print(F("  peak=idx")); Serial.print(peak);
        Serial.print(F("  pos=")); Serial.print(pos, 0);
        Serial.print(err >= 0 ? F(" (خط لجهة idx-الأعلى)") : F(" (خط لجهة idx-الأدنى)"));
        Serial.print(F("  L=")); Serial.print(l);
        Serial.print(F(" R=")); Serial.print(r);
        // l>r → العجلة اليسار أسرع → الروبوت يميل يمين
        if (l > r + 2)      Serial.println(F("  → يميل يمين ►"));
        else if (r > l + 2) Serial.println(F("  → ◄ يميل يسار"));
        else                Serial.println(F("  → مستقيم"));
        delay(300);
    }
    Serial.println(F("[Robotrick] خلص فحص التوجيه."));
}

bool Robotrick::followLineToJunction(uint8_t nJunctions) { return _followLine(nJunctions, 0); }
bool Robotrick::followLineForCM(float cm)                { return _followLine(0, cm); }

//  المحرك المشترك: PD على موقع الخط + عدّ تقاطعات أو مسافة
//  ترجع true إذا نجح، false إذا ضاع الخط/انتهت المهلة
bool Robotrick::_followLine(uint8_t nJunctions, float cm) {
    if (!_qtrReady) {
        // جرّب تحمّل المعايرة المحفوظة أوتوماتيك (بدون تحريك)
        if (!lineLoadCalibration()) {
            Serial.println(F("[Robotrick] ERR: لا توجد معايرة — اعمل lineCalibrate() مرة وحدة."));
            return false;
        }
        Serial.println(F("[Robotrick] معايرة الخط محمّلة من EEPROM ✓"));
    }
    Serial.print(F("[Robotrick] LINE "));
    if (nJunctions) { Serial.print(F("to junction #")); Serial.println(nJunctions); }
    else            { Serial.print(cm); Serial.println(F("cm")); }

#if RT_LINE_PARK_SERVOS
    _lineParkServos();   // ضد تعارض QTR-RC مع السيرفو (Timer5)
#endif

    float cmPerCount = (3.14159f * RT_WHEEL_DIAMETER_MM / RT_COUNTS_PER_REV) / 10.0f;
    long  targetCounts = (cm > 0) ? (long)(cm / cmPerCount) : 0;
    resetEncoders();

    float posFilt = RT_QTR_CENTER, errPrev = 0;
    int   lastDir = 0;                       // آخر جهة انحراف (للاسترداد)
    uint8_t found = 0;
    uint32_t timeout    = millis() + RT_LINE_TIMEOUT;
    uint32_t clearUntil = 0, darkSince = 0, lostSince = 0;
    uint32_t tPrev = micros(), dbgPrev = 0;

    while (millis() < timeout) {
        _serviceMotor4();   // خدمة الرافعة async أثناء تتبع الخط
        // نقرأ calibrated (0..1000) ونحسب الموقع بأنفسنا — نتجاهل الحساسات الخربانة
        _qtr.readCalibrated(_qtrVals);

        uint32_t nowU = micros();
        float dt = (nowU - tPrev) * 1e-6f;
        tPrev = nowU;
        if (dt <= 0 || dt > 0.2f) dt = 0.005f;

        // ── الموقع: أعلى حساس + centroid بأوزان تربيعية حوالينه فقط ──
        // (أدق بكثير + يتجاهل الضوضاء البعيدة وذراع التقاطع)
        int peak = -1; uint16_t peakV = 0;
        uint32_t sum = 0; uint8_t darkN = 0;
        for (uint8_t i = 0; i < RT_QTR_N; i++) {
            if (i == RT_QTR_DEAD_1 || i == RT_QTR_DEAD_2) continue;  // خربان — تجاهل
            uint16_t val = _qtrVals[i];
            sum += val;
            if (val >= RT_JUNCT_DARK_TH) darkN++;
            if (val > peakV) { peakV = val; peak = i; }
        }
        float wsum = 0, tot = 0;
        for (int i = 0; i < RT_QTR_N; i++) {
            if (i == RT_QTR_DEAD_1 || i == RT_QTR_DEAD_2) continue;
            if (peak < 0 || abs(i - peak) > RT_LINE_WINDOW) continue;   // بس حوالين القمة
            float w = (float)_qtrVals[i] * (float)_qtrVals[i];         // أوزان تربيعية = أحدّ
            wsum += w * (i * 1000.0f);
            tot  += w;
        }
        // الموقع 0..13000 (6500 = بالنص). لو ما في خط، خلّي آخر موقع
        uint16_t pos = (tot > 0) ? (uint16_t)(wsum / tot) : (uint16_t)posFilt;

        // debug مباشر أثناء التتبع
        if (millis() - dbgPrev >= 150) {
            dbgPrev = millis();
            Serial.print(F("dbg pos=")); Serial.print(pos);
            Serial.print(F(" sum="));    Serial.print(sum);
            Serial.print(F(" dark="));   Serial.println(darkN);
        }

        // ── تقاطع؟ ────────────────────────────────
        if (nJunctions && darkN >= RT_JUNCT_DARK_N && millis() >= clearUntil) {
            if (!darkSince) darkSince = millis();
            if (millis() - darkSince >= RT_JUNCT_DEBOUNCE_MS) {
                found++;
                Serial.print(F("  junction ")); Serial.println(found);
                if (found >= nJunctions) {
                    // كمّل قدّام ليصير محور العجلات فوق التقاطع
                    resetEncoders();
                    long off = (long)(RT_LINE_JUNCT_OFFSET_CM / cmPerCount);
                    setMotors(_lineBase, _lineBase);
                    uint32_t offT = millis() + 2000;
                    while (abs(_encL.read()) < off && millis() < offT) {}
                    stop();
                    Serial.println(F("  → arrived at junction"));
                    return true;
                }
                darkSince = 0;
                clearUntil = millis() + RT_JUNCT_CLEAR_MS;
            }
        } else darkSince = 0;

        // ── خط ضايع؟ ──────────────────────────────
        if (sum < RT_LINE_PRESENT) {
            if (!lostSince) lostSince = millis();
            if (millis() - lostSince > RT_LINE_LOST_MS) {
                stop();
                Serial.println(F("[Robotrick] LINE LOST — abort"));
                return false;
            }
            // استرداد: قوس ناعم باتجاه آخر انحراف
            if (lastDir >= 0) setMotors(_lineBase, _lineMin);
            else              setMotors(_lineMin,  _lineBase);
            continue;
        }
        lostSince = 0;

        // ── مسافة؟ ────────────────────────────────
        if (targetCounts && abs(_encL.read()) >= targetCounts) {
            stop();
            Serial.println(F("  → distance done"));
            return true;
        }

        // ── PD على موقع الخط ──────────────────────
        // أثناء التقاطع (كل الحساسات غامقة) القراءة مشوّشة → جمّد الموقع (لا يجنّ)
        bool atJunction = (darkN >= RT_JUNCT_DARK_N);
        if (!atJunction)
            posFilt += RT_LINE_ALPHA * ((float)pos - posFilt);
        float err  = posFilt - (float)RT_QTR_CENTER;  // موجب = الخط على اليمين
        lastDir = (err > 200) ? 1 : (err < -200 ? -1 : lastDir);
        // deadband: قريب من النص = امشي مستقيم (يمنع الرجّة)
        if (err > -_lineDeadband && err < _lineDeadband) err = 0;
        float dErr = (err - errPrev) / dt;
        errPrev = err;

        // نسبة البعد عن النص 0..1 (0=بالنص، 1=عالطرف/كوع حاد)
        float ratio = fabs(err) / (float)RT_QTR_CENTER;
        if (ratio > 1.0f) ratio = 1.0f;

        // gain scheduling: KP ناعم بالنص، يقوى عالكوع الحاد
        float kpEff = _lineKp * (1.0f + _lineKpBoost * ratio * ratio);
        float corr = RT_LINE_STEER_SIGN * (kpEff * err + _lineKd * dErr * 0.001f);

        // curvature speed: بطّئ الأساس عالكوع الحاد ليلحق يلف بدل ما يطير
        int base = _lineBase - (int)((_lineBase - _lineSlow) * ratio);

        // تجاهل التقاطع: بأمر followLineForCM (nJunctions==0) اعبر التقاطع مستقيم
        // بدون تصحيح ولا تبطئة — يمنع "الجنون" عند الـ + و T
        if (atJunction && nJunctions == 0) { corr = 0; base = _lineBase; }

        int l = constrain((int)(base + corr), _lineMin, _lineMax);
        int r = constrain((int)(base - corr), _lineMin, _lineMax);
        setMotors(l, r);
    }

    stop();
    Serial.println(F("[Robotrick] LINE TIMEOUT — abort"));
    return false;
}

// ─────────────────────────────────────────────────────
//  LINE FOLLOWER 2 — خوارزمية بديلة (أسلوب MegaShield)
//  centroid كامل + كشف تقاطع = الطرفين غامقين معاً + force-straight
// ─────────────────────────────────────────────────────
bool Robotrick::followLine2(float cm) {
    if (!_qtrReady && !lineLoadCalibration()) {
        Serial.println(F("[Robotrick] ERR: no line calibration — LINECAL first."));
        return false;
    }
    Serial.print(F("[Robotrick] LINE2 ")); Serial.print(cm); Serial.println(F("cm"));

#if RT_LINE_PARK_SERVOS
    _lineParkServos();   // ضد تعارض QTR-RC مع السيرفو (Timer5)
#endif

    float cmPerCount   = (3.14159f * RT_WHEEL_DIAMETER_MM / RT_COUNTS_PER_REV) / 10.0f;
    long  targetCounts = (cm > 0) ? (long)(cm / cmPerCount) : 0;
    resetEncoders();

    const float center = (RT_QTR_N - 1) / 2.0f;   // 6.5 لـ 14 حساس
    int16_t  lastError  = 0;
    uint8_t  lostCount  = 0;
    uint32_t forceUntil = 0, dbgPrev = 0;
    uint32_t timeout    = millis() + RT_LINE_TIMEOUT;

    while (millis() < timeout) {
        _serviceMotor4();   // خدمة الرافعة async أثناء تتبع الخط
        _qtr.readCalibrated(_qtrVals);

        long sumW = 0, sumV = 0;
        uint8_t darkL = 0, darkR = 0;
        for (uint8_t i = 0; i < RT_QTR_N; i++) {
            uint16_t v = _qtrVals[i];
            if (v > RT_JUNCT_DARK_TH) {                 // غامق = على الخط
                int sig = v - RT_JUNCT_DARK_TH;
                sumW += (long)i * sig;
                sumV += sig;
                if (i < RT_LF2_OUTER_N)                  darkL++;   // الطرف اليسار
                else if (i >= RT_QTR_N - RT_LF2_OUTER_N) darkR++;   // الطرف اليمين
            }
        }

        // وصل المسافة؟
        if (targetCounts && abs(_encL.read()) >= targetCounts) {
            stop(); Serial.println(F("  → distance done")); return true;
        }

        // تقاطع = الطرفين غامقين معاً (عبور عمودي يعتّم الجهتين)
        bool nowJunc = (darkL >= RT_LF2_OUTER_DARK) && (darkR >= RT_LF2_OUTER_DARK);
        if (nowJunc && millis() > forceUntil + 300) {
            forceUntil = millis() + RT_LF2_FORCE_MS;
            Serial.println(F("  junction → force straight"));
        }
        bool forceStraight = (millis() < forceUntil);

        if (millis() - dbgPrev >= 200) {
            dbgPrev = millis();
            int lp = (sumV > 0) ? (int)(sumW / sumV) : -1;
            Serial.print(F("LF2 pos=")); Serial.print(lp);
            Serial.print(F(" L=")); Serial.print(darkL);
            Serial.print(F(" R=")); Serial.print(darkR);
            Serial.print(F(" sV=")); Serial.print(sumV);
            if (forceStraight) Serial.print(F(" STR"));
            Serial.println();
        }

        // ضياع الخط
        if (sumV == 0 && !forceStraight) {
            if (++lostCount >= 25) { stop(); Serial.println(F("  LINE2 LOST")); return false; }
            int rec = (lastError < 0) ? -_lineBase / 2 : _lineBase / 2;
            setMotors(rec, -rec);       // دوّر باتجاه آخر مكان شفت فيه الخط
            continue;
        }
        if (sumV > 0) lostCount = 0;

        // PD (معطّل أثناء عبور التقاطع = يمشي مستقيم)
        int correction = 0;
        if (!forceStraight && sumV > 0) {
            int16_t error = (int16_t)(((float)sumW / sumV - center) * 10.0f);
            int16_t deriv = error - lastError;
            lastError = error;
            correction = (int)(RT_LF2_KP * error + RT_LF2_KD * deriv);
        }
        correction = constrain(correction, -(int)_lineBase, (int)_lineBase);

        int c = RT_LINE_STEER_SIGN * correction;
        int l = constrain((int)_lineBase + c, _lineMin, _lineMax);
        int r = constrain((int)_lineBase - c, _lineMin, _lineMax);
        setMotors(l, r);
    }

    stop();
    Serial.println(F("  LINE2 timeout"));
    return false;
}

// ─────────────────────────────────────────────────────
//  GYRO low-level
// ─────────────────────────────────────────────────────
void Robotrick::_gyroWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(RT_GYRO_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}

bool Robotrick::_gyroReadBuf(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(RT_GYRO_ADDR);
    Wire.write(reg | 0x80);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(RT_GYRO_ADDR, (int)len);
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

int16_t Robotrick::_readGz() {
    uint8_t buf[6];
    if (!_gyroReadBuf(REG_OUT_X_L, buf, 6)) return 0;
    return (int16_t)((buf[5] << 8) | buf[4]);   // Z axis = yaw
}

bool Robotrick::_gyroInit() {
    uint8_t id = 0;
    _gyroReadBuf(REG_WHO_AM_I, &id, 1);
    if (id != WHO_AM_I_VAL) return false;
    _gyroWrite(REG_CTRL4, 0x00);
    _gyroWrite(REG_CTRL1, 0x6F);   // ODR on, all axes
    delay(100);
    return true;
}

void Robotrick::_updateHeading() {
    _serviceMotor4();           // خدمة الرافعة async بالخلفية (كل حلقات الحركة تنادي هون)
    uint32_t now = micros();
    float dt = (now - _lastMicros) * 1e-6f;
    _lastMicros = now;
    float gz = (_readGz() - _gzBias) * RT_GYRO_SENS;
    _gyroRate = gz;                              // سرعة الدوران الحالية (للتخميد D)
    if (gz > RT_GYRO_DEADBAND || gz < -RT_GYRO_DEADBAND)
        _heading += gz * dt;
}
