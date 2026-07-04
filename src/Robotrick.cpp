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
      _gzBias(0), _heading(0), _gyroRate(0), _lastMicros(0) {}

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
    float vmax = RT_STRAIGHT_SPEED, a = RT_STRAIGHT_ACCEL, d = RT_STRAIGHT_ACCEL;
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
        float p  = RT_DIST_KP * posErr;
        float dd = RT_DIST_KD * velErr;
        if (RT_DIST_KI > 0) {                             // anti-windup integrator
            float u_pd = ff + p + dd;
            bool sat = (u_pd >= 255 || u_pd <= -255);
            if (!(sat && (posErr > 0)))
                integ = constrain(integ + posErr * dt, -255.0f / RT_DIST_KI, 255.0f / RT_DIST_KI);
        }
        float baseMag = ff + p + RT_DIST_KI * integ + dd;
        // أرضية: إذا لسا ما وصل، لازم PWM كافٍ يتحرّك (يمنع التوقف القصير)
        if (posErr > RT_POS_TOL && baseMag < RT_PWM_STATIC) baseMag = RT_PWM_STATIC;
        baseMag = constrain(baseMag, 0.0f, 255.0f);

        // ── heading hold (استقامة) — حسب RT_STRAIGHT_MODE ─
        float corr = 0;
        // MODE 1 أو 3 → جايرو PD
        if (RT_STRAIGHT_MODE == 1 || RT_STRAIGHT_MODE == 3) {
            float herr = _heading;
            if (herr > -RT_STRAIGHT_DEADBAND && herr < RT_STRAIGHT_DEADBAND) herr = 0;
            corr += RT_STEER_SIGN * (RT_STRAIGHT_KP * herr + RT_STRAIGHT_KD * _gyroRate);
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
    float slowStart = targetMag - RT_TURN_FAST_DEG;
    if (slowStart < 0) slowStart = 0;

    // Phase 1: سرعة كاملة لحد ما نقرب من الهدف
    _spinL(-dir * RT_TURN_FAST);
    _spinR( dir * RT_TURN_FAST);

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

        int spd = (int)(RT_TURN_KP * err);
        spd = constrain(spd, RT_TURN_SLOW_MIN, RT_TURN_SLOW_MAX);
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
}

void Robotrick::motor4For(int speed, uint32_t ms) {
    motor4(speed);
    uint32_t endAt = millis() + ms;
    while (millis() < endAt) {}
    motor4Stop();
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
                    setMotors(RT_LINE_BASE, RT_LINE_BASE);
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
            if (lastDir >= 0) setMotors(RT_LINE_BASE, RT_LINE_MIN);
            else              setMotors(RT_LINE_MIN,  RT_LINE_BASE);
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
        posFilt += RT_LINE_ALPHA * ((float)pos - posFilt);
        float err  = posFilt - (float)RT_QTR_CENTER;  // موجب = الخط على اليمين
        lastDir = (err > 200) ? 1 : (err < -200 ? -1 : lastDir);
        // deadband: قريب من النص = امشي مستقيم (يمنع الرجّة)
        if (err > -RT_LINE_DEADBAND && err < RT_LINE_DEADBAND) err = 0;
        float dErr = (err - errPrev) / dt;
        errPrev = err;

        float corr = RT_LINE_STEER_SIGN * (RT_LINE_KP * err + RT_LINE_KD * dErr * 0.001f);

        int l = constrain((int)(RT_LINE_BASE + corr), RT_LINE_MIN, RT_LINE_MAX);
        int r = constrain((int)(RT_LINE_BASE - corr), RT_LINE_MIN, RT_LINE_MAX);
        setMotors(l, r);
    }

    stop();
    Serial.println(F("[Robotrick] LINE TIMEOUT — abort"));
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
    uint32_t now = micros();
    float dt = (now - _lastMicros) * 1e-6f;
    _lastMicros = now;
    float gz = (_readGz() - _gzBias) * RT_GYRO_SENS;
    _gyroRate = gz;                              // سرعة الدوران الحالية (للتخميد D)
    if (gz > RT_GYRO_DEADBAND || gz < -RT_GYRO_DEADBAND)
        _heading += gz * dt;
}
