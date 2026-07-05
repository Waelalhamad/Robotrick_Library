// =====================================================
//  Robotrick — WRO Junior 2026 movement library
//  Arduino Mega 2560 + Robotrick Mega Shield
//
//  ركّز كل المعايرة هون. غيّر القيم فوق، استعمل الحركات تحت.
//
//  Drive base = M1 (left) + M2 (right) فقط — لأن:
//    • encoders تبعهم على بنات interrupt (19/18, 2/3) = دقيقة
//    • نتجنب M3_EN=D52 (يتعارض مع QTR25)
//    • نتجنب M4 encoder على D0/D1 (يتعارض مع Serial)
//
//  الحركات كلها blocking + فيها timeout أمان.
// =====================================================
#ifndef ROBOTRICK_H
#define ROBOTRICK_H

#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <QTRSensors.h>
#include <EEPROM.h>
#include <Servo.h>

// ─────────────────────────────────────────────────────
//  ███  CONFIG — عدّل من هون فقط  ███
// ─────────────────────────────────────────────────────

// ── Motor pins (M1 = left, M2 = right) ─────────────
#define RT_L_PWM    4
#define RT_L_DIR   25
#define RT_L_EN    24
#define RT_R_PWM    7
#define RT_R_DIR   A12
#define RT_R_EN    A15

// ── Motor 4 (اختياري — رافعة/آلية) ─────────────────
#define RT_M4_PWM    5
#define RT_M4_DIR   17
#define RT_M4_EN    14
#define RT_M4_REVERSE  false   // اقلب لو الرافعة بتتحرك بالاتجاه المعكوس
// ملاحظة: انكودر M4 على D0/D1 (Serial) — ما نستعمله؛ التحكّم بالوقت.
#define RT_LIFT_SPEED   200    // سرعة liftUp/liftDown (0..255)
#define RT_LIFT_UP_SIGN   1    // إذا liftUp بينزل بدل ما يرفع → خليها -1

// عكس اتجاه كل جهة (المحركات متقابلة فيزيائياً)
// إذا محرك لف غلط — اقلب قيمته.
#define RT_L_REVERSE  true
#define RT_R_REVERSE  false

// ── Encoder pins ───────────────────────────────────
#define RT_ENC_L_A   19
#define RT_ENC_L_B   18
#define RT_ENC_R_A    2
#define RT_ENC_R_B    3

// ── Robot geometry ─────────────────────────────────
#define RT_WHEEL_DIAMETER_MM   80.0f
#define RT_WHEEL_BASE_MM      150.0f   // معامل دقة اللف (للمرجع — اللف بالجايرو)
#define RT_COUNTS_PER_REV      493     // معايرة المسافة: 478×(100/97) — دقيق على 100cm

// ── Speeds (0–255) ─────────────────────────────────
#define RT_DRIVE_SPEED   120   // سرعة المشي المستقيم
#define RT_DRIVE_MIN      80   // أدنى سرعة (لازم تكفي تحرّك الروبوت بدون ما يطفي/يطنّ)
#define RT_TURN_FAST     120   // Phase 1: سرعة اللف السريعة
#define RT_TURN_SLOW_MAX  75   // Phase 2: أقصى سرعة بطيئة
#define RT_TURN_SLOW_MIN  50   // Phase 2: أدنى سرعة (ما يطفي)

// ── Straight-drive correction MODE ─────────────────
//   0 = OFF      → بدون تصحيح، فقط trim (أبسط شي)
//   1 = GYRO     → تصحيح بالجايرو (الأفضل لو في انزلاق)
//   2 = ENCODER  → موازنة عدّات اليسار/اليمين (بدون جايرو)
//   3 = BOTH     → جايرو + إنكودر مع بعض
#define RT_STRAIGHT_MODE  3

// ── Straight-drive heading hold (gyro PD) — يشتغل بالـ MODE 1/3
#define RT_STRAIGHT_KP    1.8f  // قوة الرجوع للخط المستقيم (زِد = أقوى لكن يبدأ يهتز)
#define RT_STRAIGHT_KD    0.6f  // التخميد — يمنع التذبذب يمين/يسار (زِد لو لسا يهتز)
#define RT_STRAIGHT_DEADBAND 1.0f  // تجاهل الانحراف الأصغر من هيك (deg) — يمنع الرعشة
#define RT_STEER_SIGN     -1    // انقلبت لـ -1: الجايرو كان يلف بدل ما يمشي مستقيم (دائرة)

// ── Encoder-difference correction — يشتغل بالـ MODE 2/3
#define RT_ENC_KP        -0.5f  // سالب: كان يلف بدل ما يصحّح. لا تخليها موجبة

#define RT_RAMP_MS      400      // مدة التسارع الناعم بالبداية (ms) — يمنع النطّة
#define RT_DECEL_CM      8.0f     // يبطّئ قبل الهدف بهالمسافة — وقوف ناعم بدون drift

// موازنة المحركين (trim): موجب = اليمين أسرع فنبطّئه + نسرّع اليسار
// إذا لسا ينحرف يمين زِدها، إذا انحرف يسار قلّلها (ممكن تصير سالبة)
#define RT_MOTOR_TRIM     -1

// ══════════════════════════════════════════════════
//  ██  v2 PHASE A — planned-trajectory forward/backward  ██
//  (forward/backward بيستعملوا هالقيم. لازم تعمل calibrateSpeed أول مرة)
// ══════════════════════════════════════════════════
// حركة مخطّطة: تسارع → سرعة ثابتة → تباطؤ → وقوف دقيق
#define RT_STRAIGHT_SPEED   25.0f  // سرعة الرحلة (cm/s) — خليها < السرعة القصوى
#define RT_STRAIGHT_ACCEL   50.0f  // التسارع/التباطؤ (cm/s^2) — أكبر = أسرع بس ممكن ينزلق

// Feedforward: تحويل السرعة المطلوبة → PWM  (من calibrateSpeed)
#define RT_PWM_STATIC       85.0f  // الـ PWM اللي يبلّش يتحرّك عنده (احتكاك)
#define RT_KV                5.0f  // PWM إضافي لكل cm/s

// Distance tracking PID (يلاحق الخطة)
#define RT_DIST_KP          9.0f   // قوة ملاحقة الموقع
#define RT_DIST_KD          2.0f   // تخميد (على فرق السرعة)
#define RT_DIST_KI          0.0f   // عادة 0 (زِد قليل لو بيوقف قصير دايماً)

// Completion (يخلص لما: وصل + بطيء + ثابت شوي)
#define RT_POS_TOL          1.0f   // cm — سماحية الوصول (واقعية؛ 0.3 ضيّقة كتير)
#define RT_SPEED_TOL        1.0f   // cm/s
#define RT_SETTLE_MS        80     // ms ثابت قبل ما يعتبر خلص
#define RT_STUCK_MS        300     // إذا خلصت الخطة وواقف مكانه هالمدة → اعتبره وصل (يمنع الطنين)

// Stall safety (يدفع بقوة بس مو ماشي → يوقف)
#define RT_STALL_SPEED      1.0f   // cm/s
#define RT_STALL_MS         400    // ms

// ── Turn control (gyro two-phase) ──────────────────
#define RT_TURN_DIR        -1      // إذا turnLeft بيلف يمين (معكوس) → خليها -1
#define RT_TURN_FAST_DEG  45.0f  // ابدأ تبطّئ قبل هالزاوية من الهدف
#define RT_TURN_KP         3.5f   // P-gain للمرحلة البطيئة
#define RT_TURN_TOLERANCE  1.5f   // توقف ضمن هالزاوية
#define RT_TURN_BRAKE_MS   30     // فرملة عكسية بعد الوصول

// ── Pivot turn (عجلة وحدة تتحرك، التانية واقفة) ──────
#define RT_PIVOT_FAST    120     // سرعة العجلة المتحركة Phase 1 (عجلة وحدة = بدها قوة أكثر)
#define RT_PIVOT_SLOW     75     // Phase 2 أقصى سرعة
#define RT_PIVOT_MIN      55     // Phase 2 أدنى سرعة (ما تطفي)
#define RT_PIVOT_STOP_DEG 0.0f   // يوقف قبل الهدف بهالزاوية (تعويض الاندفاع) — زِد لو لسا يزيد عن 90
#define RT_PIVOT_BRAKE   90     // قوة الفرملة العكسية (counter-spin بالعجلتين متل turn)

// ══════════════════════════════════════════════════
//  ██  v2 PHASE 3 — LINE FOLLOWER (الـ 10 حساسات الوسطى)  ██
//  لازم lineCalibrate() مرة كل تشغيل (حرّك الروبوت يدوياً فوق الخط)
// ══════════════════════════════════════════════════
#define RT_QTR_N          14      // 14 حساس وسطى (index 5..18) — مجال أوسع = ما يضيع بسهولة
#define RT_QTR_CENTER   8500      // (RT_QTR_N-1)*1000/2 — النص
#define RT_QTR_EEPROM_ADDR  0     // مكان حفظ معايرة الخط بالـ EEPROM (تنحفظ مرة، تفضل للأبد)

#define RT_LINE_BASE       50     // سرعة الأساس (فوق عتبة الحركة ~90 عشان يتحرّك أصلاً)
#define RT_LINE_MAX       100     // أقصى سرعة عجلة (نافذة واسعة = قدرة لف قوية)
#define RT_LINE_MIN        30     // أدنى سرعة عجلة (الداخلية تبطّئ كتير عالكوع الحاد)
#define RT_LINE_KP       0.03f  // P على موقع الخط (أوطى = أنعم على المستقيم)
#define RT_LINE_KD       0.060f  // D — تخميد يمنع التذبذب
#define RT_LINE_ALPHA     0.15f   // فلتر الموقع أقوى (أصغر = أنعم ضد الرجّة)
#define RT_LINE_STEER_SIGN  -1    // انقلبت: كان يصحّح بعكس الاتجاه ويفلت عن الخط
#define RT_LINE_DEADBAND  700     // نطاق أوسع: قريب من النص = امشي مستقيم بدون تصحيح (ضد الرجّة)
#define RT_LINE_WINDOW      3     // نحسب الموقع حوالين أعلى حساس ±هيك (أدق، يتجاهل الضوضاء)
#define RT_LINE_KP_BOOST  5.0f    // gain scheduling: كل ما بعد الخط عن النص، KP يزيد (يمسك الكوع)
#define RT_LINE_SLOW       70     // سرعة الأساس عند الكوع الحاد (يبطّئ ليلحق يلف)

// ── Line-Follower 2 (خوارزمية بديلة للتجربة — أسلوب MegaShield) ──
//   الفرق: كشف التقاطع = الطرفين غامقين معاً، centroid كامل، KP ثابت
#define RT_LF2_KP         1.0f    // قوة التوجيه (الخطأ = (المركز − النص) × 10)
#define RT_LF2_KD         0.5f    // تخميد
#define RT_LF2_OUTER_N     3      // كم حساس بكل طرف (يسار/يمين)
#define RT_LF2_OUTER_DARK  2      // كم غامق بكل طرف معاً ليعتبر تقاطع
#define RT_LF2_FORCE_MS  400      // مدة المشي مستقيم عبر التقاطع

// ما في حساسات ميتة بالوسط 10 (99 = معطّل)
#define RT_QTR_DEAD_1     99
#define RT_QTR_DEAD_2     99

// كشف التقاطع (junction): كم حساس غامق مع بعض
#define RT_JUNCT_DARK_N    10     // من 14 حساس: ~10 غامقة = تقاطع (الخط النحيف 1-3 بس)
#define RT_JUNCT_DARK_TH  600     // عتبة "غامق" للحساس الواحد (0..1000)
#define RT_JUNCT_DEBOUNCE_MS 30   // لازم يظل تقاطع هالمدة (ضد الغلط)
#define RT_JUNCT_CLEAR_MS   250   // بعد تقاطع، تجاهل الكشف هالمدة
#define RT_LINE_JUNCT_OFFSET_CM 5.0f // كمّل قدّام بعد التقاطع (يوسّط العجلات فوقه)

// أمان الخط
#define RT_LINE_PRESENT    700    // مجموع أدنى ليعتبر "في خط" — منخفض لأن الحساس مركّب عالي
#define RT_LINE_LOST_MS    900    // مدة البحث قبل ما يستسلم
#define RT_LINE_TIMEOUT  15000    // مهلة أمان للأمر كله (ms)

// ── Gyro (L3GD20H @ I2C 0x6B) ──────────────────────
#define RT_GYRO_ADDR    0x6B
#define RT_GYRO_SENS    0.00875f  // dps per LSB
#define RT_GYRO_DEADBAND 1.5f     // تجاهل الضوضاء الصغيرة

// ── Servos (حتى 3 سيرفو — رقمها 1 و 2 و 3) ─────────
#define RT_SERVO_N          3
#define RT_SERVO1_PIN      A2
#define RT_SERVO2_PIN      A3
#define RT_SERVO3_PIN      A4
// حدود نبضة السيرفو (µs) — عايرها لو السيرفو ما بيوصل 0/180 صح
#define RT_SERVO_MIN_US   544     // نبضة أقل زاوية (افتراضي مكتبة Servo)
#define RT_SERVO_MAX_US  2400     // نبضة أعلى زاوية
// حدود الزاوية المسموحة (حماية ميكانيكية — لو السيرفو بيصطدم قبل 0/180)
#define RT_SERVO_MIN_DEG    0
#define RT_SERVO_MAX_DEG  180
// سرعة الحركة الناعمة الافتراضية (درجة/ثانية) — 0 = فوري
#define RT_SERVO_SPEED    120

// ── Safety timeouts (ms) ───────────────────────────
#define RT_MOVE_TIMEOUT  20000
#define RT_TURN_TIMEOUT   5000

// ─────────────────────────────────────────────────────
//  ███  END CONFIG  ███
// ─────────────────────────────────────────────────────

class Robotrick {
public:
    Robotrick();

    // ── Setup ──────────────────────────────────────
    bool begin(uint32_t baud = 115200);  // init + gyro calibrate
    void calibrateGyro();                 // أعد قياس الـ bias (ثبّت الروبوت)
    void calibrateSpeed();                // قيس PWM_STATIC و KV (يحتاج ~1م مساحة قدام)

    // ── Movement (blocking) ────────────────────────
    void forward(float cm);     // مستقيم لقدام + heading hold
    void backward(float cm);    // مستقيم لخلف
    void turnLeft(float deg);   // لف يسار بالجايرو (spin — الجهتين)
    void turnRight(float deg);  // لف يمين بالجايرو (spin — الجهتين)
    void pivot(char wheel, float deg);  // pivot: عجلة وحدة تتحرك، التانية واقفة
                                // wheel: 'L' أو 'R' (اللي تتحرك) — deg موجب=قدام، سالب=خلف
    void stop();                // وقوف

    // ── Drive tuning (عدّل سرعة forward والـ PID لايف بدون recompile) ──
    void  setDriveSpeed(float cmPerSec);            // سرعة رحلة forward/backward (cm/s)
    void  setDriveAccel(float cmPerSec2);           // تسارع/تباطؤ الرحلة (cm/s^2)
    void  setDrivePID(float kp, float kd, float ki = 0); // PID ملاحقة المسافة
    void  setHeadingPD(float kp, float kd, float deadband = -1); // استقامة الجايرو (PD)
    void  resetDriveTuning();                       // رجّع السرعة والـ PID والاستقامة لقيم CONFIG
    float getDriveSpeed();                          // السرعة الحالية (cm/s)
    void  printDriveTuning();                       // اطبع القيم الحالية

    // ── Sensors ────────────────────────────────────
    float heading();            // الزاوية الحالية (deg)
    void  resetHeading();       // صفّر الزاوية
    void  update();             // integrate gyro continuously (call from loop when idle)

    // ── Line follower (QTR-25) ─────────────────────
    void lineCalibrate(uint8_t seconds = 5);       // حرّك الروبوت يدوياً فوق الخط (يحفظ EEPROM)
    bool lineLoadCalibration();                    // حمّل المعايرة المحفوظة — بدون تحريك! true=نجح
    void lineSaveCalibration();                    // احفظ المعايرة الحالية بالـ EEPROM
    bool followLineToJunction(uint8_t nJunctions = 1);  // اتبع الخط لحد التقاطع الـ n
    bool followLineForCM(float cm);                // اتبع الخط لمسافة محددة
    bool followLine2(float cm);                    // خوارزمية بديلة (أسلوب MegaShield) للتجربة
    uint16_t linePosition();                       // 0..24000 (12000 = بالنص)
    void     lineReadRaw(uint16_t* dest25);        // قراءة خام (بدون calibration)
    uint16_t lineRead(uint16_t* dest25);           // قراءة calibrated + الموقع (65535 = مش معاير)

    // ── Motor 4 (رافعة/آلية) ───────────────────────
    void  motor4(int speed);                 // -255..255، 0 = وقوف (non-blocking)
    void  motor4For(int speed, uint32_t ms); // شغّله مدة (ms) ثم يوقف (blocking)
    void  motor4Stop();
    void  liftUp(uint32_t ms);               // ارفع الرافعة مدة (ms) — blocking
    void  liftDown(uint32_t ms);             // نزّل الرافعة مدة (ms) — blocking

    // ── Servos (idx = 1..3 على البنات A2/A3/A4) ────
    void  servoWrite(uint8_t idx, int angle);                    // روح للزاوية فوراً (0..180)
    void  servoMove(uint8_t idx, int angle,
                    uint16_t degPerSec = RT_SERVO_SPEED);        // حركة ناعمة بسرعة معيّنة (blocking)
    void  servoDetach(uint8_t idx);                              // حرّر السيرفو (يوقف الطنين/العزم)
    void  servoAttachAll();                                      // فعّل السيرفوهات الثلاثة
    int   servoAngle(uint8_t idx);                               // آخر زاوية مكتوبة (-1 = غير مفعّل)

    // ── Low-level (لو احتجت تتحكم يدوي) ────────────
    void  setMotors(int left, int right);   // -255..255 لكل جهة
    long  encoderLeft();
    long  encoderRight();
    void  resetEncoders();

private:
    Encoder _encL, _encR;
    float   _gzBias;
    float   _heading;
    float   _gyroRate;     // سرعة الدوران الحالية (deg/s) — للتخميد D
    uint32_t _lastMicros;

    // drive tuning قابلة للتعديل لايف (تبدأ من قيم الـ CONFIG)
    float _driveSpeed, _driveAccel;      // = RT_STRAIGHT_SPEED / RT_STRAIGHT_ACCEL
    float _distKp, _distKd, _distKi;     // = RT_DIST_KP / KD / KI
    float _hdgKp, _hdgKd, _hdgDeadband;  // = RT_STRAIGHT_KP / KD / DEADBAND (heading hold)

    // line follower
    QTRSensors _qtr;
    uint16_t   _qtrVals[RT_QTR_N];
    bool       _qtrReady = false;     // صار calibration؟
    bool       _qtrPinsInit = false;  // صارت تهيئة البنات؟
    void _lineBegin();                // تهيئة بنات الـ QTR (مرة واحدة)
    bool _followLine(uint8_t nJunctions, float cm);  // المحرك المشترك

    // servos
    Servo    _servo[RT_SERVO_N];
    uint8_t  _servoPin[RT_SERVO_N];
    int16_t  _servoPos[RT_SERVO_N];   // آخر زاوية (-1 = ما تحرّك بعد)
    bool     _servoAttached[RT_SERVO_N];
    void     _servoBegin();           // سجّل البنات (بدون attach — كسول)
    bool     _servoEnsure(uint8_t i); // فعّل السيرفو i لو مش مفعّل؛ true=جاهز

    long  countsForCM(float cm);
    void  driveStraight(float cm, int dir);   // dir: +1 forward, -1 back
    void  turn(float deg);                     // +deg = left, -deg = right
    void  _spinL(int spd);
    void  _spinR(int spd);

    // gyro low-level
    void    _gyroWrite(uint8_t reg, uint8_t val);
    bool    _gyroReadBuf(uint8_t reg, uint8_t* buf, uint8_t len);
    int16_t _readGz();
    bool    _gyroInit();
    void    _updateHeading();
};

#endif  // ROBOTRICK_H
