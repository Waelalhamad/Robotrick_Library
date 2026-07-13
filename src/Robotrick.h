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

// ── محرك الرافعة — على درايفر M3 (المحرك + الانكودر نفس القناة) ──
//   بنات درايفر M3: PWM=6, DIR=A11, EN=52. فاضية (M3 مو مستعمل للقيادة).
//   ملاحظة: EN=D52 يتعارض مع قناة QTR25 فقط لو استعملت المصفوفة الـ25 كاملة
//   (المكتبة تستعمل 14 قناة مو منهم 52 → آمن).
#define RT_M4_PWM    6
#define RT_M4_DIR   A11
#define RT_M4_EN    52
#define RT_M4_REVERSE  false   // اقلب لو الرافعة بتتحرك بالاتجاه المعكوس
#define RT_LIFT_SPEED   200    // سرعة liftUp/liftDown (0..255)
#define RT_LIFT_UP_SIGN  -1    // اتجاه الرفع (انقلب: كان liftUp بينزّل → صار -1)
// انكودر الرافعة (اختياري): تحكّم بالرفع بالمسافة بدل الوقت.
//   0 = بالوقت فقط.  1 = فعّل الانكودر.
//   نستعمل بنات إنكودر M3 (A7/A8) — فاضية (M3 مو مستعمل) وما تتعارض مع Serial.
//   ملاحظة: A7/A8 مو interrupt → العدّ بالـ polling (تمام لرافعة بطيئة).
#define RT_LIFT_USE_ENCODER  1
#define RT_M4_ENC_A   A7
#define RT_M4_ENC_B   A8

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
#define RT_COUNTS_PER_REV     2013     // الإنكودر اليسار صار 48CPR (كان 12): 493×4.08 — تقديري، عاير على 100cm

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
#define RT_STRAIGHT_MODE  1     // جايرو فقط — موازنة الإنكودر انقلبت بعد ما انبدلت الإنكودرات

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
#define RT_QTR_CENTER   7600      // النص الفعلي المقاس (الروبوت متمركز → pos≈7600؛ المصفوفة مزاحة)
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

// حساسات ميتة (99 = مافي). idx7 خربان (قِسناه = 0 دايماً بين حساسين 1000)
#define RT_QTR_DEAD_1      7
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

// تعارض QTR-RC مع مكتبة Servo (نفس مؤقّت Timer5): قراءة الحساسات كل لوب
// بتشوّش نبضة السيرفو فيرجف. الحل: افصل السيرفوهات أثناء تتبع الخط.
// بيرجعوا لحالهم أول ما تكتب لهم أمر بعدها (attach كسول). 0 = عطّل الفصل.
#define RT_LINE_PARK_SERVOS   1

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
// نبضة "الوقوف" لسيرفو 360° (دوران مستمر) — 1500µs قياسي.
// إذا السيرفو بيزحف بطيء وهو المفروض واقف → غيّرها شوي (1490 / 1510)
#define RT_SERVO_STOP_US 1500

// ── Color sensors (4× TCS34725، DIY bus-gate mux) ──
// كلهم عنوان 0x29؛ بوابة كل حساس على بن (HIGH = يوصل عالباص)
#define RT_COLOR_N          4
#define RT_COLOR1_PIN      13
#define RT_COLOR2_PIN      10
#define RT_COLOR3_PIN      11
#define RT_COLOR4_PIN       9
#define RT_COLOR_LED_PIN   12    // إضاءة مشتركة (HIGH = ON)
#define RT_COLOR_ADDR    0x29
#define RT_COLOR_ATIME   0xC0    // زمن التكامل (~154ms)
#define RT_COLOR_GAIN    0x01    // 4x
#define RT_COLOR_AVG        5    // عدد العيّنات للمعدّل (تنعيم)
#define RT_COLOR_BLACK_C   40    // C تحتها = اسود/معتم (سطوع)
#define RT_COLOR_EEPROM_ADDR 100 // مكان حفظ معايرة الألوان (بعيد عن معايرة الخط في 0..57)

// ── Sharp IR distance sensors (GP2Y0A41SK0F, 4-30cm, تناظري) ──
#define RT_DIST_N        2       // عدد الحساسات
#define RT_DIST1_PIN    A5
#define RT_DIST2_PIN    A6
#define RT_DIST_VREF   5.0f      // فولتية المرجع (ADC = 5V)
#define RT_DIST_A     12.08f     // تحويل: cm = A * v^B  (عايرهم لدقة أعلى)
#define RT_DIST_B    -1.058f
#define RT_DIST_MIN    4.0f      // أقل مسافة موثوقة (تحتها القراءة تنعكس!)
#define RT_DIST_MAX   30.0f      // أقصى مدى
#define RT_DIST_AVG      5       // عيّنات المعدّل (تنعيم)

// ── ACS712 current sensor (على A0، يقيس تيار الروبوت من 12V) ──
#define RT_CURRENT_PIN     A0
#define RT_CURRENT_VREF   5.0f    // فولتية المرجع (ADC)
// الحساسية حسب موديل ACS712: 5A=185، 20A=100، 30A=66 (mV لكل أمبير)
#define RT_CURRENT_MV_PER_A 100.0f
#define RT_CURRENT_ZERO_V   2.5f  // فولت عند 0 تيار (نص المرجع — يُعاير)
#define RT_CURRENT_AVG      10    // عيّنات المعدّل (تنعيم)

// ── Safety timeouts (ms) ───────────────────────────
#define RT_MOVE_TIMEOUT  20000
#define RT_TURN_TIMEOUT   5000

// ─────────────────────────────────────────────────────
//  ███  END CONFIG  ███
// ─────────────────────────────────────────────────────

// نتيجة تصنيف اللون — احفظها بمتغيّر وقرّر عليها
enum RTColor {
    RT_RED, RT_GREEN, RT_BLUE, RT_YELLOW, RT_WHITE, RT_BLACK, RT_UNKNOWN
};

// قراءة لون خام + نِسب chromaticity (لو بدك تفاصيل)
struct RTColorReading {
    uint16_t c, r, g, b;      // الخام
    float    nr, ng, nb;      // النِسب (مستقلة عن السطوع)
    RTColor  color;           // التصنيف
};

// مرجع لون متعلّم (chromaticity)
struct RTColorRef { float nr, ng, nb; bool set; };

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
    void  setTurnSpeed(int fast, int slowMax, int slowMin);  // سرعات اللف (سالب = لا تغيّر)
    void  setTurnTune(float kp, float fastDeg);     // قوة الهبوط + زاوية بداية التبطئة
    void  resetTurnTuning();                        // رجّع سرعات اللف للافتراضي
    void  printTurnTuning();                        // اطبع قيم اللف
    void  setPivotSpeed(int fast, int slow, int min); // سرعات الـ pivot (سالب = لا تغيّر)
    void  resetPivotTuning();                       // رجّع سرعات الـ pivot للافتراضي
    void  printPivotTuning();                       // اطبع قيم الـ pivot
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
    bool followLineUntilDistance(uint8_t sensor, float targetCm);      // اتبع الخط لحد ما المسافة = cm
    bool followLineToRange(uint8_t sensor, float minCm, float maxCm);  // اتبع الخط ووقّف ضمن رينج
    bool followLine2(float cm);                    // خوارزمية بديلة (أسلوب MegaShield) للتجربة
    // ── Line-follower tuning (عدّل لايف بدون recompile) ──
    void setLineSpeed(int base, int maxSpd, int minSpd);  // سرعات العجلة (سالب = لا تغيّر)
    void setLinePD(float kp, float kd);                   // PD التوجيه (سالب = لا تغيّر)
    void setLineTune(int deadband, float kpBoost, int slow); // deadband/boost/سرعة الكوع (سالب = لا تغيّر)
    void resetLineTuning();                               // رجّع كل قيم الخط للافتراضي
    void printLineTuning();                               // اطبع القيم الحالية

    void lineMonitor(uint32_t ms = 4000);          // اقرأ حيّ: قيم كل حساس + القمة + الموقع (لتعرف النص الحقيقي)
    void lineSteerCheck(uint32_t ms = 10000);      // فحص اتجاه التوجيه: وين الخط + وين رح يلف (بدون حركة)

    uint16_t linePosition();                       // 0..24000 (12000 = بالنص)
    void     lineReadRaw(uint16_t* dest25);        // قراءة خام (بدون calibration)
    uint16_t lineRead(uint16_t* dest25);           // قراءة calibrated + الموقع (65535 = مش معاير)

    // ── Motor 4 (رافعة/آلية) ───────────────────────
    void  motor4(int speed);                 // -255..255، 0 = وقوف (non-blocking)
    void  motor4For(int speed, uint32_t ms); // شغّله مدة (ms) ثم يوقف (blocking)
    void  motor4Stop();
    void  liftUp(uint32_t ms);               // ارفع الرافعة مدة (ms) — blocking (ينطر يخلص)
    void  liftDown(uint32_t ms);             // نزّل الرافعة مدة (ms) — blocking
    void  liftUpBy(long counts);             // ارفع عدد عدّات — blocking (يوقف لحاله، بدون update)
    void  liftDownBy(long counts);           // نزّل عدد عدّات — blocking
    // نسخ غير حابسة: تبدأ وترجع فوراً، وتوقف لحالها بعد ms (0 = تفضل شغّالة).
    // لازم تنادي update() باللوب (أو تعمل حركة تانية) حتى توقف بوقتها.
    void  liftUpAsync(uint32_t ms = 0);      // ارفع بدون انتظار
    void  liftDownAsync(uint32_t ms = 0);    // نزّل بدون انتظار
    void  liftStop();                        // وقّف الرافعة (يلغي أي مؤقّت async)
    bool  liftBusy();                        // هل الرافعة شغّالة (وقت/عدّات)؟
    // خيار الانكودر: لما ON، قيمة liftUpAsync/liftDownAsync = عدّات بدل ms
    void  setLiftUseEncoder(bool on);        // فعّل/عطّل التحكّم بالانكودر (لايف)
    long  liftEncoder();                     // قراءة انكودر الرافعة (0 لو معطّل)
    void  resetLiftEncoder();                // صفّر انكودر الرافعة

    // ── Servos (idx = 1..3 على البنات A2/A3/A4) ────
    void  servoWrite(uint8_t idx, int angle);                    // روح للزاوية فوراً (0..180)
    void  servoMove(uint8_t idx, int angle,
                    uint16_t degPerSec = RT_SERVO_SPEED);        // حركة ناعمة بسرعة معيّنة (blocking)
    void  servoDetach(uint8_t idx);                              // حرّر السيرفو (يوقف الطنين/العزم)
    void  servoAttachAll();                                      // فعّل السيرفوهات الثلاثة
    // ── سيرفو 360° (دوران مستمر) — السرعة مش الموقع ──
    void  servoSpin(uint8_t idx, int speed);                     // -100..100 (0 = وقوف)
    void  servoStop(uint8_t idx);                                // وقّف سيرفو الدوران (= spin 0)
    void  setServoStop(int us);                                  // عاير نبضة الوقوف (~1500) لحد ما يوقف تماماً
    int   servoAngle(uint8_t idx);                               // آخر زاوية مكتوبة (-1 = غير مفعّل)

    // ── Color sensors (4× TCS34725 — sensor = 1..4) ──
    RTColor readColor(uint8_t sensor);               // اقرأ لون حساس واحد → احفظه بمتغيّر
    void    readAllColors(RTColor dst[4]);           // اقرأ الأربعة → مصفوفة [4]
    bool    readColorRGB(uint8_t sensor, RTColorReading &out);  // خام + نِسب + تصنيف
    bool    colorPresent(uint8_t sensor);            // هل الحساس موجود؟ (1..4)
    void    teachColor(RTColor color, uint8_t sensor);  // علّم مرجع لون من حساس (معايرة حيّة)
    void    calibrateColors(uint8_t sensor = 1);     // ★ معايرة موجّهة: لون-لون + حفظ EEPROM
    bool    colorLoadCalibration();                  // حمّل مراجع الألوان من EEPROM (true=نجح)
    void    colorSaveCalibration();                  // احفظ المراجع بالـ EEPROM
    void    resetColorRefs();                        // رجّع مراجع الألوان للقيم الافتراضية
    void    printColorRefs();                        // اطبع المراجع
    void    setColorLED(bool on);                    // إضاءة الألوان on/off
    static const char* colorName(RTColor c);         // اسم اللون (للطباعة)

    // ── أسهل استعمال بالمهمة (بعد المعايرة) ──────────
    RTColor readColorStable(uint8_t sensor);         // اقرأ 3 مرات ورجّع الأغلب (أثبت للقرار)
    void    printColor(uint8_t sensor);              // اطبع لون الحساس (للتجربة)
    // تحقّق مباشر — اكتب: if (bot.isRed(1)) { ... }
    bool isColor(uint8_t sensor, RTColor c) { return readColor(sensor) == c; }
    bool isRed(uint8_t s)    { return isColor(s, RT_RED); }
    bool isGreen(uint8_t s)  { return isColor(s, RT_GREEN); }
    bool isBlue(uint8_t s)   { return isColor(s, RT_BLUE); }
    bool isYellow(uint8_t s) { return isColor(s, RT_YELLOW); }
    bool isWhite(uint8_t s)  { return isColor(s, RT_WHITE); }
    bool isBlack(uint8_t s)  { return isColor(s, RT_BLACK); }

    // ── Sharp distance sensors (IR, sensor = 1..2) ──
    // ── Current sensor (ACS712 على A0) ──────────────
    float readCurrent();                         // تيار الروبوت بالأمبير (A)
    int   readCurrentRaw();                      // قراءة ADC خام 0..1023
    void  calibrateCurrentZero();                // عاير نقطة الصفر (المحركات واقفة!)

    float readDistance(uint8_t sensor);          // مسافة بالسم (4..30؛ معدّل + منحنى)
    int   readDistanceRaw(uint8_t sensor);       // قراءة ADC خام 0..1023 (للمعايرة)
    bool  distanceWithin(uint8_t sensor, float cm); // true لو الجسم أقرب من cm
    // امشِ مستقيم (بالجايرو) لحد ما يصير الجسم قدّامك على targetCm ثم وقّف.
    // يبطّئ قبل الهدف (ضد الاندفاع). يرجّع true لو وصل. cruise = سرعة المشي.
    bool  forwardUntilDistance(uint8_t sensor, float targetCm, int cruise = 120);
    // امشِ ووقّف بحيث تصير المسافة ضمن [minCm, maxCm] (دقة أعلى — يتراجع لو تجاوز).
    bool  forwardToRange(uint8_t sensor, float minCm, float maxCm, int cruise = 120);
    // خطوة مشي مستقيم واحدة (non-blocking، بالجايرو) — ناديها بلوب مع شرطك الخاص.
    // speed موجب=قدّام، سالب=خلف. اعمل resetHeading() قبل اللوب.
    void  goStraight(int speed);

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

    // line-follower tuning قابلة للتعديل لايف (تبدأ من قيم الـ CONFIG)
    int   _lineBase, _lineMax, _lineMin, _lineSlow, _lineDeadband;
    float _lineKp, _lineKd, _lineKpBoost;

    // turn tuning قابلة للتعديل لايف (تبدأ من قيم الـ CONFIG)
    int   _turnFast, _turnSlowMax, _turnSlowMin;   // = RT_TURN_FAST / SLOW_MAX / SLOW_MIN
    float _turnKp, _turnFastDeg;                   // = RT_TURN_KP / FAST_DEG

    // pivot tuning قابلة للتعديل لايف
    int   _pivotFast, _pivotSlow, _pivotMin;       // = RT_PIVOT_FAST / SLOW / MIN

    // line follower
    QTRSensors _qtr;
    uint16_t   _qtrVals[RT_QTR_N];
    bool       _qtrReady = false;     // صار calibration؟
    bool       _qtrPinsInit = false;  // صارت تهيئة البنات؟
    void _lineBegin();                // تهيئة بنات الـ QTR (مرة واحدة)
    bool _followLine(uint8_t nJunctions, float cm,
                     uint8_t distSensor = 0, float distTarget = 0);  // المحرك المشترك

    // servos
    Servo    _servo[RT_SERVO_N];
    uint8_t  _servoPin[RT_SERVO_N];
    int16_t  _servoPos[RT_SERVO_N];   // آخر زاوية (-1 = ما تحرّك بعد)
    bool     _servoAttached[RT_SERVO_N];
    int      _servoStopUs;            // نبضة الوقوف لسيرفو 360° (تبدأ من RT_SERVO_STOP_US)
    uint32_t _m4StopAt = 0;           // وقت إيقاف الرافعة async (وضع الوقت)
    uint8_t  _m4Mode = 0;             // 0=واقف/دايم  1=بالوقت  2=بالانكودر
    long     _m4TargetCount = 0;      // هدف العدّات (وضع الانكودر)
    bool     _liftUseEnc = RT_LIFT_USE_ENCODER;   // خيار: انكودر أو وقت (لايف)
    float    _currentZeroV = RT_CURRENT_ZERO_V;   // فولت ACS712 عند 0 تيار (يُعاير)
#if RT_LIFT_USE_ENCODER
    Encoder  _liftEnc{RT_M4_ENC_A, RT_M4_ENC_B};  // انكودر الرافعة (مفعّل بالكومبايل)
#endif
    void     _serviceMotor4();        // يوقف الرافعة (وقت/عدّات) — يُنادى بالخلفية
    void     _liftStartAsync(int speed, uint32_t value, const __FlashStringHelper* tag);

    // color sensors (DIY bus-gate mux)
    uint8_t     _colorPin[RT_COLOR_N];
    bool        _colorOk[RT_COLOR_N];
    RTColorRef  _colorRef[7];          // مراجع الألوان (حسب enum RTColor)
    void    _colorBegin();
    void    _colorSelect(int slot);    // شغّل بوابة سلوت واحد
    void    _colorDeselectAll();
    bool    _colorAck();
    bool    _tcsW(uint8_t reg, uint8_t val);
    uint8_t _tcsR(uint8_t reg);
    void    _colorConfig();
    bool    _colorReadAvg(int slot, RTColorReading &out);
    RTColor _colorClassify(const RTColorReading &rd);
    char    _waitSerialChar();        // ينطر سطر من Serial، يرجّع أول حرف (للمعايرة الموجّهة)
    void     _servoBegin();           // سجّل البنات (بدون attach — كسول)
    bool     _servoEnsure(uint8_t i); // فعّل السيرفو i لو مش مفعّل؛ true=جاهز
    void     _lineParkServos();       // افصل كل السيرفوهات قبل تتبع الخط (ضد تعارض Timer5)

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
