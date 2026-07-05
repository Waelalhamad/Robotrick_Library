// =====================================================
//  Robotrick v2 — TEST BENCH
//  اكتب أمر في Serial Monitor (115200) واضغط Enter.
//  بعد كل حركة بيطبع النتيجة (encoders + heading) عشان
//  نحلّلها مع بعض.
//
//  الأوامر:
//    g            معايرة الجايرو (ثبّت الروبوت)
//    c            معايرة السرعة (يحتاج ~1 متر فاضي قدام!)
//    f <cm>       لقدام     مثال:  f 50
//    b <cm>       لخلف      مثال:  b 30
//    l <deg>      لف يسار   مثال:  l 90
//    r <deg>      لف يمين   مثال:  r 90
//    e            اقرأ الإنكودرات (L, R, النسبة R/L)
//    h            اقرأ الزاوية (heading)
//    q            مربّع: (لقدام 30 + لف يسار 90) ×4
//    x            وقوف
//    ?            اعرض القائمة
//
//  IMPORTANT: بالـ Serial Monitor خلي "Line ending" = Newline
// =====================================================
#include <Robotrick.h>

Robotrick bot;

char    buf[24];
uint8_t idx = 0;

void printMenu() {
    Serial.println(F("\n===== Robotrick v2 Test Bench ====="));
    Serial.println(F("  g        calibrate gyro"));
    Serial.println(F("  c        calibrate speed (~1m clear!)"));
    Serial.println(F("  f <cm>   forward     (f 50)"));
    Serial.println(F("  b <cm>   backward    (b 30)"));
    Serial.println(F("  l <deg>  turn left   (l 90)"));
    Serial.println(F("  r <deg>  turn right  (r 90)"));
    Serial.println(F("  p <deg>  PIVOT left wheel  (+قدام/-خلف)  (p 90)"));
    Serial.println(F("  o <deg>  PIVOT right wheel (+قدام/-خلف)  (o -90)"));
    Serial.println(F("  e        read encoders (L, R, ratio)"));
    Serial.println(F("  h        heading"));
    Serial.println(F("  y        GYRO TEST (rotate by hand, watch heading)"));
    Serial.println(F("  k        LINE calibrate مرة وحدة (يحفظ EEPROM للأبد)"));
    Serial.println(F("  j <n>    follow line to junction #n  (j 1)"));
    Serial.println(F("  m <cm>   follow line for cm          (m 40)"));
    Serial.println(F("  n <cm>   LINE2 خوارزمية بديلة         (n 40)"));
    Serial.println(F("  q        square: (fwd30 + left90) x4"));
    Serial.println(F("  4 <spd>  motor4 (رافعة) بسرعة spd (-255..255، 0=وقوف)"));
    Serial.println(F("  u <ms>   liftUp مدة ms       i <ms>  liftDown"));
    Serial.println(F("  s <i> <a>  servo فوري (s 1 90)   v <i> <a>  servo ناعم"));
    Serial.println(F("  d <i>    فصل السيرفو (d 1)"));
    Serial.println(F("  w <cm/s> سرعة forward (w 25)"));
    Serial.println(F("  t <kp> <kd> <ki>  PID المسافة (t 9 2 0)   T  اطبع القيم"));
    Serial.println(F("  x        stop        ?  menu"));
    Serial.println(F("==================================="));
    Serial.print(F("> "));
}

// يطبع حالة الإنكودرات + النسبة (نشوف الـ 4:1)
void printEnc() {
    long L = bot.encoderLeft();
    long R = bot.encoderRight();
    Serial.print(F("  encL=")); Serial.print(L);
    Serial.print(F("  encR=")); Serial.print(R);
    if (L != 0) { Serial.print(F("  R/L=")); Serial.print((float)R / (float)L, 2); }
    Serial.println();
}

// بعد أي حركة: اطبع النتيجة
void reportMove() {
    printEnc();
    Serial.print(F("  heading=")); Serial.print(bot.heading(), 1); Serial.println(F("°"));
    Serial.println(F("  >> قِس المسافة/الزاوية الفعلية بالمسطرة وقلّي القيمة"));
}

void handle(char cmd, float val, float val2, float val3) {
    if (cmd == 'g') {
        bot.calibrateGyro();
    }
    else if (cmd == 'c') {
        bot.calibrateSpeed();     // بيطبع RT_PWM_STATIC و RT_KV
    }
    else if (cmd == 'f') {
        Serial.print(F(">> forward ")); Serial.println(val);
        bot.forward(val);
        reportMove();
    }
    else if (cmd == 'b') {
        Serial.print(F(">> backward ")); Serial.println(val);
        bot.backward(val);
        reportMove();
    }
    else if (cmd == 'l') {
        Serial.print(F(">> turn LEFT ")); Serial.println(val);
        bot.turnLeft(val);
        Serial.print(F("  heading=")); Serial.print(bot.heading(), 1); Serial.println(F("°"));
    }
    else if (cmd == 'r') {
        Serial.print(F(">> turn RIGHT ")); Serial.println(val);
        bot.turnRight(val);
        Serial.print(F("  heading=")); Serial.print(bot.heading(), 1); Serial.println(F("°"));
    }
    else if (cmd == 'p') {   // pivot على العجلة اليسار (+قدام / -خلف)
        Serial.print(F(">> PIVOT L ")); Serial.println(val);
        bot.pivot('L', val);
    }
    else if (cmd == 'o') {   // pivot على العجلة اليمين (+قدام / -خلف)
        Serial.print(F(">> PIVOT R ")); Serial.println(val);
        bot.pivot('R', val);
    }
    else if (cmd == 'e') {
        printEnc();
    }
    else if (cmd == 'h') {
        Serial.print(F("  heading=")); Serial.print(bot.heading(), 1); Serial.println(F("°"));
    }
    else if (cmd == 'y') {
        // اختبار الجايرو: لِف الروبوت بإيدك وراقب الزاوية
        Serial.println(F(">> GYRO TEST — لِف الروبوت بإيدك 6 ثواني:"));
        Serial.println(F("   يسار لازم يعطي إشارة، يمين الإشارة المعاكسة."));
        Serial.println(F("   إذا ما تغيّرت الزاوية أبداً = الجايرو مفصول/خربان."));
        bot.resetHeading();
        uint32_t end = millis() + 6000;
        while (millis() < end) {
            bot.update();
            Serial.print(F("  heading=")); Serial.println(bot.heading(), 1);
            delay(200);
        }
        Serial.println(F("  خلص الاختبار."));
    }
    else if (cmd == 'k') {
        bot.lineCalibrate(5);
    }
    else if (cmd == 'j') {
        int n = (val >= 1) ? (int)val : 1;
        Serial.print(F(">> follow line to junction #")); Serial.println(n);
        bool ok = bot.followLineToJunction(n);
        Serial.println(ok ? F("  SUCCESS") : F("  FAILED (lost/timeout)"));
    }
    else if (cmd == 'm') {
        Serial.print(F(">> follow line for ")); Serial.print(val); Serial.println(F("cm"));
        bool ok = bot.followLineForCM(val);
        Serial.println(ok ? F("  SUCCESS") : F("  FAILED (lost/timeout)"));
    }
    else if (cmd == 'n') {
        Serial.print(F(">> LINE2 (خوارزمية بديلة) for ")); Serial.print(val); Serial.println(F("cm"));
        bool ok = bot.followLine2(val);
        Serial.println(ok ? F("  SUCCESS") : F("  FAILED (lost/timeout)"));
    }
    else if (cmd == 'q') {
        Serial.println(F(">> square test"));
        for (int i = 0; i < 4; i++) { bot.forward(30); bot.turnLeft(90); }
        bot.stop();
        Serial.println(F("  square done"));
    }
    else if (cmd == '4') {
        Serial.print(F(">> motor4 speed=")); Serial.println((int)val);
        bot.motor4((int)val);   // 0 = وقوف
    }
    else if (cmd == 'u') {
        Serial.print(F(">> lift UP ")); Serial.print((uint32_t)val); Serial.println(F("ms"));
        bot.liftUp((uint32_t)val);
    }
    else if (cmd == 'i') {
        Serial.print(F(">> lift DOWN ")); Serial.print((uint32_t)val); Serial.println(F("ms"));
        bot.liftDown((uint32_t)val);
    }
    else if (cmd == 's') {   // servo فوري:  s <idx> <angle>
        Serial.print(F(">> servo ")); Serial.print((int)val);
        Serial.print(F(" -> ")); Serial.println((int)val2);
        bot.servoWrite((uint8_t)val, (int)val2);
    }
    else if (cmd == 'v') {   // servo ناعم:  v <idx> <angle>  (بالسرعة الافتراضية)
        Serial.print(F(">> servo (smooth) ")); Serial.print((int)val);
        Serial.print(F(" ~> ")); Serial.println((int)val2);
        bot.servoMove((uint8_t)val, (int)val2);
    }
    else if (cmd == 'd') {   // فصل السيرفو:  d <idx>
        Serial.print(F(">> servo detach ")); Serial.println((int)val);
        bot.servoDetach((uint8_t)val);
    }
    else if (cmd == 'w') {   // سرعة forward:  w <cm/s>
        bot.setDriveSpeed(val);
    }
    else if (cmd == 't') {   // PID المسافة:  t <kp> <kd> <ki>  (سالب = لا تغيّرها؛ ki اختياري)
        bot.setDrivePID(val, val2, val3);
    }
    else if (cmd == 'T') {   // اطبع قيم الـ tuning الحالية
        bot.printDriveTuning();
    }
    else if (cmd == 'x') {
        bot.stop();
        bot.motor4Stop();
        Serial.println(F("  STOPPED"));
    }
    else if (cmd == '?') {
        printMenu();
        return;
    }
    else {
        Serial.println(F("  ؟ أمر غير معروف — اضغط ? للقائمة"));
    }
    Serial.print(F("> "));
}

void setup() {
    bot.begin();          // init + معايرة الجايرو
    delay(300);
    // حاول تحمّل معايرة الخط المحفوظة (بدون تحريك)
    if (bot.lineLoadCalibration())
        Serial.println(F("[line] ✓ معايرة الخط محمّلة من EEPROM — ما بدك تعمل 'k'"));
    else
        Serial.println(F("[line] ما في معايرة محفوظة — اعمل 'k' مرة وحدة بس"));
    printMenu();
}

void loop() {
    bot.update();         // خلي الـ heading حي وأنت واقف

    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\r') continue;
        if (ch == '\n') {
            buf[idx] = '\0';
            if (idx > 0) {
                char cmd = buf[0];
                float v[3] = {0, 0, 0};
                // حتى 3 أرقام بعد الأمر — مثال: s 1 90 / t 9 2 0
                uint8_t i = 1, n = 0;
                #define IS_NUM(c) (((c) >= '0' && (c) <= '9') || (c) == '-' || (c) == '.')
                while (i < idx && n < 3) {
                    while (i < idx && !IS_NUM(buf[i])) i++;   // تجاوز الفراغ
                    if (i >= idx) break;
                    v[n++] = atof(&buf[i]);
                    while (i < idx && IS_NUM(buf[i])) i++;    // تجاوز الرقم
                }
                #undef IS_NUM
                Serial.println(buf);   // echo
                handle(cmd, v[0], v[1], v[2]);
            }
            idx = 0;
        }
        else if (idx < sizeof(buf) - 1) {
            buf[idx++] = ch;
        }
    }
}
