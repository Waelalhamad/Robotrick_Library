// =====================================================
//  ColorScenarioExample — بدّل السيناريو حسب كل قراءة لون
//
//  الفكرة: كل لون = "سيناريو" كامل (تسلسل حركات). الروبوت يقرأ،
//  ويشغّل السيناريو المناسب. وبمهمة متعددة المحطات، كل قراءة
//  بتغيّر شو رح يعمل بالمحطة اللي بعدها.
//
//  الألوان: RT_RED RT_GREEN RT_BLUE RT_YELLOW RT_WHITE RT_BLACK RT_UNKNOWN
// =====================================================
#include <Robotrick.h>

Robotrick bot;

// ─────────────────────────────────────────────────────
//  السيناريوهات — كل واحد تسلسل حركات مستقل
//  (غيّر محتواها حسب مهمتك؛ هدول أمثلة)
// ─────────────────────────────────────────────────────
void scenarioRed() {
    Serial.println(F(">> سيناريو أحمر: خذ لليسار وارفع"));
    bot.turnLeft(90);
    bot.forward(20);
    bot.liftUp(1000);
}

void scenarioGreen() {
    Serial.println(F(">> سيناريو أخضر: خذ لليمين وأنزل"));
    bot.turnRight(90);
    bot.forward(20);
    bot.liftDown(1000);
}

void scenarioBlue() {
    Serial.println(F(">> سيناريو أزرق: كمّل مستقيم"));
    bot.forward(40);
}

void scenarioYellow() {
    Serial.println(F(">> سيناريو أصفر: التقط بالسيرفو"));
    bot.servoWrite(1, 30);
    delay(400);
    bot.servoWrite(1, 150);
}

void scenarioDefault(RTColor c) {
    Serial.print(F(">> لون غير معرّف (")); Serial.print(bot.colorName(c));
    Serial.println(F(") — تجاهل/وقوف"));
    bot.stop();
}

// ─────────────────────────────────────────────────────
//  المُرسِل (dispatcher): قراءة وحدة → سيناريو واحد
// ─────────────────────────────────────────────────────
void runScenarioFor(RTColor c) {
    switch (c) {
        case RT_RED:    scenarioRed();    break;
        case RT_GREEN:  scenarioGreen();  break;
        case RT_BLUE:   scenarioBlue();   break;
        case RT_YELLOW: scenarioYellow(); break;
        default:        scenarioDefault(c); break;   // WHITE/BLACK/UNKNOWN
    }
}

// ─────────────────────────────────────────────────────
//  مهمة متعددة المحطات: بكل محطة اقرأ → بدّل السيناريو
//  → كل قراءة تحدّد سلوك المحطة
// ─────────────────────────────────────────────────────
void runMission(uint8_t stations) {
    for (uint8_t s = 1; s <= stations; s++) {
        Serial.print(F("\n== محطة ")); Serial.println(s);
        bot.followLineToJunction(1);   // امشِ للمحطة التالية
        bot.stop();
        delay(200);                    // ثبّت قبل القراءة (أدق)

        RTColor c = bot.readColor(1);  // اقرأ لون هالمحطة
        Serial.print(F("   قراءة = ")); Serial.println(bot.colorName(c));
        runScenarioFor(c);             // بدّل السيناريو حسبها
    }
    Serial.println(F("\n== خلصت المهمة"));
}

// ─────────────────────────────────────────────────────
//  قرار مؤجّل: احفظ القراءة بمتغيّر واستعملها لاحقاً
//  مثال: اقرأ الهدف الآن، ونفّذ الإسقاط بمكان تاني
// ─────────────────────────────────────────────────────
RTColor targetColor = RT_UNKNOWN;      // متغيّر القرار (يُحفظ)

void readTarget() {
    targetColor = bot.readColor(1);    // خزّن القرار
    Serial.print(F("الهدف المحفوظ = ")); Serial.println(bot.colorName(targetColor));
}

void deliverToTarget() {
    // استعمل المتغيّر المحفوظ لتقرّر لوين تروح
    switch (targetColor) {
        case RT_RED:    bot.turnLeft(90);  break;   // منطقة الأحمر يسار
        case RT_GREEN:  bot.turnRight(90); break;   // منطقة الأخضر يمين
        case RT_BLUE:   bot.forward(30);   break;   // الأزرق قدّام
        default:        bot.stop();        break;
    }
    bot.liftDown(800);                  // أنزل الحمولة
}

void setup() {
    bot.begin();
    Serial.println(F("=== Color Scenario Example ==="));
    bot.printColorRefs();
    Serial.println(F("اكتب: c=اقرأ ونفّذ  m=مهمة 3 محطات  t=احفظ هدف  g=سلّم للهدف"));
}

void loop() {
    bot.update();
    if (Serial.available()) {
        char ch = Serial.read();
        if      (ch == 'c') runScenarioFor(bot.readColor(1)); // قراءة واحدة → سيناريو
        else if (ch == 'm') runMission(3);                    // مهمة متعددة المحطات
        else if (ch == 't') readTarget();                     // احفظ قرار
        else if (ch == 'g') deliverToTarget();                // استعمل القرار المحفوظ
    }
}
