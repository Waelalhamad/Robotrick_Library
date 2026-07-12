// =====================================================
//  MissionColorExample — كيف تستعمل حساسات الألوان بمهمة
//  يقرأ الألوان ويتّخذ قرار (حركة/رفع/سيرفو) حسب اللون أو الترتيب.
//
//  الألوان (enum RTColor):
//    RT_RED  RT_GREEN  RT_BLUE  RT_YELLOW  RT_WHITE  RT_BLACK  RT_UNKNOWN
//
//  فكرة عامة: توقف عند مكان، تقرأ اللون، تقرّر شو تعمل.
// =====================================================
#include <Robotrick.h>

Robotrick bot;

// ─────────────────────────────────────────────────────
//  1) قرار بحساس واحد — الأكثر شيوعاً
//     مثال: لون تحت الروبوت يقرّر وين يلف
// ─────────────────────────────────────────────────────
void decideBySingle() {
    RTColor c = bot.readColor(1);              // احفظه بمتغيّر
    Serial.print(F("لون S1 = ")); Serial.println(bot.colorName(c));

    switch (c) {
        case RT_RED:    bot.turnLeft(90);       break;   // أحمر → يسار
        case RT_GREEN:  bot.turnRight(90);      break;   // أخضر → يمين
        case RT_BLUE:   bot.forward(30);        break;   // أزرق → قدّام
        case RT_YELLOW: bot.liftUp(1000);       break;   // أصفر → ارفع
        case RT_BLACK:  bot.stop();             break;   // أسود → وقوف
        default:        /* RT_UNKNOWN */        break;   // مش متأكد → لا تعمل شي
    }
}

// ─────────────────────────────────────────────────────
//  2) دالة "افعل حسب اللون" — تعيد استعمالها لأي حساس
// ─────────────────────────────────────────────────────
void actByColor(RTColor c) {
    switch (c) {
        case RT_RED:    bot.servoWrite(1, 30);  break;   // مثال: افتح كفّ
        case RT_GREEN:  bot.servoWrite(1, 150); break;
        case RT_BLUE:   bot.liftUp(800);        break;
        case RT_YELLOW: bot.liftDown(800);      break;
        default:                                break;
    }
}

// ─────────────────────────────────────────────────────
//  3) قرار بالأربعة سوا — عالج كل حساس بالدور
//     مثال: 4 أغراض ملوّنة بصفّ، كل واحد له إجراء
// ─────────────────────────────────────────────────────
void decideByAll() {
    RTColor cols[4];
    bot.readAllColors(cols);                   // املأ المصفوفة

    for (uint8_t i = 0; i < 4; i++) {
        Serial.print(F("S")); Serial.print(i + 1);
        Serial.print(F(" = ")); Serial.println(bot.colorName(cols[i]));
        actByColor(cols[i]);                   // نفّذ إجراء كل لون
    }
}

// ─────────────────────────────────────────────────────
//  4) مطابقة ترتيب محدّد — "إذا الترتيب كذا، اعمل تسلسل كذا"
// ─────────────────────────────────────────────────────
void decideByArrangement() {
    RTColor c[4];
    bot.readAllColors(c);

    // مثال ترتيب A: احمر-اخضر-ازرق-اصفر
    if (c[0] == RT_RED && c[1] == RT_GREEN && c[2] == RT_BLUE && c[3] == RT_YELLOW) {
        Serial.println(F(">> ترتيب A"));
        bot.forward(20); bot.turnLeft(90); bot.liftUp(1000);
    }
    // مثال ترتيب B: كله أزرق
    else if (c[0] == RT_BLUE && c[1] == RT_BLUE && c[2] == RT_BLUE && c[3] == RT_BLUE) {
        Serial.println(F(">> ترتيب B (كله أزرق)"));
        bot.forward(40);
    }
    // عدّ كم حساس شايف أحمر (قرار حسب العدد)
    else {
        uint8_t reds = 0;
        for (uint8_t i = 0; i < 4; i++) if (c[i] == RT_RED) reds++;
        Serial.print(F(">> عدد الأحمر = ")); Serial.println(reds);
        if (reds >= 2) bot.turnRight(90);
        else           bot.forward(25);
    }
}

// ─────────────────────────────────────────────────────
//  مهمة كاملة مصغّرة: اتبع الخط → اقرأ → قرّر
// ─────────────────────────────────────────────────────
void runMission() {
    bot.followLineToJunction(1);   // امشِ لحد أول تقاطع
    bot.stop();
    delay(200);                    // ثبّت قبل القراءة
    decideBySingle();              // قرّر حسب اللون
    // ... كمّل باقي المهمة
}

void setup() {
    bot.begin();                   // فيه _colorBegin — يهيّئ الألوان
    Serial.println(F("=== Mission Color Example ==="));
    bot.printColorRefs();          // اعرض المراجع المتعلّمة
    Serial.println(F("اكتب: 1=single 2=all 3=arrange m=mission"));
}

void loop() {
    bot.update();                  // خلي الخلفية حيّة (heading/lift)
    if (Serial.available()) {
        char ch = Serial.read();
        if      (ch == '1') decideBySingle();
        else if (ch == '2') decideByAll();
        else if (ch == '3') decideByArrangement();
        else if (ch == 'm') runMission();
    }
}
