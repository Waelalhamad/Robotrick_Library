// =====================================================
//  MotorEncoderTest — تشخيص المحركات والإنكودرات
//  (مستقل تماماً — ما بيستعمل مكتبة Robotrick، تحكّم مباشر بالبنات)
//
//  الهدف: نعرف بعد ما فكّيت الروبوت:
//    • أي قناة درايفر (A=يسار / B=يمين) بتحرّك أي عجلة فيزيائياً
//    • بأي اتجاه تلف (قدّام/خلف)
//    • أي إنكودر (encA/encB) بيستجيب لكل محرك، وكم عدّة
//    • الفرق بالعدّات (نسبة 4:1؟ أي جهة الـ12CPR وأي جهة 48CPR)
//
//  الاستعمال: افتح Serial Monitor 115200، Line ending = Newline.
//    اكتب أمر واحد واضغط Enter، وراقب العجلات + القراءات.
//
//  الأوامر:
//    1   شغّل محرك A (قناة يسار) قدّام 1.5s   ← راقب: أي عجلة؟ أي اتجاه؟
//    2   شغّل محرك A (قناة يسار) خلف   1.5s
//    3   شغّل محرك B (قناة يمين) قدّام 1.5s
//    4   شغّل محرك B (قناة يمين) خلف   1.5s
//    5   شغّل الاثنين قدّام 1.5s              ← لازم يمشي مستقيم لو كله مزبوط
//    e   اقرأ الإنكودرين مباشرة (دحرِج العجلات بإيدك وراقب)
//    r   صفّر الإنكودرين
//    x   وقوف
//    ?   القائمة
// =====================================================
#include <Encoder.h>

// ── بنات المحركات (نفس Robotrick Mega Shield) ──────
// قناة A = "اليسار" بالمكتبة
#define A_PWM    4
#define A_DIR   25
#define A_EN    24
// قناة B = "اليمين" بالمكتبة
#define B_PWM    7
#define B_DIR   A12
#define B_EN    A15

// ── بنات الإنكودرات ───────────────────────────────
#define ENC_A_1  19
#define ENC_A_2  18
#define ENC_B_1   2
#define ENC_B_2   3

#define TEST_PWM   120     // سرعة الاختبار (0..255)
#define TEST_MS   1500     // مدة كل نبضة

Encoder encA(ENC_A_1, ENC_A_2);   // إنكودر "اليسار"
Encoder encB(ENC_B_1, ENC_B_2);   // إنكودر "اليمين"

char    buf[16];
uint8_t idx = 0;

void driverA(int dirFwd, int pwm) {   // dirFwd: 1=قدّام(HIGH) 0=خلف(LOW)
    digitalWrite(A_EN, HIGH);
    digitalWrite(A_DIR, dirFwd ? HIGH : LOW);
    analogWrite(A_PWM, pwm);
}
void driverB(int dirFwd, int pwm) {
    digitalWrite(B_EN, HIGH);
    digitalWrite(B_DIR, dirFwd ? HIGH : LOW);
    analogWrite(B_PWM, pwm);
}
void stopAll() {
    analogWrite(A_PWM, 0);
    analogWrite(B_PWM, 0);
}

void printMenu() {
    Serial.println(F("\n===== MotorEncoderTest ====="));
    Serial.println(F("  1  محرك A (يسار) قدّام     2  محرك A خلف"));
    Serial.println(F("  3  محرك B (يمين) قدّام     4  محرك B خلف"));
    Serial.println(F("  5  الاثنين قدّام (تست استقامة)"));
    Serial.println(F("  e  اقرأ الإنكودرين   r  صفّر   x  وقوف   ?  قائمة"));
    Serial.println(F("  >> راقب: أي عجلة تحرّكت؟ أي اتجاه؟ أي إنكودر عدّ؟"));
    Serial.print(F("> "));
}

// شغّل محرك محدّد ويطبع تغيّر الإنكودرين
void pulse(char which, int dirFwd) {
    encA.write(0);
    encB.write(0);
    long a0 = 0, b0 = 0;

    Serial.print(F("\n>> MOTOR ")); Serial.print(which);
    Serial.print(which <= 'A' || which == 'A' ? F(" (LEFT-channel PWM4)") : F(""));
    if (which == 'B') Serial.print(F(" (RIGHT-channel PWM7)"));
    Serial.print(F("  DIR=")); Serial.println(dirFwd ? F("FWD(HIGH)") : F("BACK(LOW)"));
    Serial.println(F("   ↓ راقب العجلة الآن"));

    uint32_t endAt = millis() + TEST_MS;
    if (which == 'A') driverA(dirFwd, TEST_PWM);
    else              driverB(dirFwd, TEST_PWM);

    while (millis() < endAt) {
        Serial.print(F("   encA=")); Serial.print(encA.read());
        Serial.print(F("  encB=")); Serial.println(encB.read());
        delay(200);
    }
    stopAll();
    long a = encA.read(), b = encB.read();
    Serial.print(F("   >> النتيجة: encA=")); Serial.print(a);
    Serial.print(F("  encB=")); Serial.println(b);
    // أي إنكودر استجاب؟
    if (abs(a) > abs(b) * 3 && abs(a) > 20)
        Serial.println(F("   → هالمحرك مربوط مع encA (الأكثر عدّاً)"));
    else if (abs(b) > abs(a) * 3 && abs(b) > 20)
        Serial.println(F("   → هالمحرك مربوط مع encB (الأكثر عدّاً)"));
    else if (abs(a) < 20 && abs(b) < 20)
        Serial.println(F("   !! ولا إنكودر عدّ — المحرك ما لف أو الإنكودر مفصول"));
    else
        Serial.println(F("   ?? الاثنين عدّوا — تحقّق من التوصيل"));
    Serial.print(F("> "));
}

void handle(char c) {
    if      (c == '1') pulse('A', 1);
    else if (c == '2') pulse('A', 0);
    else if (c == '3') pulse('B', 1);
    else if (c == '4') pulse('B', 0);
    else if (c == '5') {
        encA.write(0); encB.write(0);
        Serial.println(F("\n>> الاثنين قدّام — لازم يمشي مستقيم لو كله مزبوط"));
        driverA(1, TEST_PWM); driverB(1, TEST_PWM);
        uint32_t endAt = millis() + TEST_MS;
        while (millis() < endAt) {
            Serial.print(F("   encA=")); Serial.print(encA.read());
            Serial.print(F("  encB=")); Serial.println(encB.read());
            delay(200);
        }
        stopAll();
        Serial.print(F("   >> encA=")); Serial.print(encA.read());
        Serial.print(F("  encB=")); Serial.println(encB.read());
        Serial.println(F("   (لاحظ الفرق بالعدّات = نسبة CPR بين الجهتين)"));
        Serial.print(F("> "));
    }
    else if (c == 'e') {
        Serial.print(F("   encA=")); Serial.print(encA.read());
        Serial.print(F("  encB=")); Serial.println(encB.read());
        Serial.print(F("> "));
    }
    else if (c == 'r') {
        encA.write(0); encB.write(0);
        Serial.println(F("   صُفّرت الإنكودرات"));
        Serial.print(F("> "));
    }
    else if (c == 'x') { stopAll(); Serial.println(F("   STOP")); Serial.print(F("> ")); }
    else if (c == '?') { printMenu(); }
    else { Serial.println(F("   ؟ أمر غير معروف — ? للقائمة")); Serial.print(F("> ")); }
}

void setup() {
    Serial.begin(115200);
    pinMode(A_PWM, OUTPUT); pinMode(A_DIR, OUTPUT); pinMode(A_EN, OUTPUT);
    pinMode(B_PWM, OUTPUT); pinMode(B_DIR, OUTPUT); pinMode(B_EN, OUTPUT);
    digitalWrite(A_EN, HIGH); digitalWrite(B_EN, HIGH);
    stopAll();
    delay(300);
    Serial.println(F("\n[MotorEncoderTest] جاهز."));
    printMenu();
}

void loop() {
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\r') continue;
        if (ch == '\n') {
            if (idx > 0) { buf[idx] = '\0'; handle(buf[0]); idx = 0; }
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = ch;
        }
    }
}
