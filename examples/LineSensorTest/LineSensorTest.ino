// =====================================================
//  LineSensorTest — فحص كل قنوات QTR-25 (D28..D53)
//  (مستقل تماماً — ما بيستعمل مكتبة Robotrick)
//
//  الهدف: نعرف أي بنات (قنوات) شغّالة وأي ميتة، عشان
//  ننقل نافذة الـ14 حساس لقنوات شغّالة (المصفوفة 25 قناة).
//
//  الاستعمال: Serial Monitor 115200، Line ending = Newline.
//    l   بثّ خام ~10 ثواني — حرّك الخط على طول المصفوفة ببطء،
//        وبالآخر بيطبع لكل بن: ALIVE (تغيّر) أو DEAD (عالق 2500)
//    r   اقرأ خام مرة وحدة (كل البنات)
//    ?   القائمة
//
//  RAW: ~0-600 أبيض/انعكاس، ~2500 = timeout (خط أسود أو قناة ميتة)
//  ملاحظة: قناة تظل 2500 حتى لما تمرّر الخط على غيرها = ميتة/مفصولة.
// =====================================================
#include <QTRSensors.h>

// كل بنات داتا QTR-25 على الميجا بالترتيب الرقمي (D28..D53 = 26 بن)
#define N 26
const uint8_t PINS[N] = {
    28,29,30,31,32,33,34,35,36,37,38,39,40,
    41,42,43,44,45,46,47,48,49,50,51,52,53
};
#define EMIT_ODD  27
#define EMIT_EVEN 26
#define TIMEOUT_US 2500
#define STUCK_TH  2400     // فوقها بنعتبرها "غامق/عالق"

QTRSensors qtr;
uint16_t v[N];

char    buf[16];
uint8_t idx = 0;

void printMenu() {
    Serial.println(F("\n===== LineSensorTest — فحص QTR-25 (D28..D53) ====="));
    Serial.println(F("  l  بثّ خام 10s + تقرير ALIVE/DEAD لكل بن"));
    Serial.println(F("  r  اقرأ خام مرة     ?  قائمة"));
    Serial.println(F("  >> بالبثّ: مرّر الخط على طول المصفوفة كلها ببطء"));
    Serial.print(F("> "));
}

void printRaw() {
    qtr.read(v);
    for (uint8_t i = 0; i < N; i++) {
        Serial.print('p'); Serial.print(PINS[i]);
        Serial.print('='); Serial.print(v[i]); Serial.print(' ');
    }
    Serial.println();
}

// بثّ + تتبّع min/max لكل بن، وبالآخر تقرير حي/ميت
void streamAndReport(uint32_t ms) {
    uint16_t mn[N], mx[N];
    for (uint8_t i = 0; i < N; i++) { mn[i] = 65535; mx[i] = 0; }

    Serial.println(F(">> مرّر الخط على طول المصفوفة كلها الآن (10s)…"));
    uint32_t endAt = millis() + ms;
    while (millis() < endAt) {
        qtr.read(v);
        for (uint8_t i = 0; i < N; i++) {
            if (v[i] < mn[i]) mn[i] = v[i];
            if (v[i] > mx[i]) mx[i] = v[i];
        }
        // سطر مختصر: بس القيم (للمراقبة)
        for (uint8_t i = 0; i < N; i++) { Serial.print(v[i]); Serial.print(' '); }
        Serial.println();
        delay(250);
    }

    // التقرير: قناة "حيّة" لو تغيّرت وشافت أبيض (min منخفض)؛ "ميتة" لو ضلّت عالية
    Serial.println(F("\n===== التقرير ====="));
    Serial.println(F("  بن(pin) : min..max  → الحالة"));
    uint8_t alive = 0;
    for (uint8_t i = 0; i < N; i++) {
        bool dead = (mn[i] >= STUCK_TH);          // ما شاف أبيض أبداً = عالق/مفصول
        Serial.print(F("   p")); Serial.print(PINS[i]);
        if (PINS[i] < 10) Serial.print(' ');
        Serial.print(F(" : ")); Serial.print(mn[i]);
        Serial.print(F("..")); Serial.print(mx[i]);
        Serial.println(dead ? F("  → DEAD (عالق)") : F("  → ALIVE ✓"));
        if (!dead) alive++;
    }
    Serial.print(F("  المجموع: ")); Serial.print(alive);
    Serial.println(F(" قناة شغّالة."));
    Serial.println(F("  >> انسخ قائمة الـ ALIVE وابعتها — بنبني منها الـ14."));
    Serial.print(F("> "));
}

void handle(char c) {
    if      (c == 'r') { printRaw(); Serial.print(F("> ")); }
    else if (c == 'l') streamAndReport(10000);
    else if (c == '?') printMenu();
    else { Serial.println(F("   ؟ أمر غير معروف — ? للقائمة")); Serial.print(F("> ")); }
}

void setup() {
    Serial.begin(115200);
    qtr.setTypeRC();
    qtr.setSensorPins(PINS, N);
    qtr.setEmitterPins(EMIT_EVEN, EMIT_ODD);
    qtr.setTimeout(TIMEOUT_US);
    delay(300);
    Serial.println(F("\n[LineSensorTest] جاهز — فحص كل قنوات QTR-25."));
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
