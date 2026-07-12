// =====================================================
//  ColorSensorTest — تجربة 4 حساسات TCS34725 (DIY bus-gate)
//  (مستقل — Wire فقط)
//
//  الحساسات كلهم 0x29؛ DIY mux على بنات INT:
//    S1→بن13   S2→بن10   S3→بن11   S4→بن9   LED→بن12
//
//  تحسين الدقّة والثبات:
//   • chromaticity: نصنّف بنسبة اللون r/(r+g+b) — مستقلة عن السطوع/البُعد
//     (بتحل مشكلة "القيم بتتغير") بدل الأرقام المطلقة.
//   • averaging: كل قراءة = معدّل عدّة عيّنات (تنعيم).
//   • TEACH: تعرض كل لون مرة ويسجّله، بعدين يصنّف بأقرب لون متعلّم
//     (بتحل مشكلة "اللون المطبوع غلط" — مثل أصفر→أحمر).
//
//  Serial 115200، Newline. الأوامر:
//    l        بثّ حيّ 15s (يصنّف كل حساس)
//    r        اقرأ الأربعة مرة
//    t<1..6>  علّم لون من الحساس 1: 1=احمر 2=اخضر 3=ازرق 4=اصفر 5=ابيض 6=اسود
//             (حط اللون قدام الحساس 1 واكتب مثلاً t1)
//    p        اطبع الألوان المتعلّمة
//    d        فحص السلوتات + العزل
//    o/x      LED on/off      ?  قائمة
// =====================================================
#include <Arduino.h>
#include <Wire.h>

#define TCS_ADDR   0x29
const uint8_t EN[4] = { 13, 10, 11, 9 };
#define LED_PIN    12

#define CMD        0x80
#define AUTOINC    0x20
#define R_ENABLE   0x00
#define R_ATIME    0x01
#define R_CONTROL  0x0F
#define R_ID       0x12
#define R_STATUS   0x13
#define R_CDATAL   0x14
#define EN_PON     0x01
#define EN_AEN     0x02
#define ATIME_VAL  0xC0
#define GAIN_4X    0x01

#define AVG_N       5      // عدد العيّنات للمعدّل (تنعيم)
#define BLACK_C     60     // C تحتها = اسود/معتم
#define TEACH_SLOT  0      // الحساس اللي نعلّم منه (0 = S1)

bool present[4] = { false, false, false, false };

struct Ref { float nr, ng, nb; bool set; };
Ref refs[6];
const char* refName[6] = { "احمر", "اخضر", "ازرق", "اصفر", "ابيض", "اسود" };

// قيم مقاسة من S3 (نقطة انطلاق — عدّلها بـ t<لون><حساس> إذا لزم)
// الترتيب: احمر، اخضر، ازرق، اصفر، ابيض(غير مقاس)، اسود
const Ref refDefault[6] = {
    { 0.54f, 0.22f, 0.24f, true  },   // احمر
    { 0.23f, 0.43f, 0.34f, true  },   // اخضر
    { 0.18f, 0.36f, 0.47f, true  },   // ازرق
    { 0.47f, 0.34f, 0.20f, true  },   // اصفر
    { 0.00f, 0.00f, 0.00f, false },   // ابيض — غير مقاس بعد
    { 0.25f, 0.36f, 0.39f, true  },   // اسود
};

char buf[16]; uint8_t idx = 0;

void selectOnly(int slot) {
    for (uint8_t i = 0; i < 4; i++) digitalWrite(EN[i], (i == slot) ? HIGH : LOW);
    delayMicroseconds(600);          // settle أطول = أثبت
}
void deselectAll() { for (uint8_t i = 0; i < 4; i++) digitalWrite(EN[i], LOW); }
bool ackTCS() { Wire.beginTransmission(TCS_ADDR); return Wire.endTransmission() == 0; }

bool wByte(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(TCS_ADDR); Wire.write(CMD | reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}
uint8_t rByte(uint8_t reg) {
    Wire.beginTransmission(TCS_ADDR); Wire.write(CMD | reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    Wire.requestFrom((uint8_t)TCS_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}
void configSensor() {
    wByte(R_ENABLE, EN_PON); delay(3);
    wByte(R_ATIME, ATIME_VAL);
    wByte(R_CONTROL, GAIN_4X);
    wByte(R_ENABLE, EN_PON | EN_AEN);
}

// قراءة خام وحدة (بعد select)
bool readRaw(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b) {
    uint32_t t0 = millis();
    while (millis() - t0 < 250) { if (rByte(R_STATUS) & 0x01) break; delay(5); }
    Wire.beginTransmission(TCS_ADDR);
    Wire.write(CMD | AUTOINC | R_CDATAL);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)TCS_ADDR, (uint8_t)8);
    if (Wire.available() < 8) return false;
    uint8_t d[8]; for (uint8_t i = 0; i < 8; i++) d[i] = Wire.read();
    c = (d[1] << 8) | d[0]; r = (d[3] << 8) | d[2];
    g = (d[5] << 8) | d[4]; b = (d[7] << 8) | d[6];
    return true;
}

// معدّل AVG_N عيّنات لحساس (تنعيم) — يرجّع المتوسط
bool readAvg(int slot, uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b) {
    selectOnly(slot);
    if (!ackTCS()) { deselectAll(); return false; }
    uint32_t sc = 0, sr = 0, sg = 0, sb = 0; uint8_t n = 0;
    for (uint8_t k = 0; k < AVG_N; k++) {
        uint16_t cc, rr, gg, bb;
        if (readRaw(cc, rr, gg, bb)) { sc += cc; sr += rr; sg += gg; sb += bb; n++; }
    }
    deselectAll();
    if (n == 0) return false;
    c = sc / n; r = sr / n; g = sg / n; b = sb / n;
    return true;
}

// chromaticity: نسبة كل قناة من (r+g+b) — مستقلة عن السطوع
void chroma(uint16_t r, uint16_t g, uint16_t b, float &nr, float &ng, float &nb) {
    float s = (float)r + g + b; if (s < 1) s = 1;
    nr = r / s; ng = g / s; nb = b / s;
}

// التصنيف: اسود لو C واطي؛ وإلا أقرب لون متعلّم بالـ chromaticity
const char* classify(uint16_t c, float nr, float ng, float nb) {
    if (c < BLACK_C) return "اسود/معتم";
    int best = -1; float bestD = 1e9;
    for (uint8_t i = 0; i < 6; i++) {
        if (!refs[i].set) continue;
        float d = sq(nr - refs[i].nr) + sq(ng - refs[i].ng) + sq(nb - refs[i].nb);
        if (d < bestD) { bestD = d; best = i; }
    }
    if (best < 0) return "(علّم الألوان أول: t1..t6)";
    return refName[best];
}

void teach(uint8_t n, uint8_t slot) {   // n=1..6 لون، slot=0..3 الحساس
    if (n < 1 || n > 6) { Serial.println(F("  رقم لون 1..6")); return; }
    if (slot > 3)       { Serial.println(F("  رقم حساس 1..4")); return; }
    uint16_t c, r, g, b;
    if (!readAvg(slot, c, r, g, b)) { Serial.print(F("  فشلت القراءة من S")); Serial.println(slot + 1); return; }
    float nr, ng, nb; chroma(r, g, b, nr, ng, nb);
    refs[n - 1] = { nr, ng, nb, true };
    Serial.print(F("  ✓ علّمت ")); Serial.print(refName[n - 1]);
    Serial.print(F(" من S")); Serial.print(slot + 1);
    Serial.print(F(":  nr=")); Serial.print(nr, 3);
    Serial.print(F(" ng=")); Serial.print(ng, 3);
    Serial.print(F(" nb=")); Serial.print(nb, 3);
    Serial.print(F("  (C=")); Serial.print(c); Serial.println(F(")"));
}

void printRefs() {
    Serial.println(F("  الألوان المتعلّمة:"));
    for (uint8_t i = 0; i < 6; i++) {
        Serial.print(F("   ")); Serial.print(i + 1); Serial.print(F(". ")); Serial.print(refName[i]);
        if (refs[i].set) {
            Serial.print(F("  nr=")); Serial.print(refs[i].nr, 3);
            Serial.print(F(" ng=")); Serial.print(refs[i].ng, 3);
            Serial.print(F(" nb=")); Serial.println(refs[i].nb, 3);
        } else Serial.println(F("  — غير متعلّم"));
    }
}

// اطبع قراءة حساس واحد (s = 0..3)
void printSensor(uint8_t s) {
    Serial.print(F("  S")); Serial.print(s + 1); Serial.print(F(": "));
    if (!present[s]) { Serial.println(F("مفقود")); return; }
    uint16_t c, r, g, b;
    if (!readAvg(s, c, r, g, b)) { Serial.println(F("فشلت")); return; }
    float nr, ng, nb; chroma(r, g, b, nr, ng, nb);
    Serial.print(F("C=")); Serial.print(c);
    Serial.print(F(" R=")); Serial.print(r);
    Serial.print(F(" G=")); Serial.print(g);
    Serial.print(F(" B=")); Serial.print(b);
    Serial.print(F(" | nr=")); Serial.print(nr, 2);
    Serial.print(F(" ng=")); Serial.print(ng, 2);
    Serial.print(F(" nb=")); Serial.print(nb, 2);
    Serial.print(F("  → ")); Serial.println(classify(c, nr, ng, nb));
}

void readAll() { for (uint8_t s = 0; s < 4; s++) printSensor(s); }

// بثّ حساس واحد لوحده ~10s (لاختبار الثبات)
void streamOne(uint8_t s) {
    if (s > 3) return;
    Serial.print(F(">> بثّ الحساس ")); Serial.print(s + 1); Serial.println(F(" لوحده 10s:"));
    uint32_t endAt = millis() + 10000;
    while (millis() < endAt) printSensor(s);
    Serial.print(F("> "));
}

void scanConfig() {
    Serial.println(F("\n>> فحص السلوتات:"));
    Serial.println(F("  slot | ACK | ID   | حالة"));
    for (uint8_t s = 0; s < 4; s++) {
        selectOnly(s);
        bool ack = ackTCS();
        uint8_t id = ack ? rByte(R_ID) : 0xFF;
        bool ok = (id == 0x44 || id == 0x4D || id == 0x10);
        if (ok) configSensor();
        present[s] = ok;
        Serial.print(F("   ")); Serial.print(s + 1);
        Serial.print(F("   |  ")); Serial.print(ack ? F("Y") : F("-"));
        Serial.print(F("  | 0x")); if (id < 0x10) Serial.print('0'); Serial.print(id, HEX);
        Serial.print(F(" | ")); Serial.println(ok ? F("OK") : F("مفقود"));
    }
    deselectAll(); delayMicroseconds(600);
    Serial.print(F("  الكل مطفي → ACK؟ "));
    Serial.println(ackTCS() ? F("!! نعم (العزل مكسور)") : F("لا ✓"));
}

void printMenu() {
    Serial.println(F("\n===== ColorSensorTest (chromaticity + teach) ====="));
    Serial.println(F("  l  بثّ الأربعة 15s   r  اقرأ الأربعة مرة"));
    Serial.println(F("  1..4  اقرأ حساس واحد مرة    s<n> بثّ حساس واحد 10s (مثل s2)"));
    Serial.println(F("  t<لون><حساس>  علّم لون من حساس محدّد (t1=احمر من S1، t23=اخضر من S3)"));
    Serial.println(F("     الألوان: 1احمر 2اخضر 3ازرق 4اصفر 5ابيض 6اسود  (الحساس اختياري، افتراضي S1)"));
    Serial.println(F("  p  اطبع المتعلّم   d  فحص/عزل   o/x LED   ?  قائمة"));
    Serial.print(F("> "));
}

void handle(char* s) {
    char cmd = s[0];
    if      (cmd == 'r') { readAll(); Serial.print(F("> ")); }
    else if (cmd == 'l') {
        Serial.println(F(">> بثّ 15s:"));
        uint32_t endAt = millis() + 15000;
        while (millis() < endAt) { readAll(); Serial.println(); }
        Serial.print(F("> "));
    }
    else if (cmd >= '1' && cmd <= '4') { printSensor(cmd - '1'); Serial.print(F("> ")); }
    else if (cmd == 's') { streamOne((uint8_t)(s[1] - '1')); }
    else if (cmd == 't') {
        uint8_t color = (s[1] >= '1' && s[1] <= '6') ? (s[1] - '0') : 0;
        uint8_t slot  = (s[2] >= '1' && s[2] <= '4') ? (s[2] - '1') : TEACH_SLOT; // افتراضي S1
        teach(color, slot); Serial.print(F("> "));
    }
    else if (cmd == 'p') { printRefs(); Serial.print(F("> ")); }
    else if (cmd == 'd') { scanConfig(); Serial.print(F("> ")); }
    else if (cmd == 'o') { digitalWrite(LED_PIN, HIGH); Serial.println(F("  LED ON")); Serial.print(F("> ")); }
    else if (cmd == 'x') { digitalWrite(LED_PIN, LOW);  Serial.println(F("  LED OFF")); Serial.print(F("> ")); }
    else if (cmd == '?') printMenu();
    else { Serial.println(F("  ؟ أمر غير معروف")); Serial.print(F("> ")); }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000);
    for (uint8_t i = 0; i < 4; i++) { pinMode(EN[i], OUTPUT); digitalWrite(EN[i], LOW); }
    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
    for (uint8_t i = 0; i < 6; i++) refs[i] = refDefault[i];   // حمّل القيم المثبّتة
    delay(300);
    Serial.println(F("\n[ColorSensorTest] جاهز — chromaticity + teach."));
    scanConfig();
    printMenu();
}

void loop() {
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\r') continue;
        if (ch == '\n') { if (idx > 0) { buf[idx] = '\0'; handle(buf); idx = 0; } }
        else if (idx < sizeof(buf) - 1) buf[idx++] = ch;
    }
}
