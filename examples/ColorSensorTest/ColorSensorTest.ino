// =====================================================
//  ColorSensorTest — تجربة 4 حساسات لون TCS34725 (DIY bus-gate)
//  (مستقل — Wire فقط، ما بيعتمد على مكتبة MegaShield)
//
//  الفكرة: الـ4 حساسات كلهم على نفس العنوان 0x29. مربوطين على
//  DIY mux: كل حساس عليه FETs على SDA/SCL، بوابتها على بن INT.
//  تشغّل سلوت = تعطي بنّه HIGH → بس هداك ينوصل عالباص.
//    الحساس 1 → بن 13    الحساس 2 → بن 10
//    الحساس 3 → بن 11    الحساس 4 → بن 9
//    LED مشترك (إضاءة) → بن 12 (HIGH = مضوّي)
//
//  Serial 115200، Line ending = Newline.
//  الأوامر:
//    r   اقرأ الأربعة مرة وحدة
//    l   بثّ حيّ للأربعة ~12s (حط ألوان قدامهم وراقب)
//    d   فحص العزل: يتأكد لما الكل مطفي مافي ACK، وأي سلوت حي + ID
//    o   LED on      x  LED off
//    ?   القائمة
// =====================================================
#include <Arduino.h>
#include <Wire.h>

#define TCS_ADDR   0x29
const uint8_t EN[4] = { 13, 10, 11, 9 };   // بوابات السلوت 1..4
#define LED_PIN    12

// سجلات TCS34725
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
#define ATIME_VAL  0xC0    // ~154ms تكامل (متل إعداد الشيلد)
#define GAIN_4X    0x01

bool present[4] = { false, false, false, false };
char buf[16]; uint8_t idx = 0;

void selectOnly(int slot) {
    for (uint8_t i = 0; i < 4; i++) digitalWrite(EN[i], (i == slot) ? HIGH : LOW);
    delayMicroseconds(200);
}
void deselectAll() { for (uint8_t i = 0; i < 4; i++) digitalWrite(EN[i], LOW); }

bool ackTCS() { Wire.beginTransmission(TCS_ADDR); return Wire.endTransmission() == 0; }

bool wByte(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(TCS_ADDR);
    Wire.write(CMD | reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}
uint8_t rByte(uint8_t reg) {
    Wire.beginTransmission(TCS_ADDR);
    Wire.write(CMD | reg);
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

// فحص + تهيئة كل سلوت (يحدّد مين موجود)
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
    deselectAll();
    // تأكيد العزل
    delayMicroseconds(200);
    Serial.print(F("  كل السلوتات مطفية → ACK على 0x29؟ "));
    Serial.println(ackTCS() ? F("!! نعم (العزل مكسور)") : F("لا ✓ (عزل سليم)"));
}

bool readOne(int slot, uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b) {
    selectOnly(slot);
    if (!ackTCS()) { deselectAll(); return false; }
    // انتظر AVALID (تكامل خلص)
    uint32_t t0 = millis();
    while (millis() - t0 < 250) { if (rByte(R_STATUS) & 0x01) break; delay(5); }
    Wire.beginTransmission(TCS_ADDR);
    Wire.write(CMD | AUTOINC | R_CDATAL);
    if (Wire.endTransmission(false) != 0) { deselectAll(); return false; }
    Wire.requestFrom((uint8_t)TCS_ADDR, (uint8_t)8);
    if (Wire.available() < 8) { deselectAll(); return false; }
    uint8_t d[8]; for (uint8_t i = 0; i < 8; i++) d[i] = Wire.read();
    c = (d[1] << 8) | d[0];
    r = (d[3] << 8) | d[2];
    g = (d[5] << 8) | d[4];
    b = (d[7] << 8) | d[6];
    deselectAll();
    return true;
}

const char* classify(uint16_t c, uint16_t r, uint16_t g, uint16_t b) {
    if (c < 40) return "اسود/لاشي";
    long rr = (long)r * 255 / c, gg = (long)g * 255 / c, bb = (long)b * 255 / c;
    if (rr > 120 && gg > 120 && bb > 120) return "ابيض";
    if (rr >= gg && rr >= bb) return "احمر";
    if (gg >= rr && gg >= bb) return "اخضر";
    return "ازرق";
}

void readAll() {
    for (uint8_t s = 0; s < 4; s++) {
        Serial.print(F("  S")); Serial.print(s + 1); Serial.print(F(": "));
        if (!present[s]) { Serial.println(F("مفقود")); continue; }
        uint16_t c, r, g, b;
        if (readOne(s, c, r, g, b)) {
            Serial.print(F("C=")); Serial.print(c);
            Serial.print(F(" R=")); Serial.print(r);
            Serial.print(F(" G=")); Serial.print(g);
            Serial.print(F(" B=")); Serial.print(b);
            Serial.print(F("  → ")); Serial.println(classify(c, r, g, b));
        } else Serial.println(F("فشلت القراءة"));
    }
}

void printMenu() {
    Serial.println(F("\n===== ColorSensorTest (4x TCS34725) ====="));
    Serial.println(F("  r  اقرأ الأربعة   l  بثّ حيّ 12s"));
    Serial.println(F("  d  فحص السلوتات + العزل"));
    Serial.println(F("  o  LED on   x  LED off   ?  قائمة"));
    Serial.print(F("> "));
}

void handle(char cmd) {
    if      (cmd == 'r') { readAll(); Serial.print(F("> ")); }
    else if (cmd == 'l') {
        Serial.println(F(">> بثّ 12s — حط ألوان قدام الحساسات:"));
        uint32_t endAt = millis() + 12000;
        while (millis() < endAt) { readAll(); Serial.println(); delay(600); }
        Serial.print(F("> "));
    }
    else if (cmd == 'd') { scanConfig(); Serial.print(F("> ")); }
    else if (cmd == 'o') { digitalWrite(LED_PIN, HIGH); Serial.println(F("  LED ON")); Serial.print(F("> ")); }
    else if (cmd == 'x') { digitalWrite(LED_PIN, LOW);  Serial.println(F("  LED OFF")); Serial.print(F("> ")); }
    else if (cmd == '?') printMenu();
    else { Serial.println(F("  ؟ أمر غير معروف")); Serial.print(F("> ")); }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    for (uint8_t i = 0; i < 4; i++) { pinMode(EN[i], OUTPUT); digitalWrite(EN[i], LOW); }
    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);   // إضاءة ON
    delay(300);
    Serial.println(F("\n[ColorSensorTest] جاهز."));
    scanConfig();
    printMenu();
}

void loop() {
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\r') continue;
        if (ch == '\n') { if (idx > 0) { buf[idx] = '\0'; handle(buf[0]); idx = 0; } }
        else if (idx < sizeof(buf) - 1) buf[idx++] = ch;
    }
}
