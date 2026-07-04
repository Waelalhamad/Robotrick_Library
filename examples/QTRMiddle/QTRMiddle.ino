// =====================================================
//  QTR MIDDLE-10 RAW READER
//  يقرأ الـ 10 حساسات الوسطى فقط (يتجنّب الميتين 3 و 24)
//
//  الاستخدام:
//    1. Serial Monitor على 115200
//    2. حط الروبوت على الأبيض  → اقرأ الأرقام (لازم تكون واطية)
//    3. حط الروبوت على الأسود  → اقرأ الأرقام (لازم تكون عالية)
//
//  QTR-RC: أبيض = قيمة واطية (~100-300)، أسود = قيمة عالية (1500-2500)
//  إذا الأسود ما وصل ~1500 → الحساس عالي/التباين ضعيف (نزّله لـ 2-3mm)
// =====================================================
#include <QTRSensors.h>

QTRSensors qtr;

// الـ 10 حساسات الوسطى (index 7..16 من مصفوفة الـ 25)
const uint8_t MID_PINS[10] = { 34, 37, 36, 39, 38, 41, 40, 43, 42, 45 };
uint16_t v[10];

// تتبّع أدنى/أعلى قيمة شافها كل حساس (يساعد تشوف المدى)
uint16_t mn[10], mx[10];

void setup() {
    Serial.begin(115200);

    qtr.setTypeRC();
    qtr.setSensorPins(MID_PINS, 10);
    qtr.setEmitterPins(26, 27);
    qtr.setTimeout(2500);
    pinMode(26, OUTPUT); digitalWrite(26, HIGH);
    pinMode(27, OUTPUT); digitalWrite(27, HIGH);

    for (uint8_t i = 0; i < 10; i++) { mn[i] = 9999; mx[i] = 0; }

    // تدفئة
    for (uint8_t i = 0; i < 8; i++) { qtr.read(v); delay(20); }

    Serial.println(F("=== QTR MIDDLE-10 RAW ==="));
    Serial.println(F("ch:  s7   s8   s9   s10  s11  s12  s13  s14  s15  s16 | sum"));
    Serial.println(F("أبيض=واطي، أسود=عالي. اكتب 'r' لتصفير المدى min/max."));
}

void loop() {
    if (Serial.available() && Serial.read() == 'r') {
        for (uint8_t i = 0; i < 10; i++) { mn[i] = 9999; mx[i] = 0; }
        Serial.println(F(">> min/max reset"));
    }

    qtr.read(v);

    uint32_t sum = 0;
    Serial.print(F("RAW "));
    for (uint8_t i = 0; i < 10; i++) {
        if (v[i] < mn[i]) mn[i] = v[i];
        if (v[i] > mx[i]) mx[i] = v[i];
        char buf[6];
        sprintf(buf, "%4u ", v[i]);
        Serial.print(buf);
        sum += v[i];
    }
    Serial.print(F("| sum=")); Serial.println(sum);

    // كل ثانية: اطبع المدى (min..max) لكل حساس
    static uint32_t last = 0;
    if (millis() - last > 1000) {
        last = millis();
        Serial.print(F("RANGE(min-max) "));
        for (uint8_t i = 0; i < 10; i++) {
            Serial.print(mn[i]); Serial.print('-'); Serial.print(mx[i]); Serial.print(' ');
        }
        Serial.println();
    }

    delay(200);
}
