// =====================================================
//  QTRCheck — فحص شامل لمصفوفة الـ QTR-25 (RC)
//  الشيلد: بيانات D28..D53، emitters D26/D27
//
//  يشتغل تلقائي — افتح Serial Monitor على 115200.
//  ما بيحرّك المحركات. آمن تماماً.
//
//  كيف تستخدمه:
//    1. ارفعه، افتح Serial Monitor (115200).
//    2. حرّك الخط الأسود تحت المصفوفة ببطء من اليسار لليمين.
//    3. راقب أي حساس "يغمق" (يصير # أو X) لما يمرّ فوقه الخط.
//    4. بعد ~10 ثواني بيطبع تقرير صحّة: مين شغّال ومين خربان.
//    5. اكتب 'r' لتصفير الـ min/max وإعادة الفحص.
//
//  قراءة القيم:  فاتح/أبيض = رقم صغير | غامق/أسود = رقم كبير (لحد 2500)
//  حساس خربان: عالق على 2500 دايماً (ما بيتغيّر) = مفصول/تالف
// =====================================================
#include <QTRSensors.h>

#define NUM 25

// نفس ترتيب الشيلد (بدون D52 المحجوز)
const uint8_t PINS[NUM] = {
    29, 28, 31, 30, 33, 32, 35, 34,
    37, 36, 39, 38, 41, 40, 43, 42,
    45, 44, 47, 46, 49, 48, 51, 50, 53
};

QTRSensors qtr;
uint16_t v[NUM];
uint16_t mn[NUM], mx[NUM];   // أقل/أكبر قيمة شفناها لكل حساس

uint32_t lastBar = 0, lastHealth = 0;

char barChar(uint16_t x) {
    if (x < 300)   return '.';   // أبيض/فاتح
    if (x < 700)   return ':';   // رمادي فاتح
    if (x < 1300)  return '+';   // رمادي
    if (x < 2200)  return '#';   // غامق (خط)
    return 'X';                  // 2500 = أسود تام أو حساس عالق
}

void resetMinMax() {
    for (uint8_t i = 0; i < NUM; i++) { mn[i] = 9999; mx[i] = 0; }
}

void printRuler() {
    Serial.print(F("      idx: "));
    for (uint8_t i = 0; i < NUM; i++) Serial.print(i % 10);
    Serial.println();
}

void printHealth() {
    Serial.println(F("\n────── HEALTH CHECK (بعد ما حرّكت الخط) ──────"));
    Serial.print(F("STUCK (عالق ~2500 دايماً = خربان/مفصول): "));
    bool anyStuck = false;
    for (uint8_t i = 0; i < NUM; i++) {
        if (mn[i] >= 2200) { Serial.print(i); Serial.print(' '); anyStuck = true; }
    }
    if (!anyStuck) Serial.print(F("مافي — كله منيح"));
    Serial.println();

    Serial.print(F("NO SWING (ما تغيّر أبداً = ما شاف الخط/تالف):  "));
    bool anyFlat = false;
    for (uint8_t i = 0; i < NUM; i++) {
        if (mn[i] < 2200 && (mx[i] - mn[i]) < 250) { Serial.print(i); Serial.print(' '); anyFlat = true; }
    }
    if (!anyFlat) Serial.print(F("مافي"));
    Serial.println();

    Serial.print(F("GOOD (تباين واضح = ممتاز): "));
    uint8_t good = 0;
    for (uint8_t i = 0; i < NUM; i++)
        if (mn[i] < 700 && mx[i] > 1200) good++;
    Serial.print(good); Serial.println(F(" / 25 حساس"));
    Serial.println(F("──────────────────────────────────────────\n"));
}

void setup() {
    Serial.begin(115200);
    delay(300);

    qtr.setTypeRC();
    qtr.setSensorPins(PINS, NUM);
    qtr.setEmitterPins(26, 27);
    qtr.setTimeout(2500);

    pinMode(26, OUTPUT); digitalWrite(26, HIGH);   // شغّل الـ IR emitters
    pinMode(27, OUTPUT); digitalWrite(27, HIGH);

    resetMinMax();

    Serial.println(F("\n=== QTR-25 CHECK ==="));
    Serial.println(F("حرّك الخط الأسود ببطء تحت المصفوفة يمين↔يسار."));
    Serial.println(F("راقب أي عمود يصير # أو X لما يمرّ الخط فوقه."));
    Serial.println(F("'r' = تصفير الفحص\n"));
    printRuler();
}

void loop() {
    // أمر تصفير
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'r' || c == 'R') { resetMinMax(); Serial.println(F(">> reset")); }
    }

    qtr.read(v);

    // تحديث min/max
    for (uint8_t i = 0; i < NUM; i++) {
        if (v[i] < mn[i]) mn[i] = v[i];
        if (v[i] > mx[i]) mx[i] = v[i];
    }

    // ── بار مرئي + قيم كل 150ms ──
    if (millis() - lastBar >= 150) {
        lastBar = millis();

        Serial.print(F("LIVE  ["));
        for (uint8_t i = 0; i < NUM; i++) Serial.print(barChar(v[i]));
        Serial.print(F("]  "));

        // أي حساس أغمق حالياً (وين الخط)
        uint8_t darkest = 0; uint16_t dv = 0;
        for (uint8_t i = 0; i < NUM; i++) if (v[i] > dv) { dv = v[i]; darkest = i; }
        Serial.print(F("line@"));
        if (dv > 700) Serial.print(darkest); else Serial.print(F("?"));
        Serial.println();
    }

    // ── تقرير صحّة كل 4 ثواني ──
    if (millis() - lastHealth >= 4000) {
        lastHealth = millis();
        printHealth();
        printRuler();
    }
}
