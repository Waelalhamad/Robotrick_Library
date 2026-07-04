// =====================================================
//  CalibrateSpeed — قياس RT_PWM_STATIC و RT_KV (v2 Phase A)
//
//  لازم للحركة المخطّطة الجديدة (forward/backward).
//  ⚠️ حط الروبوت وقدامه ~1 متر مساحة فاضية.
//
//  شغّل، افتح Serial Monitor 115200، وخُد القيم المطبوعة
//  وحطها في Robotrick.h (أو من الداشبورد).
// =====================================================
#include <Robotrick.h>

Robotrick bot;

void setup() {
    bot.begin();          // init + gyro calibrate
    delay(500);
    bot.calibrateSpeed(); // يمشي دفعتين ويطبع RT_PWM_STATIC + RT_KV + السرعة القصوى
}

void loop() {}
