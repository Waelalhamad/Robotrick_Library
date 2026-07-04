// =====================================================
//  TuneTurn — معايرة دقة اللف وثبات المشي المستقيم
//
//  يعمل: لف يسار 90 → يمين 90 → لقدام 50 → لخلف 50
//  راقب Serial Monitor (115200) وقيس الزوايا/المسافات.
//
//  إذا اللف مش دقيق:
//    • عدّل RT_TURN_TOLERANCE / RT_TURN_KP في Robotrick.h
//    • الجايرو هو المرجع — الدقة لازم تكون عالية أصلاً
//  إذا المشي مش مستقيم:
//    • عدّل RT_STRAIGHT_KP (زِد = تصحيح أقوى)
//    • أو اقلب RT_L_REVERSE / RT_R_REVERSE لو محرك بالعكس
// =====================================================
#include <Robotrick.h>

Robotrick bot;

void setup() {
    bot.begin();
    delay(1000);

    Serial.println(F("--- Turn LEFT 90 ---"));
    bot.turnLeft(90);
    delay(1000);

    Serial.println(F("--- Turn RIGHT 90 ---"));
    bot.turnRight(90);
    delay(1000);

    Serial.println(F("--- Forward 50cm ---"));
    bot.forward(50);
    delay(1000);

    Serial.println(F("--- Backward 50cm ---"));
    bot.backward(50);

    bot.stop();
    Serial.println(F("=== DONE ==="));
}

void loop() {}
