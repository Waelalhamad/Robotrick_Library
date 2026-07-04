// =====================================================
//  FullTest — اختبار كل وظائف مكتبة Robotrick
//
//  افتح Serial Monitor على 115200 واكتب رقم الأمر:
//
//    1 = Forward 30cm        5 = Print heading
//    2 = Backward 30cm       6 = Motors test (يسار/يمين)
//    3 = Turn LEFT 90        7 = Encoder read
//    4 = Turn RIGHT 90       8 = Full map sequence
//                            9 = Re-calibrate gyro
//                            0 = STOP
//
//  بتقدر تختبر كل حركة لحالها بدون ما تعيد الرفع.
// =====================================================
#include <Robotrick.h>

Robotrick bot;

void printMenu() {
    Serial.println(F("\n========= Robotrick FullTest ========="));
    Serial.println(F("  1 = Forward 30cm     5 = Heading"));
    Serial.println(F("  2 = Backward 30cm    6 = Motors test"));
    Serial.println(F("  3 = Turn LEFT 90     7 = Encoder read"));
    Serial.println(F("  4 = Turn RIGHT 90    8 = Full map"));
    Serial.println(F("  9 = Calibrate gyro   0 = STOP"));
    Serial.println(F("======================================"));
    Serial.print(F("> "));
}

void setup() {
    bot.begin();          // init + gyro calibrate
    delay(500);
    printMenu();
}

void loop() {
    if (!Serial.available()) return;

    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') return;   // تجاهل الفراغات

    Serial.println(c);   // echo

    if (c == '1') {
        Serial.println(F(">>> Forward 30cm"));
        bot.forward(30);
    }
    else if (c == '2') {
        Serial.println(F(">>> Backward 30cm"));
        bot.backward(30);
    }
    else if (c == '3') {
        Serial.println(F(">>> Turn LEFT 90"));
        bot.turnLeft(90);
    }
    else if (c == '4') {
        Serial.println(F(">>> Turn RIGHT 90"));
        bot.turnRight(90);
    }
    else if (c == '5') {
        Serial.print(F(">>> Heading = "));
        Serial.print(bot.heading(), 1);
        Serial.println(F("°"));
    }
    else if (c == '6') {
        Serial.println(F(">>> Motors: LEFT fwd 1.5s"));
        bot.setMotors(120, 0);
        delay(1500);
        bot.stop();
        delay(500);
        Serial.println(F(">>> Motors: RIGHT fwd 1.5s"));
        bot.setMotors(0, 120);
        delay(1500);
        bot.stop();
        Serial.println(F("    (تأكد كل محرك لف صح — وإلا اقلب REVERSE)"));
    }
    else if (c == '7') {
        Serial.print(F(">>> Enc L = "));
        Serial.print(bot.encoderLeft());
        Serial.print(F("   Enc R = "));
        Serial.println(bot.encoderRight());
    }
    else if (c == '8') {
        Serial.println(F(">>> Full map sequence"));
        delay(2000);
        bot.turnLeft(90);
        bot.forward(30);
        bot.turnLeft(90);
        bot.forward(50);
        bot.stop();
        Serial.println(F("    map done."));
    }
    else if (c == '9') {
        Serial.println(F(">>> Re-calibrating gyro (keep STILL)"));
        bot.calibrateGyro();
    }
    else if (c == '0') {
        bot.stop();
        Serial.println(F(">>> STOPPED"));
    }
    else {
        Serial.println(F("؟ أمر غير معروف"));
    }

    printMenu();
}
