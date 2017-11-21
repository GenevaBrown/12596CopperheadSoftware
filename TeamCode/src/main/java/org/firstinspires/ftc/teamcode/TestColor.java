package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by HP 15t-as100 on 9/25/2017.
 */
@Autonomous (name = "TestColor", group = "12596")
public class TestColor extends AutoMode {

    @Override


 /*   void runAutoMode() {
        turnToHeading(180);
    }*/
    void runAutoMode () {
        colorSensor.enableLed(true);
        boolean isRed = isJewelRed();
        telemetry.addData("Red?",isRed);
        telemetry.update();

        /*telemetry.addData("Alpha", ColorSensor.alpha());
        telemetry.addData("Red  ", ColorSensor.red());
        telemetry.addData("Green", ColorSensor.green());
        telemetry.addData("Blue ", ColorSensor.blue());*/
    }
}
