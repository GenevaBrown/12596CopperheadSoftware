package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by HP 15t-as100 on 9/25/2017.
 */
@Autonomous (name = "Test Turn", group = "12596")
public class TestTurn extends AutoMode {

    @Override
    void runAutoMode() {
        double startHeading = IMU.getHeading();
        double previousHeading = startHeading;
        double jumpAngle = 0;
        double currentHeading = IMU.getHeading();
        waitForStart();
        while (opModeIsActive()) {
            while ((Math.abs(currentHeading - startHeading) < Math.abs(1000000000)) && opModeIsActive()) {
                currentHeading = IMU.getHeading() + jumpAngle;
                if (currentHeading > 180 + previousHeading) {
                    jumpAngle = -360;
                    currentHeading = currentHeading + jumpAngle;
                    telemetry.addData("Jump Angle: ", jumpAngle);
                    telemetry.update();
                }
                if (currentHeading < previousHeading - 180) {
                    jumpAngle = 360;
                    currentHeading = currentHeading + jumpAngle;
                    telemetry.addData("Jump Angle: ", jumpAngle);
                    telemetry.update();
                }
                telemetry.addData("Current Heading: ", currentHeading);
                telemetry.update();
                previousHeading = currentHeading;
            }
        }
    }
}