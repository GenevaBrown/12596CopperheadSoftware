package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by HP 15t-as100 on 9/25/2017.
 */
@Autonomous (name = "JewelSwiperTest", group = "12596")
public class JewelSwipeTest extends AutoMode {

    @Override


 /*   void runAutoMode() {
        turnToHeading(180);
    }*/
    void runAutoMode () {

        waitForStart();

        while (opModeIsActive()) {
           jewelSwiper.setPosition(1);
            telemetry.addData("Jewel Swipe Pos: ", "1");
            telemetry.update();
            sleep (1000);
            jewelSwiper.setPosition(.75);
            telemetry.addData("Jewel Swipe Pos: ", ".75");
            telemetry.update();
            sleep(1000);
            jewelSwiper.setPosition(.5);
            telemetry.addData("Jewel Swipe Pos: ", ".5");
            telemetry.update();
            sleep(1000);
            jewelSwiper.setPosition(.25);
            telemetry.addData("Jewel Swipe Pos: ", ".25");
            telemetry.update();
            sleep(1000);
            jewelSwiper.setPosition(0);
            telemetry.addData("Jewel Swipe Pos: ", "0");
            telemetry.update();
            sleep(1000);
            jewelSwiper.setPosition(.75);
            sleep(1000);
            jewel2.setPosition(1);
            sleep(1000);
            telemetry.addData("Pos: ", jewel2.getPosition());
            telemetry.update();
            jewel2.setPosition(.5);
            sleep(1000);
            telemetry.addData("Pos: ", jewel2.getPosition());
            telemetry.update();
            jewel2.setPosition(0);
            telemetry.addData("Pos: ", jewel2.getPosition());
            telemetry.update();
            sleep(1000);

        }

        }





}
