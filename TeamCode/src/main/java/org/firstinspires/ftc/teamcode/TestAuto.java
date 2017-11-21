package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by HP 15t-as100 on 9/25/2017.
 */
@Autonomous (name = "TestAuto", group = "12596")
public class TestAuto extends AutoMode {

    @Override


 /*   void runAutoMode() {
        turnToHeading(180);
    }*/
    void runAutoMode () {
        double initJewelSwiperPos = jewelSwiper.getPosition();
        int glyphPosition = -1;
        telemetry.addData("Encoder Position L ", left.getCurrentPosition());
        telemetry.addData("Encoder Postition C ", center.getCurrentPosition());
        telemetry.addData("Encoder Position Rt ", right.getCurrentPosition());
        telemetry.update();
        jewelSwiper.setPosition(0);
        jewel2.setPosition(0.5);
        lHDrive.setPosition(0.5);
        rHDrive.setPosition(0.5);
        while (opModeIsActive()) {
            double jewelSwiperCurrentPos = jewelSwiper.getPosition();
            vuforia.start();
            Vuforia();
            sleep(2000);
            vuforia.stop();
            telemetry.addData("Vuforia Column: ", Vuforia());
            telemetry.update();
            jewelSwiper.setPosition(.5);
            sleep(1500);

            if (isJewelRed() == true) {
                jewel2.setPosition(1);
                sleep(2000);
                jewelSwiper.setPosition(1);

            } else if (isJewelRed() == false) {
                jewel2.setPosition(0);
                sleep(2000);
                jewelSwiper.setPosition(1);
            }
            sleep(1000);
            goDistance(30, .9, true);
            if (Vuforia() == 1) {
                goDistanceCenter(4, .5);
            }
            else if (Vuforia() == 2) {
                goDistanceCenter(8, .5);
            }
            else if (Vuforia() == 3) {
                goDistanceCenter(12, .5);
            }
            omniPusher.setPosition(1);
            sleep(1000);
            omniPusher.setPosition(0);
            goDistance(15, -.6, true);



            jewelSwiperCurrentPos = jewelSwiper.getPosition();
        }




        telemetry.addData("Encoder Position Lt ", left.getCurrentPosition());
        telemetry.addData("Encoder Postition C ", center.getCurrentPosition());
        telemetry.addData("Encoder Postition Rt ", right.getCurrentPosition());
        telemetry.update();
        //sleep(5000);
    }
}
