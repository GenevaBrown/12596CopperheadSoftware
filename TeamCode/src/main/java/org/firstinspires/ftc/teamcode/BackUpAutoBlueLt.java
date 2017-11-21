package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by HP 15t-as100 on 9/25/2017.
 */
//@Autonomous (name = "AutoBlueLt", group = "12596")
public class BackUpAutoBlueLt extends AutoMode {

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
        //int i = 0;
        jewelSwiper.setPosition(0);
        //double IMUInitPitch = IMU.getPitch();
        lHDrive.setPosition(0.5);
        rHDrive.setPosition(0.5);
        colorSensor.enableLed(false);

            double jewelSwiperCurrentPos = jewelSwiper.getPosition();
            //double IMUpitch = IMU.getPitch();
           /* telemetry.addData("Position", left.getCurrentPosition());
            telemetry.update();*/
            jewelSwiper.setPosition(.57);
           // if (getDecodedColumn)

            sleep(1500);

            if (isJewelRed() == true) {
                jewel2.setPosition(1);
                jewelSwiper.setPosition(1);
                sleep(1000);
                goDistance(20, .9, true);
                goDistanceCenter(40, -.7);
                stop();

            } else {
                jewel2.setPosition(0);
                goDistanceCenter(45, -.85);
                jewelSwiper.setPosition(1);
                sleep(1000);
                goDistance(25, .7, true);
                goDistanceCenter(50, -.7);
                stop();

                /*if (isLineBlue() == false) {
                    goDistanceCenter(50, .7);
                    left.setPower(-.5);
                    right.setPower(-.5);
                } else if (isLineBlue() == true) {
                    break;
                }*/

            jewelSwiperCurrentPos = jewelSwiper.getPosition();
        }




        telemetry.addData("Encoder Position Lt ", left.getCurrentPosition());
        telemetry.addData("Encoder Postition C ", center.getCurrentPosition());
        telemetry.addData("Encoder Postition Rt ", right.getCurrentPosition());
        telemetry.update();
        //sleep(5000);
    }
}
