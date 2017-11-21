package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by HP 15t-as100 on 9/25/2017.
 */
@Disabled
@Autonomous (name = "AutoBlueRt", group = "12596")
public class BackUpAutoBlueRt extends AutoMode {

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
        jewelSwiper.setPosition(1);
        double jewelSwiperInitPos = jewelSwiper.getPosition();
        //double IMUInitPitch = IMU.getPitch();
        lHDrive.setPosition(0.5);
        rHDrive.setPosition(0.5);

        while (opModeIsActive()) {
            double jewelSwiperCurrentPos = jewelSwiper.getPosition();
            //double IMUpitch = IMU.getPitch();
           /* telemetry.addData("Position", left.getCurrentPosition());
            telemetry.update();*/
            jewelSwiper.setPosition(.4);
           // if (getDecodedColumn)

            sleep(1500);

            if (isJewelRed() == true) {
                jewel2.setPosition(1);
                sleep(1000);
                jewelSwiper.setPosition(1);
                sleep (1000);
                goDistance(30, .7, true);
                sleep (500);
                goDistanceCenter(55, -.7);
                stop();

            } else if (isJewelRed() == false) {
                jewel2.setPosition(0);
                sleep(1000);
                jewelSwiper.setPosition(1);
                sleep(1000);
                goDistance(35, .9, true);
                goDistanceCenter(50, -.9);
                stop();
            }
            jewelSwiperCurrentPos = jewelSwiper.getPosition();
        }


        telemetry.addData("Encoder Position Lt ", left.getCurrentPosition());
        telemetry.addData("Encoder Postition C ", center.getCurrentPosition());
        telemetry.addData("Encoder Postition Rt ", right.getCurrentPosition());
        telemetry.update();
        //sleep(5000);
    }
}
