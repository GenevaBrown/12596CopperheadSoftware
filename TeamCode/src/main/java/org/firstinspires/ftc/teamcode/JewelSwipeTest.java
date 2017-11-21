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

        while (opModeIsActive()) {
            jewelSwiper.setPosition(.25);
            sleep(1000);
            jewel2.setPosition(1);
            sleep(1000);
            jewel2.setPosition(0);
            sleep(1000);

        }

        }





}
