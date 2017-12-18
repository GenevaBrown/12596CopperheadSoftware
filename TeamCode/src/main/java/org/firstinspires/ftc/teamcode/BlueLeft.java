package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by HP 15t-as100 on 9/25/2017.
 */
@Autonomous (name = "BlueLt", group = "12596")
public class BlueLeft extends AutoMode {

    @Override


    void runAutoMode () {
        waitForStart();
        SuperAuto(true, false, false, false, false, true);
        goDistance(30, .7, false, 7);

    }
}
