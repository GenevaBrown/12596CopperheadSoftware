package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by HP 15t-as100 on 9/25/2017.
 */
@Autonomous (name = "RedLt", group = "12596")
public class RedLeft extends AutoMode {

    @Override


    void runAutoMode () {
        waitForStart();
        SuperAuto(false, true, true, false, false, true);

    }
}
