package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by HP 15t-as100 on 9/25/2017.
 */
@Autonomous (name = "BlueRt", group = "12596")
public class BlueRight extends AutoMode {

    @Override


    void runAutoMode () {
        waitForStart();
        SuperAuto (true, false, true, false, false, true);

    }
}
