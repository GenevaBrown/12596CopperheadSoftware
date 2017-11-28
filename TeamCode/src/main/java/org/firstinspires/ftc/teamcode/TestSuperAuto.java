package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by HP 15t-as100 on 9/25/2017.
 */
@Autonomous (name = "TestSuperAutoRedLt", group = "12596")
public class TestSuperAuto extends AutoMode {

    @Override


    void runAutoMode () {
        waitForStart();
        SuperAuto(false, true, true, false, false, false);

    }
}
