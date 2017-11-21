package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by HP 15t-as100 on 11/2/2017.
 */
@TeleOp(name="Hello")
public class BlankMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("hello","helo2");
            telemetry.update();
        }
    }
}
