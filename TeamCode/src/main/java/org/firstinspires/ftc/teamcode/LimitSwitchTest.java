package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by HP 15t-as100 on 9/25/2017.
 */
@Autonomous (name = "TouchSensorTest", group = "12596")
public class LimitSwitchTest extends AutoMode {
    DigitalChannel pusherStop;
    DigitalChannel pusherStop2;
    @Override


    void runAutoMode () {
        while (opModeIsActive()) {
            pusherStop = hardwareMap.digitalChannel.get("pusherStop");
            pusherStop2 = hardwareMap.digitalChannel.get("pusherStop2");
            pusherStop.setMode(DigitalChannel.Mode.INPUT);
            pusherStop2.setMode(DigitalChannel.Mode.INPUT);
            waitForStart();
            telemetry.addData("Pos: ", pusherStop.getState());
            telemetry.addData("Pos2: ", pusherStop2.getState());
            telemetry.update();
        }
    }
}
