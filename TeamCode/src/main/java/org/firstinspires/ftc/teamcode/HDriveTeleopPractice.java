package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by HP 15t-as100 on 9/22/2017.
 */
@TeleOp(name="H-Drive Teleop", group="12596")
public class HDriveTeleopPractice extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor centerMotor;
    DcMotor servoCollectorLt;
    DcMotor servoCollectorRt;
    Servo jewelSwiper;
    Servo jewel2;
    DcMotor HDrive;
    CRServo omnipulatorRt;
    CRServo omnipulatorLt;
    CRServo omniPusher;
    CRServo omniPusher2;
    DcMotor omnipulatorLift;
    DigitalChannel pusherStop;
    DigitalChannel pusherStop2;
    ElapsedTime Timer = new ElapsedTime();
    double dropHeight = 0.43;
    double recordTime = Timer.time();
    double recordPowerLeft = 0;
    double recordPowerRight = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        HDrive = hardwareMap.dcMotor.get("H");
        leftMotor = hardwareMap.dcMotor.get("L");
        rightMotor = hardwareMap.dcMotor.get("R");
        centerMotor = hardwareMap.dcMotor.get("C");
        servoCollectorLt = hardwareMap.dcMotor.get("LtCollector");
        servoCollectorRt = hardwareMap.dcMotor.get("RtCollector");
        jewelSwiper = hardwareMap.servo.get("jewelSwiper");
        jewel2 = hardwareMap.servo.get("jewel2");
        omnipulatorRt = hardwareMap.crservo.get("omnipulatorRt");
        omnipulatorLt = hardwareMap.crservo.get("omnipulatorLt");
        omniPusher = hardwareMap.crservo.get("pusher");
        omniPusher2 = hardwareMap.crservo.get("pusher2");
        omnipulatorLift = hardwareMap.dcMotor.get("Lift");
        pusherStop = hardwareMap.digitalChannel.get("pusherStop");
        pusherStop2 = hardwareMap.digitalChannel.get("pusherStop2");
        pusherStop.setMode(DigitalChannel.Mode.INPUT);
        pusherStop2.setMode(DigitalChannel.Mode.INPUT);
        centerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("leftStickPosition ", gamepad1.left_stick_y);
        telemetry.addData("rightStickPosition ", gamepad1.right_stick_y);
        telemetry.addData("leftTriggerPosition ", gamepad1.left_trigger);

        //double servoInitPositionLt = servoCollectorLt.getPosition();
        //double servoInitPositionRt = servoCollectorRt.getPosition();

        jewelSwiper.setPosition(0.23);
        jewel2.setPosition(0.95);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Jewel Swiper Pos: ", jewelSwiper.getPosition());
            telemetry.update();
            double jewelSwiperCurrentPos = jewelSwiper.getPosition();
            double jewel2CurrentPos = jewel2.getPosition();
            double omnipulatorLtCurrentPos = omnipulatorLt.getPower();
            double omnipulatorRtCurrentPos = omnipulatorRt.getPower();
            double left_stick = gamepad1.left_stick_y;
            double right_stick = gamepad1.right_stick_y;
            double maxPowerChange = 0.3;
            recordPowerLeft = left_stick;
            recordPowerRight = right_stick;
            double maxPowerLeft = recordPowerLeft + (maxPowerChange * Math.signum(recordPowerLeft));
            double maxPowerRight = recordPowerRight + (maxPowerChange * Math.signum(recordPowerRight));

            double currentTime = Timer.time();

            //double pusherCurrentPos = omniPusher.getPosition();
            //double pusher2CurrentPos = omniPusher2.getPosition();
            //double servoCurrentPosLt = servoCollectorLt.getPosition();
            //double servoCurrentPosRt = servoCollectorRt.getPosition();

            if (Math.abs(left_stick) > .01) {
                /*HDrive.setPower(1);
                sleep(750);
                HDrive.setPower(0);*/
                    leftMotor.setPower(processStick(left_stick));

            }
            /*if (Math.abs(gamepad1.left_stick_y) > .01 && Math.abs(gamepad1.left_stick_y) < .5) {
                leftMotor.setPower(gamepad1.left_stick_y / 2);
            }
            if (Math.abs(gamepad1.left_stick_y) >= .5 && Math.abs(gamepad1.left_stick_y) < .7) {
                leftMotor.setPower(gamepad1.left_stick_y / 1.25);
            }
            if (Math.abs(gamepad1.left_stick_y) >= .7) {
                leftMotor.setPower(gamepad1.left_stick_y);
            }*/
            else {
                leftMotor.setPower(0);
            }
            /*if (Math.abs(gamepad1.right_stick_y) > .01 && Math.abs(gamepad1.right_stick_y) < .5) {
                rightMotor.setPower(-gamepad1.right_stick_y / 2);
            }
            if (Math.abs(gamepad1.right_stick_y) >= .5 && Math.abs(gamepad1.right_stick_y) < .7) {
                rightMotor.setPower(-gamepad1.right_stick_y / 1.25);
            }
            if (Math.abs(gamepad1.right_stick_y) >= .7) {
                rightMotor.setPower(-gamepad1.right_stick_y);
            }*/
            if (Math.abs(right_stick) > .01) {
               /* HDrive.setPower(1);
                sleep(750);
                HDrive.setPower(0); */
                rightMotor.setPower(-processStick(right_stick));
            }
            else {
                rightMotor.setPower(0);
            }
            if (gamepad1.left_trigger > .05) {
                HDrive.setPower(-1);
                sleep(750);
                HDrive.setPower(0);
                centerMotor.setPower(-gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > .05) {
                HDrive.setPower(-1);
                sleep(750);
                HDrive.setPower(0);
                centerMotor.setPower(gamepad1.right_trigger);
            }
            else {
                centerMotor.setPower(0);
                //lHDrive.setPosition(0.5);
                //rHDrive.setPosition(0.5);

            }
            if (gamepad2.right_trigger > .05) {
                omnipulatorLt.setPower(.7);
                omnipulatorRt.setPower(-.7);
            }
            else if (gamepad2.left_trigger > .05) {
                omnipulatorLt.setPower(-.7);
                omnipulatorRt.setPower(.7);
            }
            else {
                omnipulatorLt.setPower(0);
                omnipulatorRt.setPower(0);
            }
            if (gamepad2.y) {
                omniPusher.setPower(.9);
                //ElapsedTime Timer = new ElapsedTime();
            }
            else {
                if (pusherStop.getState()) {
                    omniPusher.setPower(-.7);
                } else {
                    omniPusher.setPower(0);
                }
            }
            if (gamepad2.a) {
                omniPusher2.setPower(.9);
            }
            else {
                if (pusherStop2.getState()) {
                    omniPusher2.setPower(-.7);
                }
                else {
                    omniPusher2.setPower(0);
                }
            }
            if (gamepad2.dpad_up) {
                omnipulatorLift.setPower(.9);
            }
            else if (gamepad2.dpad_down) {
                omnipulatorLift.setPower(-.9);
            }
            else {
                omnipulatorLift.setPower(0);
            }
            if (gamepad2.right_bumper) {
                servoCollectorLt.setPower(.95);
                servoCollectorRt.setPower(-.95);
            }
            else if (gamepad2.left_bumper) {
                servoCollectorLt.setPower(-.95);
                servoCollectorRt.setPower(.95);
            }
            else if (gamepad2.x) {
                servoCollectorLt.setPower(.5);
                servoCollectorRt.setPower(.5);
                }
            else if (gamepad2.b) {
                servoCollectorRt.setPower(-.5);
                servoCollectorLt.setPower(-.5);
            }
            else {
                servoCollectorLt.setPower(0);
                servoCollectorRt.setPower (0);
            }


            if (gamepad1.dpad_up) {
                jewelSwiper.setPosition(0);
            }
            else if (gamepad1.dpad_down) {
                jewelSwiper.setPosition(.4);
            }
            else{
                jewelSwiper.setPosition(jewelSwiperCurrentPos);
            }
            if (gamepad1.dpad_left) {
                jewel2.setPosition(0);
            }
            else if (gamepad1.dpad_right) {
                jewel2.setPosition(1);
            }
            else {
                jewel2.setPosition(jewel2CurrentPos);
            }
            jewelSwiperCurrentPos = jewelSwiper.getPosition();



        }

    }
    double processStick (double inputStick) {
        double outputStick = 0;
        if (Math.abs(inputStick) > .01) {

            if (Math.abs(inputStick) < 0.7) {
                outputStick = inputStick * 0.4 + (.1 * Math.signum(inputStick));
            }
                else {
                outputStick = inputStick * 2.066 - (1.066 * Math.signum(inputStick)) ;
            }
        }
        return outputStick;
    }
}

