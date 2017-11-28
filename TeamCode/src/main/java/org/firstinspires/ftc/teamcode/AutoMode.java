package org.firstinspires.ftc.teamcode;

/**
 * Created by arvindv on 9/25/17.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static android.R.attr.key;

public abstract class AutoMode extends LinearOpMode {


    DcMotor left;
    DcMotor right;
    DcMotor center;
    ColorSensor colorSensor;
    IMU IMU;
    DcMotor servoCollectorLt;
    DcMotor servoCollectorRt;
    CRServo omnipulatorRt;
    CRServo omnipulatorLt;
    Servo jewelSwiper;
    Servo jewel2;
    Servo lHDrive;
    Servo rHDrive;
    Servo omniPusher;

    double dropHeight = 0.43;


    public Decoder vuforia;

    public  boolean isJewelRed() {
        if (colorSensor.red() > colorSensor.blue()) {
            return true;
        }
        return false;
    }

    public  boolean isLineBlue() {
        if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green() && colorSensor.blue() > colorSensor.alpha()) {
            return true;
        }
        return false;
    }
    public int getDecodedMessage() {
        return 0;
    }
    public double getTurnError(double target, double current) {
        double o = target - current;
        return (((o + 180)) % 360) - 180;
    }

    public void turnToPitch (double targetPitch) {
        double currentPitch = IMU.getPitch();
        double error = getTurnError(targetPitch, currentPitch);

        while(Math.abs(error) > .5 && opModeIsActive()) {
            currentPitch = IMU.getHeading();
            error = -getTurnError(targetPitch, currentPitch);
            left.setPower(error * 0.001);
            right.setPower(error * 0.001);
            telemetry.addData("Degree: ", currentPitch);
            telemetry.addData("Error: ", error);
            telemetry.update();
        }
        left.setPower(.7);
        right.setPower(-.7);
        sleep(2000);
        left.setPower(0);
        right.setPower(0);
    }
    public void goTurn (double anglesToGo, double power, boolean liftCWheel) {
        double startHeading = 0;
        double targetHeading = anglesToGo;
        double currentHeading = IMU.getPitch();

        if (liftCWheel == true) {
            lHDrive.setPosition(0.6);
            rHDrive.setPosition(0.4);
            sleep(500);
        }
        /*if (anglesToGo < 0) {
            power = -power;
            anglesToGo = Math.abs(anglesToGo);
        }*/
        startHeading = IMU.getPitch();
        while ((Math.abs(currentHeading - startHeading) < Math.abs(targetHeading + startHeading)) && opModeIsActive()) {
            currentHeading = IMU.getPitch();
            if (targetHeading < 0) {
                left.setPower(power);
                right.setPower(power);
            } else if (targetHeading > 0) {
                right.setPower(-power);
                left.setPower(-power);
            }
        }
        left.setPower(0);
        right.setPower(0);
    }

    public void goDistance (double distanceToGo, double power, boolean liftCWheel) {


        int startPositionLt = 0;
        int startPositionRt = 0;

        if(liftCWheel == true) {
            lHDrive.setPosition(0.6);
            rHDrive.setPosition(0.4);
            sleep (500);
        }

        if(distanceToGo < 0) {
            power = -power;
            distanceToGo = Math.abs(distanceToGo);
        }

        startPositionLt = left.getCurrentPosition();
        startPositionRt = right.getCurrentPosition();
        int targetDistance = ((int) ((distanceToGo / (4 * Math.PI) * 1120))/2);

        double currentPositionLt = left.getCurrentPosition();
        double currentPositionRt = right.getCurrentPosition();

        while ((Math.abs(currentPositionLt - startPositionLt) < Math.abs(targetDistance)) && opModeIsActive()) {

            telemetry.addData("Current pos L: ", currentPositionLt);
            telemetry.addData("target pos L: ", targetDistance);
            telemetry.addData("error pos: L", Math.abs(targetDistance - currentPositionLt));
            telemetry.update();
            telemetry.addData("Current pos: ", currentPositionRt);
            telemetry.addData("target pos: ", targetDistance);
            telemetry.addData("error pos: ", Math.abs(targetDistance - currentPositionRt));
            telemetry.update();
            currentPositionLt = left.getCurrentPosition();
            currentPositionRt = right.getCurrentPosition();

            if (currentPositionLt > currentPositionRt) {
                left.setPower(-power * .9);
                right.setPower(power);
            } else if (currentPositionRt > currentPositionLt) {
                left.setPower(-power);
                right.setPower(power * .9);
            }
            else {
                left.setPower(-power);
                right.setPower(power);
            }

        }

        left.setPower(0);
        right.setPower(0);
        sleep(50);
        //again power if doesn't work
    }
    public void goDistanceCenter (double distanceToGoC, double power) {
        int startPositionC = 0;
        rHDrive.setPosition(0.5 + dropHeight);
        lHDrive.setPosition(0.5 - dropHeight);
        sleep(500);
        if (distanceToGoC < 0) {
            power = -power;
            distanceToGoC = Math.abs(distanceToGoC);
        }
        startPositionC = center.getCurrentPosition();
        int targetDistance = ((int) ((distanceToGoC / (4 * Math.PI) * 1120))/2);

        double currentPositionC = center.getCurrentPosition();

        while ((Math.abs(currentPositionC - startPositionC) < Math.abs(targetDistance)) && opModeIsActive()) {
            center.setPower(power);
            if(power > 0 ){
                left.setPower(0.3);
            } else {
                right.setPower(0.3);
            }
            telemetry.addData("Current pos C: ", currentPositionC);
            telemetry.addData("target pos C: ", targetDistance);
            telemetry.addData("error pos: C", Math.abs(targetDistance - currentPositionC));
            telemetry.update();

            currentPositionC = center.getCurrentPosition();
        }
            center.setPower(0);
            sleep(50);
        }
    public int Vuforia () {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        // Set up our telemetry dashboard

        // Wait until we're told to go

        // Start the logging of measured acceleration



        // Wait for the game to start (driver presses PLAY)
        vuforia.start();
        sleep (1000);

        // run until the end of the match (driver presses STOP)



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Target: " + vuforia.getDecodedColumn());
            telemetry.addData("Status", "Name: " + vuforia.getMark().name());
            telemetry.update();
            sleep (1500);
            int decodedColumn = vuforia.getDecodedColumn();
//            vuforia.stop();




        return decodedColumn;



    }
        /*vuforia = new Decoder(hardwareMap, "AYx3Kw3/////AAAAGQreNEJhLkdWqUbBsQ06dnWIksoccLxh/R9WNkXB8hvuonWmFXUWJ2tYqM+8VqYCWXkHfanXzG/G1un7ZvwgGkkO6u0ktevZDb8AFWF2/Y4wVH1BWGQ2psV5QkHAKZ7Z6ThZI01HPZqixiQowyeUstv7W/QU8jJ48NrqGBLVYdE6eFfzNDzVY/1IvrBJaRwqKR8vo+3a2zmeFEnEhFTqMI7anU2WSPy8RP7tR61CdfidjL2biMe0RiSOBIbqOe4rs9NGaDvp1Crtz17uyY71GyMkp+Kmjbejyfj8LgZ/dZQoEsuVuQyo0dbd4KBxsEJlQj/uAEst22QoEwZe0Af4DnFtwn6/IEe02L3DT3/Np+ZX");
        RelicRecoveryVuMark vuMark = vuforia.getMark();
        //if that doesn't work uncoment that out and put back in decoded column
        vuforia.start();
        return vuforia.getDecodedColumn();*/

    public void SuperAuto (boolean blue, boolean left, boolean glyph, boolean threeGlyphs, boolean hDrive, boolean jewels) {
        int direction;
        if (blue) {
            direction = 1;
        } else {
            direction = -1;
        }
            double initJewelSwiperPos = jewelSwiper.getPosition();
            int glyphPosition = -1;
            //jewelSwiper.setPosition(0.48);
            jewel2.setPosition(0.5);
            lHDrive.setPosition(0.5);
            rHDrive.setPosition(0.5);
            vuforia = new Decoder(hardwareMap, "AYx3Kw3/////AAAAGQreNEJhLkdWqUbBsQ06dnWIksoccLxh/R9WNkXB8hvuonWmFXUWJ2tYqM+8VqYCWXkHfanXzG/G1un7ZvwgGkkO6u0ktevZDb8AFWF2/Y4wVH1BWGQ2psV5QkHAKZ7Z6ThZI01HPZqixiQowyeUstv7W/QU8jJ48NrqGBLVYdE6eFfzNDzVY/1IvrBJaRwqKR8vo+3a2zmeFEnEhFTqMI7anU2WSPy8RP7tR61CdfidjL2biMe0RiSOBIbqOe4rs9NGaDvp1Crtz17uyY71GyMkp+Kmjbejyfj8LgZ/dZQoEsuVuQyo0dbd4KBxsEJlQj/uAEst22QoEwZe0Af4DnFtwn6/IEe02L3DT3/Np+ZX");
            waitForStart();
                double jewelSwiperCurrentPos = jewelSwiper.getPosition();
                int vuforiaColumn = Vuforia();
                telemetry.addData("Vuforia Column: ", Vuforia());
                telemetry.addData("Vuforia Column Saved", vuforiaColumn);
                telemetry.update();
                if (jewels) {
                    jewelSwiper.setPosition(.43);
                    sleep(1500);

                    if (isJewelRed() == true) {
                        if (blue) {
                            jewel2.setPosition(1);
                        } else {
                            jewel2.setPosition(0);
                        }
                        sleep(2000);
                        jewelSwiper.setPosition(1);

                    } else if (isJewelRed() == false) {
                        if (blue) {
                            jewel2.setPosition(0);
                        } else {
                            jewel2.setPosition(1);
                        }
                        sleep(2000);
                        jewelSwiper.setPosition(1);
                    }
                    sleep(1000);
                }
                if (glyph) {
                    if (!hDrive) {
                       if (vuforiaColumn == 1) {
                           goDistance(25, .7 * direction , true);
                       }
                       else if (vuforiaColumn == 2) {
                           goDistance(33, .7 * direction, true);
                       }
                       else if (vuforiaColumn == 3) {
                           goDistance(41, .7 * direction, true);
                       }
                        goTurn(85, .7 * direction, true);
                        goDistance(10, .7, true);
                        omniPusher.setPosition(1);
                        sleep(2000);
                        omniPusher.setPosition(0);
                        sleep(2000);
                        goDistance(10, -.5, true);
                        stop();
                    }
                    else {
                        if (blue && left || !blue && !left) {
                            goDistance(30, .9 * direction, true);
                        }
                        if (blue && !left || !blue && left) {
                            goDistance(15, .9 * direction, true);
                            goTurn(90, -0.7, true);
                        }
                        if (vuforiaColumn == 1) {
                            goDistanceCenter(4, .5 * direction);
                        } else if (vuforiaColumn == 2) {
                            goDistanceCenter(8, .5 * direction);
                        } else if (vuforiaColumn == 3) {
                            goDistanceCenter(12, .5 * direction);
                        }
                        omniPusher.setPosition(1);
                        sleep(1000);
                        omniPusher.setPosition(0);
                        goDistance(15, -.6, true);
                    }
                } else {
                    stop();
                }
                if (threeGlyphs) {
                    if (blue && !left || !blue && left) {
                        goDistance(15, -.7, true);
                        goTurn(180, .6, true);
                        goDistance(20, .7, true);
                        servoCollectorRt.setPower(.9);
                        servoCollectorLt.setPower(.9);
                        goDistance(5, .7, true);
                        sleep(3000);
                        servoCollectorRt.setPower(0);
                        servoCollectorLt.setPower(0);
                        omnipulatorLt.setPower(.9);
                        omnipulatorRt.setPower(.9);
                        sleep(3000);
                        omnipulatorLt.setPower(0);
                        omnipulatorRt.setPower(0);
                        goDistance(5, .7, true);
                        servoCollectorRt.setPower(.9);
                        servoCollectorLt.setPower(.9);
                        sleep(3000);
                        servoCollectorRt.setPower(0);
                        servoCollectorLt.setPower(0);
                        omnipulatorLt.setPower(.9);
                        omnipulatorRt.setPower(.9);
                        sleep(3000);
                        omnipulatorLt.setPower(0);
                        omnipulatorRt.setPower(0);
                    } else {
                        stop();
                    }
                }


                jewelSwiperCurrentPos = jewelSwiper.getPosition();




    }

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.dcMotor.get("L");
        right = hardwareMap.dcMotor.get("R");
        center = hardwareMap.dcMotor.get("C");

        servoCollectorLt = hardwareMap.dcMotor.get("LtCollector");
        servoCollectorRt = hardwareMap.dcMotor.get("RtCollector");
        omnipulatorRt = hardwareMap.crservo.get("omnipulatorRt");
        omnipulatorLt = hardwareMap.crservo.get("omnipulatorLt");

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        center.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        center = hardwareMap.dcMotor.get("C");
        lHDrive = hardwareMap.servo.get("LH");
        rHDrive = hardwareMap.servo.get("RH");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color");
        colorSensor.setI2cAddress(I2cAddr.create7bit(0x39));
        IMU = new IMU();
        jewelSwiper = hardwareMap.servo.get("jewelSwiper");
        jewel2 = hardwareMap.servo.get("jewel2");
        omniPusher = hardwareMap.servo.get("pusher");

        IMU.setup(hardwareMap);



        IMU.start();

        runAutoMode();
    }


    abstract void runAutoMode();

}
