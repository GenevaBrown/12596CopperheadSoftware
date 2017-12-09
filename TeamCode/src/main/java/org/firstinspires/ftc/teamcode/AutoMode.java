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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    //Servo lHDrive;
    //Servo rHDrive;
    CRServo omniPusher;
    CRServo omniPusher2;
    DcMotor lift;

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
        double startHeading = IMU.getHeading();
        double targetDiff = anglesToGo;
        double currentHeading = IMU.getHeading();
        double previousHeading = startHeading;

        /*if (liftCWheel == true) {
            lHDrive.setPosition(0.6);
            rHDrive.setPosition(0.4);
            sleep(500);
        }*/
        /*if (anglesToGo < 0) {
            power = -power;
            anglesToGo = Math.abs(anglesToGo);
        }*/
        double jumpAngle = 0;
        while ((Math.abs(currentHeading - startHeading) < Math.abs(targetDiff)) && opModeIsActive()) {
            currentHeading = IMU.getHeading() + jumpAngle;
            if (currentHeading > 180 + previousHeading) {
                jumpAngle = -360;
                currentHeading = currentHeading + jumpAngle;
            }
            if (currentHeading < previousHeading - 180) {
                jumpAngle = 360;
                currentHeading = currentHeading + jumpAngle;
            }
            if (targetDiff < 0) {
                left.setPower(power);
                right.setPower(power);
            } else if (targetDiff > 0) {
                right.setPower(-power);
                left.setPower(-power);
            }
            telemetry.addData("Current Heading: ", currentHeading);
            telemetry.addData("Previous Heading: ", previousHeading);
            telemetry.addData("Target Diff: ", targetDiff);
            telemetry.addData("Start Heading: ", startHeading);
            telemetry.addData("Jump Angle: ", jumpAngle);
            telemetry.update();
            //Repeat at start
            //To slow down before the end of turns -- Needs to be tested!!
           /*if ((Math.abs(targetDiff) - Math.abs((currentHeading - startHeading))) < 15) {
                double slowDownPower = (power * ((Math.abs(targetDiff) - Math.abs((currentHeading - startHeading))) / 15));
                if (power > 0) {
                    slowDownPower = slowDownPower + 0.2;
                }
                if (power < 0) {
                    slowDownPower = slowDownPower - 0.2;
                }
                left.setPower(slowDownPower);
                right.setPower(slowDownPower);
                if (Math.abs(slowDownPower) > Math.abs(power)) {
                    left.setPower(power);
                    right.setPower(power);
                }
            }*/
            previousHeading = currentHeading;
        }
        left.setPower(0);
        right.setPower(0);
    }

    public void goDistance (double distanceToGo, double power, boolean liftCWheel, double timeout) {


        int startPositionLt = 0;
        //int startPositionRt = 0;

        if(liftCWheel == true) {
            //lHDrive.setPosition(0.6);
            //rHDrive.setPosition(0.4);
            sleep (500);
        }

        if (distanceToGo < 0) {
            power = -power;
            distanceToGo = Math.abs(distanceToGo);
        }

        startPositionLt = left.getCurrentPosition();
        //startPositionRt = right.getCurrentPosition();
        int targetDistance = ((int) ((distanceToGo / (4 * Math.PI) * 1120))/2);
        ElapsedTime Timer = new ElapsedTime();
        double currentPositionLt = left.getCurrentPosition();
        //double currentPositionRt = right.getCurrentPosition();
        while ((Math.abs(currentPositionLt - startPositionLt) < Math.abs(targetDistance)) && opModeIsActive()) {
            telemetry.addData("Current pos L: ", currentPositionLt);
            telemetry.addData("target pos L: ", targetDistance);
            telemetry.addData("error pos: L", Math.abs(targetDistance - currentPositionLt));
            telemetry.update();
            //telemetry.addData("Current pos: ", currentPositionRt);
            //telemetry.addData("target pos: ", targetDistance);
            //telemetry.addData("error pos: ", Math.abs(targetDistance - currentPositionRt));
            telemetry.update();
            currentPositionLt = left.getCurrentPosition();
            //currentPositionRt = right.getCurrentPosition();

            /*if (currentPositionLt > currentPositionRt) {
                left.setPower(-power * .9);
                right.setPower(power);
            } else if (currentPositionRt > currentPositionLt) {
                left.setPower(-power);
                right.setPower(power * .9);
            }
            else {
                left.setPower(-power);
                right.setPower(power);
            }*/
            if (Timer.seconds() > timeout) {
                left.setPower(0);
                right.setPower(0);
                break;
            }

        }

        left.setPower(0);
        right.setPower(0);
        sleep(50);
        //again power if doesn't work
    }
    public void goDistanceCenter (double distanceToGoC, double power) {
        int startPositionC = 0;
        //rHDrive.setPosition(0.5 + dropHeight);
        //HDrive.setPosition(0.5 - dropHeight);
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
        sleep (500);

        // run until the end of the match (driver presses STOP)



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Target: " + vuforia.getDecodedColumn());
            telemetry.addData("Status", "Name: " + vuforia.getMark().name());
            telemetry.update();
            sleep (1000);
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
            jewelSwiper.setPosition(0);
            jewel2.setPosition(0.6);
            //lHDrive.setPosition(0.5);
            //rHDrive.setPosition(0.5);
            vuforia = new Decoder(hardwareMap, "AYx3Kw3/////AAAAGQreNEJhLkdWqUbBsQ06dnWIksoccLxh/R9WNkXB8hvuonWmFXUWJ2tYqM+8VqYCWXkHfanXzG/G1un7ZvwgGkkO6u0ktevZDb8AFWF2/Y4wVH1BWGQ2psV5QkHAKZ7Z6ThZI01HPZqixiQowyeUstv7W/QU8jJ48NrqGBLVYdE6eFfzNDzVY/1IvrBJaRwqKR8vo+3a2zmeFEnEhFTqMI7anU2WSPy8RP7tR61CdfidjL2biMe0RiSOBIbqOe4rs9NGaDvp1Crtz17uyY71GyMkp+Kmjbejyfj8LgZ/dZQoEsuVuQyo0dbd4KBxsEJlQj/uAEst22QoEwZe0Af4DnFtwn6/IEe02L3DT3/Np+ZX");
            waitForStart();
                double jewelSwiperCurrentPos = jewelSwiper.getPosition();
                int vuforiaColumn = Vuforia();
                telemetry.addData("Vuforia Column: ", Vuforia());
                telemetry.addData("Vuforia Column Saved", vuforiaColumn);
                telemetry.update();
                if (jewels) {
                    jewelSwiper.setPosition(.4);
                    sleep(2000);

                    if (isJewelRed() == true) {
                        if (blue) {
                            jewel2.setPosition(1);
                        } else {
                            jewel2.setPosition(0);
                        }
                        sleep(2000);
                        jewelSwiper.setPosition(0);

                    } else if (isJewelRed() == false) {
                        if (blue) {
                            jewel2.setPosition(0);
                        } else {
                            jewel2.setPosition(1);
                        }
                        sleep(2000);
                        jewelSwiper.setPosition(0);
                    }
                    sleep(1000);
                }
                //WHEN H-DRIVE IS BACK(if that ever happens) YOU NEED TO ADD BACK IN THE TRUE'S FOR GO DISTANCE AND GO TURN
                if (glyph) {
                    //omnipulatorRt.setPower(0.6);
                    //omnipulatorLt.setPower(0.6);
                    if (!hDrive) {
                       if (vuforiaColumn == 1) {
                           if (!blue) {
                               goDistance(29, .7 * direction, false, 20);
                           }
                           else if (blue) {
                               goDistance(50, .7, false, 20);
                           }
                       }
                       else if (vuforiaColumn == 2) {
                           if (!blue) {
                               goDistance(21, .7 * direction, false, 20);
                           }
                           else if (blue) {
                               goDistance(54, .7, false, 20);
                           //changed
                           }
                       }
                       else if (vuforiaColumn == 3) {
                           if (!blue) {
                               goDistance(27, .7 * direction, false, 20);
                           }
                           else if (blue) {
                               goDistance(29, .7, false, 20);
                           }
                       }
                       else if (vuforiaColumn == -1) {
                           if (!blue) {
                               goDistance(29, .7 * direction, false, 20);
                           }
                           else if (blue) {
                               goDistance(29, .7, false, 20);
                           }
                       }
                        sleep(1000);
                        if (!blue) {
                            if (vuforiaColumn == 3) {
                                goTurn(67, -.7, false);
                            }
                            else {
                                goTurn(98, -.7, false);
                            }
                        }
                        if (blue) {
                            if (vuforiaColumn == 3) {
                                goTurn(43, -.7, false);
                                telemetry.addData("turn", "");
                            }
                            else if (vuforiaColumn == 1) {
                                goTurn(105, -.7, false);
                            }
                            else if (vuforiaColumn == 2) {
                                goTurn(95, -.7, false);
                            }
                            else {
                                goTurn(43, -.7, false);
                            }
                        }
                        sleep(1000);
                        if (blue) {
                            goDistance(18, .7, false, 5);
                        }
                        if (!blue) {
                            if (vuforiaColumn == 3) {
                                goDistance(12, 0.7, false, 4);
                            }
                            else {
                                goDistance(15, .7, false, 5);
                            }
                        }
                        /*lift.setPower(.4);
                        sleep(1500);
                        lift.setPower(-.4);
                        sleep(1000);
                        lift.setPower(0); */
                        //omnipulatorRt.setP0ower(0);
                        //omnipulatorLt.setPower(0);
                        omniPusher2.setPower(.8);
                        goTurn(5, -.5, false);
                        goTurn(3, .5, false);
                        sleep(3500);
                        omniPusher2.setPower(0);
                        goDistance(4, -.5, false, 8);
                        servoCollectorRt.setPower(.7);
                        servoCollectorLt.setPower(-.7);
                        sleep(1500);
                        servoCollectorLt.setPower(0);
                        servoCollectorRt.setPower(0);
                        goDistance(3, .5, false, 3);
                        goDistance(5, -.5, false, 8);
                        stop();
                    }
                    else {
                        /*if (blue && left || !blue && !left) {
                            goDistance(30, .9 * direction, false);
                        }
                        if (blue && !left || !blue && left) {
                            goDistance(15, .9 * direction, false);
                            goTurn(90, -0.7, false);
                        }
                        if (vuforiaColumn == 1) {
                            goDistanceCenter(4, .5 * direction);
                        } else if (vuforiaColumn == 2) {
                            goDistanceCenter(8, .5 * direction);
                        } else if (vuforiaColumn == 3) {
                            goDistanceCenter(12, .5 * direction);
                        }
                        omniPusher2.setPower(.8);
                        sleep(1500);
                        omniPusher2.setPower(-.6);
                        sleep(1500);
                        goDistance(15, -.6, true);
                        */
                    }
                } else {
                    stop();
                }
                if (threeGlyphs) {
                    if (blue && !left || !blue && left) {
                        goDistance(15, -.7, true, 20);
                        goTurn(180, .6, true);
                        goDistance(20, .7, true, 20);
                        servoCollectorRt.setPower(.9);
                        servoCollectorLt.setPower(.9);
                        goDistance(5, .7, true, 10);
                        sleep(3000);
                        servoCollectorRt.setPower(0);
                        servoCollectorLt.setPower(0);
                        omnipulatorLt.setPower(.9);
                        omnipulatorRt.setPower(.9);
                        sleep(3000);
                        omnipulatorLt.setPower(0);
                        omnipulatorRt.setPower(0);
                        goDistance(5, .7, true, 10);
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
        lift = hardwareMap.dcMotor.get("Lift");

        servoCollectorLt = hardwareMap.dcMotor.get("LtCollector");
        servoCollectorRt = hardwareMap.dcMotor.get("RtCollector");
        omnipulatorRt = hardwareMap.crservo.get("omnipulatorRt");
        omnipulatorLt = hardwareMap.crservo.get("omnipulatorLt");

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        center.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //left.setDirection(DcMotorSimple.Direction.REVERSE);
        //right.setDirection(DcMotorSimple.Direction.REVERSE);



        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        center = hardwareMap.dcMotor.get("C");
        //lHDrive = hardwareMap.servo.get("LH");
        //rHDrive = hardwareMap.servo.get("RH");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color");
        colorSensor.setI2cAddress(I2cAddr.create7bit(0x39));
        IMU = new IMU();
        jewelSwiper = hardwareMap.servo.get("jewelSwiper");
        jewel2 = hardwareMap.servo.get("jewel2");
        omniPusher2 = hardwareMap.crservo.get("pusher2");
        omniPusher = hardwareMap.crservo.get("pusher");


        IMU.setup(hardwareMap);



        IMU.start();

        runAutoMode();
    }


    abstract void runAutoMode();

}
