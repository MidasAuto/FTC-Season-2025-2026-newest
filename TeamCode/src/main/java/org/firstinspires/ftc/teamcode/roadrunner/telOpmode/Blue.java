package org.firstinspires.ftc.teamcode.roadrunner.telOpmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.extraCode.ShootAllThree;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;


@TeleOp(name = "Blue")
public class Blue extends OpMode {

    DcMotor frontRightMotor, intakeMoter, backLeftMotor, backRightMotor, frontLeftMotor;
    DcMotorEx launch1, launch2, sorterMotor;
    ColorSensor checkColorSensor;
    Servo launchServo, ballGate, locker, aimLight;

    ShootAllThree shootThree = new ShootAllThree();

    //----------------------

    double redValue = 1, blueValue = 1, greenValue = 1, alphaValue = 1;
    boolean greenTrue = false, purpleTrue = false;
    boolean reset = false;
    /// Start position
    Pose2d currPos = new Pose2d(0, 0, Math.toRadians(90));
    ElapsedTime loopTimer = new ElapsedTime();
    //Ball Positions
    List<Integer> holderOne = new ArrayList<>(Arrays.asList(0, 0)), holderTwo = new ArrayList<>(Arrays.asList(2, 0)), holderThree = new ArrayList<>(Arrays.asList(4, 0));
    //timer
    int target_value = 0;
    double distance = 49;
    double spinRate = 300;
    double targetx = -64;
    double targety = 96;
    PinpointDrive drive;
    Pose2d relocate = new Pose2d(-14, -7, 0);
    boolean shootPos = false, rightBumberPreveous = false, leftBumperPreveous = false, sorterMoving = false, shoot = false;

    @Override
    public void init() {
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        sorterMotor = hardwareMap.get(DcMotorEx.class, "spedMotor");
        intakeMoter = hardwareMap.get(DcMotor.class, "intakeMotor");
        launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
        launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
        //servos
        launchServo = hardwareMap.get(Servo.class, "launchServo");
        ballGate = hardwareMap.get(Servo.class, "ballGate");
        locker = hardwareMap.get(Servo.class, "locker");
        aimLight = hardwareMap.get(Servo.class, "aimLight");
        //Color sensor
        checkColorSensor = hardwareMap.get(ColorSensor.class, "checkColorSensor");

        // Brake so robot stops instead of coasting
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch2.setDirection(DcMotorSimple.Direction.REVERSE);


        ///sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        target_value = sorterMotor.getCurrentPosition();
        drive = new PinpointDrive(hardwareMap, currPos);

    }

    @Override
    public void loop() {
        loopTimer.reset();
        drive.getPinpoint().update();
        boolean leftBumperCurrent = gamepad2.left_bumper;
        boolean rightBumperCurrent = gamepad2.right_bumper;

        double verticle = -gamepad1.left_stick_y; // forward/back
        double strafe = -gamepad1.left_stick_x;  // left/right
        double turn = -gamepad1.right_stick_x;   // rotation

        double fRightPower = verticle + turn + strafe;
        double fLeftPower = (verticle - turn - strafe);
        double bRightPower = verticle + turn - strafe;
        double bLeftPower = (verticle - turn + strafe);
        /*
        double max = Math.max(1.0,
                Math.max(Math.abs(fRightPower),
                        Math.max(Math.abs(fLeftPower),
                                Math.max(Math.abs(bRightPower), Math.abs(bLeftPower)))));

        fRightPower /= max;
        fLeftPower /= max;
        bRightPower /= max;
        bLeftPower /= max;

         */

        // apply power
        frontRightMotor.setPower(fRightPower);
        frontLeftMotor.setPower(fLeftPower);
        backRightMotor.setPower(bRightPower);
        backLeftMotor.setPower(bLeftPower);
        ballGate.setPosition(0.8);

        distance = testPos();

        //Read color sensor value
        if (checkForBall() && !sorterMoving && !shootPos) {
            checkColor();
        }

        //----------------------

        if (leftBumperCurrent && !leftBumperPreveous) {
            target_value -= 90;
            sorterMoving = true;
            shootPos = !shootPos;
            move_slots(-1);
        }

        //----------------------

        if (rightBumperCurrent && !rightBumberPreveous) {
            target_value += 90;
            sorterMoving = true;
            shootPos = !shootPos;
            move_slots(1);
        }

        //----------------------

        if (gamepad1.left_trigger > 0) {
            intakeMoter.setPower(1);
        } else if (gamepad1.right_trigger > 0) {
            intakeMoter.setPower(-1);
        } else {
            intakeMoter.setPower(0);
        }

        //----------------------
        //628 radians is 6000 rpm for the 6000 rpm motor
        //----------------------
        /// Spinrate Adjustment

        if (gamepad2.right_trigger > 0) {
            ballGate.setPosition(0.8);
            distance = testPos();
            spinRate = (0.748 * distance + 159.87); // convert deg/s → rad/s
            if (distance > 95) {

                ///Change this for long distance

                spinRate += 6;
            }
            if (launch2.getVelocity(AngleUnit.DEGREES) > spinRate - 7) {
                launchServo.setPosition(.55);
            }

            launch1.setVelocity(spinRate, AngleUnit.DEGREES);
            launch2.setVelocity(spinRate, AngleUnit.DEGREES);

        } else if (gamepad2.left_trigger > 0) {

            launch1.setVelocity(-180, AngleUnit.DEGREES);
            launch2.setVelocity(-180, AngleUnit.DEGREES);

        } else {
            launch1.setPower(0);
            launch2.setPower(0);
            ballGate.setPosition(1);
        }

        /// Shoot all three balls in quick succsession

        if (gamepad2.dpad_up && gamepad2.right_trigger > 0.1) {
            shootThree.shootAllThree(launch1, launch2, launchServo, locker, spinRate, sorterMotor);
        }

        //----------------------

        if (gamepad1.dpad_left && gamepad1.right_trigger > 0.2) {
            drive.pinpoint.setPosition(relocate);
        }

        //Launch Servo

        if (launch1.getVelocity(AngleUnit.DEGREES) > (spinRate - 3d) || gamepad2.y) {
            launchServo.setPosition(.55);
            if (holderOne.get(0) == 2) {
                holderOne.set(1, 0);
            }
            if (holderTwo.get(0) == 2) {
                holderTwo.set(1, 0);
            }
            if (holderThree.get(0) == 2) {
                holderThree.set(1, 0);
            }
        } else {
            launchServo.setPosition(.7);
        }

        //----------------------

        if (gamepad2.dpad_right || gamepad1.dpad_left) {
            target_value += 3;
        }
        if (gamepad2.dpad_left || gamepad1.dpad_left) {
            target_value -= 3;
        }

        if (gamepad2.dpad_down) {
            locker.setPosition(0.61);
        }

        if (sorterMoving) {
            locker.setPosition((0.70));
            moveSorter();
        }
        if (gamepad1.dpad_up) {
            spinRate += 2;
        } else if (gamepad1.dpad_down) {
            spinRate -= 2;
        }
        telemetry();
    }

    //----------------------

    public void getColor() {
        redValue = checkColorSensor.red();
        blueValue = checkColorSensor.blue();
        greenValue = checkColorSensor.green(

        );
        alphaValue = checkColorSensor.alpha();
    }

    //----------------------

    public void telemetry() {
        telemetry.addData("Shoot Velocity: ", launch2.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Distance", testPos());
        telemetry.addData("SpinRate", spinRate);
        telemetry.addData("x", drive.getPinpoint().getPositionRR().position.x);
        telemetry.addData("y", drive.getPinpoint().getPositionRR().position.y);
        telemetry.addData("Heading", Math.toDegrees(drive.pinpoint.getHeading()));

        telemetry.update();
    }

    //----------------------

    public void checkColor() {
        getColor();// Get values

        if (blueValue < greenValue) {
            greenTrue = true;
            purpleTrue = false;
        }

        if (greenValue < blueValue) {
            purpleTrue = true;
            greenTrue = false;
        }
        if (holderOne.get(0) == 0) {
            if (greenTrue && checkForBall()) {
                holderOne.set(1, 1);
            } else if (purpleTrue && checkForBall()) {
                holderOne.set(1, 2);
            }
        } else if (holderTwo.get(0) == 0) {
            if (greenTrue && checkForBall()) {
                holderTwo.set(1, 1);
            } else if (purpleTrue && checkForBall()) {
                holderTwo.set(1, 2);
            }
        } else if (holderThree.get(0) == 0) {
            if (greenTrue && checkForBall()) {
                holderThree.set(1, 1);
            } else if (purpleTrue && checkForBall()) {
                holderThree.set(1, 2);
            }
            greenTrue = false;
            purpleTrue = false;
        }
    }

    //----------------------

    public boolean checkForBall() {
        ;
        int blueV = checkColorSensor.blue();


        return blueV > 100 && !sorterMoving;
    }

    //----------------------

    public void moveSorter() {
        double currpos = sorterMotor.getCurrentPosition();

        if (currpos <= target_value - 7 || currpos >= target_value + 7) {
            if (currpos < target_value) {
                sorterMotor.setPower(.3);
            } else if (currpos > target_value) {
                sorterMotor.setPower(-0.3);
            }
        } else if (currpos >= target_value - 7 && currpos <= target_value + 7) {
            sorterMotor.setPower(0);
            sorterMoving = false;
        }
        if (currpos >= target_value - 35 && currpos <= target_value + 35) {
            locker.setPosition(.61);
        }
    }

    //----------------------

    public void checkPos() {
        int currpos = sorterMotor.getCurrentPosition();
        if (currpos <= target_value - 7 || currpos >= target_value + 7) {
            sorterMoving = true;
        }
    }

    //----------------------

    public void move_slots(int distance) {
        holderOne.set(0, holderOne.get(0) + distance);
        holderTwo.set(0, holderTwo.get(0) + distance);
        holderThree.set(0, holderThree.get(0) + distance);

        if (holderOne.get(0) > 5) {
            holderOne.set(0, 0);
        } else if (holderOne.get(0) < 0) {
            holderOne.set(0, 5);
        }
        if (holderTwo.get(0) > 5) {
            holderTwo.set(0, 0);
        } else if (holderTwo.get(0) < 0) {
            holderTwo.set(0, 5);
        }
        if (holderThree.get(0) > 5) {
            holderThree.set(0, 0);
        } else if (holderThree.get(0) < 0) {
            holderThree.set(0, 5);
        }
    }

    //----------------------

    public double testPos() {
        Pose2d newPos = drive.getPose();
        double x = newPos.position.x;
        double y = newPos.position.y;

        double distance1 = targetx - x;
        double distance2 = targety - y;

        double length = Math.sqrt((distance1 * distance1) + (distance2 * distance2));

        return length;
    }
    public void light() {
        Pose2d newPos = drive.getPose();
        double heading = Math.toDegrees(newPos.heading.toDouble()) - 180;
        double x = newPos.position.x;
        double y = newPos.position.y;

        double distance1 = targetx - x;
        double distance2 = targety - y;

        double angleRad = Math.atan(distance2 / distance1);
        double angleDeg = Math.toDegrees(angleRad);

        if (angleDeg >= heading-3 && angleDeg <= heading+3) {
            aimLight.setPosition(.5);
        }
        else {
            aimLight.setPosition(.3);
        }
    }
}