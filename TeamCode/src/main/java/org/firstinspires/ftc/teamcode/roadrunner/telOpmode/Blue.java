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

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;


@TeleOp(name = "Blue")
public class Blue extends OpMode {

    DcMotor frontRightMotor, intakeMoter, backLeftMotor, backRightMotor, frontLeftMotor;
    DcMotorEx launch1, launch2, sorterMotor;
    ColorSensor checkColorSensor;
    Servo launchServo, rampAngle, locker;

    //----------------------

    double redValue = 1, blueValue = 1, greenValue = 1, alphaValue = 1;
    boolean greenTrue = false, purpleTrue = false;
    Pose2d currPos = new Pose2d(9,-31 , 0);
    //Ball Positions
    List<Integer> holderOne = new ArrayList<>(Arrays.asList(0,0)), holderTwo = new ArrayList<>(Arrays.asList(2,0)), holderThree = new ArrayList<>(Arrays.asList(4,0));
    //timer
    int target_value = 0;
    double distance = 49;
    double spinRate = 1.038 * distance + 204.24;
    double targetx = 127;
    double targety = 57;
    PinpointDrive drive;
    Pose2d relocate = new Pose2d(62.5, 9, 0);
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
        rampAngle = hardwareMap.get(Servo.class, "rampAngle");
        locker = hardwareMap.get(Servo.class, "locker");
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
        boolean leftBumperCurrent = gamepad2.left_bumper;
        boolean rightBumperCurrent = gamepad2.right_bumper;
        // keep your original stick mapping
        double verticle = -gamepad1.left_stick_y; // forward/back
        double strafe = -gamepad1.left_stick_x;  // left/right
        double turn = -gamepad1.right_stick_x;   // rotation

        // motor power calc with left-side compensation
        double fRightPower = verticle + turn + strafe;
        double fLeftPower = (verticle - turn - strafe);
        double bRightPower = verticle + turn - strafe;
        double bLeftPower = (verticle - turn + strafe);

        // normalize so no value exceeds ±1
        double max = Math.max(1.0,
                Math.max(Math.abs(fRightPower),
                        Math.max(Math.abs(fLeftPower),
                                Math.max(Math.abs(bRightPower), Math.abs(bLeftPower)))));

        fRightPower /= max;
        fLeftPower /= max;
        bRightPower /= max;
        bLeftPower /= max;

        // apply power
        frontRightMotor.setPower(fRightPower);
        frontLeftMotor.setPower(fLeftPower);
        backRightMotor.setPower(bRightPower);
        backLeftMotor.setPower(bLeftPower);
        rampAngle.setPosition(0.5);

        distance = testPos();

        //Read color sensor value
        if (checkForBall() && !sorterMoving && !shootPos) {
            checkColor();
            target_value = autoMove(0,0);
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

        if (gamepad2.right_trigger > 0) {
            spinRate = (1.038 * distance + 204.24); // convert deg/s → rad/s
            if (distance < 115) {
                spinRate -= 20;
            } else {
                spinRate += 7;
            }
            if (launch2.getVelocity(AngleUnit.DEGREES) > spinRate-7) {
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
        }


        //----------------------

        if (gamepad1.dpad_left && gamepad1.right_trigger > 0.2) {
            drive.pinpoint.setPosition(relocate);
        }

        //Launch Servo

        if (launch1.getVelocity(AngleUnit.DEGREES) > (spinRate-3d) || gamepad2.y) {
            launchServo.setPosition(.55);
            if (holderOne.get(0) == 2) {
                holderOne.set(1,0);
            }
            if (holderTwo.get(0) == 2) {
                holderTwo.set(1,0);
            }
            if (holderThree.get(0) == 2) {
                holderThree.set(1,0);
            }
        } else {
            launchServo.setPosition(.7);
        }

        //----------------------

        if (gamepad2.dpad_right) {
            target_value += 3;
        }
        if (gamepad2.dpad_left) {
            target_value -= 3;
        }
        if (gamepad1.right_bumper) {
            target_value = autoMove(2, 3);
        }
        if (gamepad1.left_bumper) {
            target_value = autoMove(1, 3);
        }
        if (gamepad2.dpad_down) {
            locker.setPosition(0.61);
        }
        if (gamepad2.dpad_up) {
            autoShoot();
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
        telemetry.update();
        leftBumperPreveous = leftBumperCurrent;
        rightBumberPreveous = rightBumperCurrent;
        checkPos();

    }

    //----------------------

    public void getColor() {
        redValue = checkColorSensor.red();
        blueValue = checkColorSensor.blue();
        greenValue = checkColorSensor.green();
        alphaValue = checkColorSensor.alpha();
    }

    //----------------------

    public void telemetry() {
        telemetry.addData("Red: ", redValue);
        telemetry.addData("Green: ", greenValue);
        telemetry.addData("Blue: ", blueValue);
        telemetry.addData("CheckForBall", checkForBall());
        telemetry.addData("holderOne value: ", holderOne.get(1));
        telemetry.addData("holderTwo value: ", holderTwo.get(1));
        telemetry.addData("Should Shoot: ", shoot);
        telemetry.addData("Shoot Velocity: ", launch2.getVelocity(AngleUnit.DEGREES));

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
            } else if(purpleTrue && checkForBall()){
                holderOne.set(1, 2);
            }
        } else if (holderTwo.get(0) == 0) {
            if (greenTrue && checkForBall()) {
                holderTwo.set(1, 1);
            } else if (purpleTrue && checkForBall()){
                holderTwo.set(1, 2);
            }
        } else if (holderThree.get(0) == 0) {
            if (greenTrue && checkForBall()) {
                holderThree.set(1, 1);
            } else if(purpleTrue && checkForBall()){
                holderThree.set(1, 2);
            }
            greenTrue = false;
            purpleTrue = false;
        }
    }

    //----------------------

    public boolean checkForBall() {;
        int blueV = checkColorSensor.blue();


        return blueV > 100 && !sorterMoving;
    }

    //----------------------

    public void moveSorter() {
        double currpos = sorterMotor.getCurrentPosition();

        if (currpos <= target_value-7 || currpos >= target_value+7) {
            if (currpos < target_value) {
                sorterMotor.setPower(.3);
            }
            else if (currpos > target_value) {
                sorterMotor.setPower(-0.3);
            }
        }
        else if (currpos >= target_value-7 && currpos <= target_value+7) {
            sorterMotor.setPower(0);
            sorterMoving = false;
        }
        if (currpos >= target_value-35 && currpos <= target_value+35) {
            locker.setPosition(.61);
        }
    }

    //----------------------

    public void checkPos() {
        int currpos = sorterMotor.getCurrentPosition();
        if (currpos <= target_value-7 || currpos >= target_value+7) {
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
        }
        else if (holderOne.get(0) < 0) {
            holderOne.set(0, 5);
        }
        if (holderTwo.get(0) > 5) {
            holderTwo.set(0, 0);
        }
        else if (holderTwo.get(0) < 0) {
            holderTwo.set(0, 5);
        }
        if (holderThree.get(0) > 5) {
            holderThree.set(0, 0);
        }
        else if (holderThree.get(0) < 0) {
            holderThree.set(0, 5);
        }
    }

    //----------------------

    public int autoMove(int color, int target) {
        int distance = 0;
        int hOnePos = holderOne.get(0);
        int hTwoPos = holderTwo.get(0);
        int hThreePos = holderThree.get(0);

        if (holderOne.get(1) == color) {
            if (hOnePos == 0) {
                distance += target - hOnePos;
            }
            else if (hOnePos == 1) {
                distance += target - hOnePos;
            }
            else if (hOnePos == 2) {
                distance += target - hOnePos;
            }
            else if (hOnePos == 3) {
                distance += target - hOnePos;
            }
            else if (hOnePos == 4) {
                distance += target - hOnePos;
            }
            else if (hOnePos == 5) {
                distance += target - hOnePos;
            }
        }

        //----------------------

        else if (holderTwo.get(1) == color) {
            if (hTwoPos == 0) {
                distance += target - hTwoPos;
            }
            else if (hTwoPos == 1) {
                distance += target - hTwoPos;
            }
            else if (hTwoPos == 2) {
                distance += target - hTwoPos;
            }
            else if (hTwoPos == 3) {
                distance += target - hTwoPos;
            }
            else if (hTwoPos == 4) {
                distance += target - hTwoPos;
            }
            else if (hTwoPos == 5) {
                distance += target - hTwoPos;
            }
        }

        //----------------------

        else if (holderThree.get(1) == color) {
            if (hThreePos == 0) {
                distance += target - hThreePos;
            }
            else if (hThreePos == 1) {
                distance += target - hThreePos;
            }
            else if (hThreePos == 2) {
                distance += target - hThreePos;
            }
            else if (hThreePos == 3) {
                distance += target - hThreePos;
            }
            else if (hThreePos == 4) {
                distance += target - hThreePos;
            }
            else if (hThreePos == 5) {
                distance += target - hThreePos;
            }
        }

        move_slots(distance);
        return 90 * distance;
    }
    public void autoShoot() {
        launch2.setVelocity(628 , AngleUnit.RADIANS);
        if (launch2.getVelocity(AngleUnit.RADIANS) > 600) {
            launchServo.setPosition(.55);
        }
    }
    public double testPos() {
        Pose2d newPos = drive.getPose();
        double x = newPos.position.x;
        double y = newPos.position.y;

        double distance1 = targetx - x;
        double distance2 = targety - y;

        double length = Math.sqrt((distance1 * distance1) + (distance2 * distance2));

        return length;
    }
}