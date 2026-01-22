package org.firstinspires.ftc.teamcode.roadrunner.telOpmode;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.ArrayList;


@TeleOp(name = "AlexTest")
public class AlexTest extends OpMode {
    /// Calling
    DcMotor frontRightMotor, intakeMotor, backLeftMotor, backRightMotor, frontLeftMotor;
    DcMotorEx launch1, launch2, sorterMotor;
    ColorSensor checkColorSensor;
    Servo launchServo, ballGate, locker, aimLight;
    PinpointDrive drive;
    private HuskyLens huskyLens;
    Pose2d currPos = new Pose2d(0, 0, Math.toRadians(90));
    Pose2d blueTarget = new Pose2d(120, -56, 90);
    Pose2d redTarget = new Pose2d(120, 56, 90);
    /// 0 is the intake slot, 1 is the first slot clockwise from the intake, 2 is after that, this ignores half steps eg. shooter is slot 1
    List<Integer> sorter = new ArrayList<>(Arrays.asList(0,0,0));
    HuskyLens.Block[] blocks;
    double target_value, posX, posY, heading, distance, tangent, adjust;
    double tagID = 6, colorSortRelative;
    boolean sorterSafe, sorterThere, motorsSpooled, shooter;
    double servoShootPos = 0.55, servoStayPos = 0.7, colorMove, focalLength, distance1;



    @Override
    public void init() {
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        sorterMotor = hardwareMap.get(DcMotorEx.class, "spedMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
        launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
        launchServo = hardwareMap.get(Servo.class, "launchServo");
        ballGate = hardwareMap.get(Servo.class, "ballGate");
        locker = hardwareMap.get(Servo.class, "locker");
        aimLight = hardwareMap.get(Servo.class, "aimLight");
        checkColorSensor = hardwareMap.get(ColorSensor.class, "checkColorSensor");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");


        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch2.setDirection(DcMotorSimple.Direction.REVERSE);


        drive = new PinpointDrive(hardwareMap, currPos);
        target_value = sorterMotor.getCurrentPosition();
        colorSortRelative = sorterMotor.getCurrentPosition();

    }


    @Override
    public void loop() {
        blocks = huskyLens.blocks();
        drive.getPinpoint().update();
        peripherals();
        posMath();
        colorSort();

        double vert = -gamepad1.left_stick_y; // forward/back
        double strafe = -gamepad1.left_stick_x;  // left/right
        double turn = -gamepad1.right_stick_x;   // rotation

        double fRightPower = (vert + turn + strafe);
        double fLeftPower = (vert - turn - strafe);
        double bRightPower = (vert + turn - strafe);
        double bLeftPower = (vert - turn + strafe);

        frontRightMotor.setPower(fRightPower);
        frontLeftMotor.setPower(fLeftPower);
        backRightMotor.setPower(bRightPower);
        backLeftMotor.setPower(bLeftPower);

        if (gamepad2.left_bumper && sorterSafe) {
            target_value += 90;
            colorMove += 1;
        } else if (gamepad2.right_bumper && sorterSafe) {
            target_value -= 90;
            colorMove -= 1;
        }

        if (gamepad2.right_trigger > 0.1) {
            shooter(spinRateCalc());
            sorterMove(90, false);
        }

        telemetry.addData("Tag", tagID);
        telemetry.addData("FocalLength", focalLength);
        telemetry.addData("Distance", distance1);
        telemetry.update();
    }


    public void posMath() {

        /// Calculates all pos math, anything which uses it must have this called before it to update properly

        double xDiff, yDiff;
        posX = drive.getPinpoint().getPosX();
        posY = drive.getPinpoint().getPosY();
        heading = Math.toDegrees(drive.getPinpoint().getHeading());

        xDiff = blueTarget.position.x - posX;
        yDiff = blueTarget.position.y - posY;

        distance = Math.sqrt((xDiff * xDiff) + (yDiff * yDiff));

        tangent = Math.toDegrees(Math.atan2(yDiff, xDiff));
    }

    public boolean lookingTarget() {
        /// Tells you if you are looking at the target, will return true if so
        return tangent - 2 > heading && heading < tangent + 2;

    }

    public double spinRateCalc() {

        /// Calculates the spin rate of the launcher

        //Contains speed algorithm

        double spinRate;

        spinRate = distance * 0.7 + 172.5 + adjust;

        return spinRate;
    }

    public void shooter(double shootSpeed) {

        /// Handles all shooter actions

        //Sets the speed to the spin rate and sets a marker

        if (launch1.getVelocity() < shootSpeed) {
            launch1.setPower(1);
            launch2.setPower(1);
        } else if (launch1.getVelocity() >= shootSpeed){
            launch1.setPower(0.8);
        }

//            launch1.setVelocity(shootSpeed);
//            launch2.setVelocity(shootSpeed);
            shooter = true;
    }

    public boolean interfereCalc () {

        /// Calculates if it is safe to move the sorter

        return launchServo.getPosition() <= 0.57 && locker.getPosition() >= 0.69;
    }

    public void sorterMove(double TargetValue, boolean ShooterSafeOverride) {
        /// Responsible for all sorter movement

        //Parabolic tuning is a beta feature and currently not implemented

        double parabolicTuning, sorterDiff = target_value - sorterMotor.getCurrentPosition(), sorterErr;

        parabolicTuning = 0.1 - Math.cbrt(sorterDiff * sorterDiff);

        //The movement code, adjust the sorterErr by +/- if motor is not stopping in time or is going too far

        sorterErr = 7;

        target_value += TargetValue;

        if (target_value > sorterMotor.getCurrentPosition() + sorterErr && sorterSafe || ShooterSafeOverride) {
            sorterMotor.setPower(0.3);
            sorterThere = false;
        } else if (target_value < sorterMotor.getCurrentPosition() - sorterErr && sorterSafe || ShooterSafeOverride) {
            sorterMotor.setPower(-0.3);
            sorterThere = false;
        } else {
            sorterMotor.setPower(0);
            sorterThere = locker.getPosition() <= 0.67;
        }
    }

    public void colorSort() {

        /// Sorts all colors and organizes them into slots

        double currball, width;

        //Assigns the id which the lens sees to a variable which will stay static

        for (int i = 0; i < blocks.length; i++) {
            if (blocks[i].id == 1) {
                tagID = 1;
            } if (blocks[i].id == 2) {
                tagID = 2;
            } if (blocks[i].id == 4) {
                tagID = 3;
            }

//            if (blocks[i].width == 0) {
//                width = 0;
//            } else {
//                width = blocks[i].width;
//            }

            width = blocks[i].width;

            focalLength = (width * 609.6) / 162;

            distance1 = (301 * width / 162);

            //301
        }

        //Calculates color of the ball in the intake

        if (checkColorSensor.green() > checkColorSensor.blue() && checkColorSensor.green() > 80) {
            currball = 1;
        } else if (checkColorSensor.green() < checkColorSensor.blue() && checkColorSensor.green() > 80) {
            currball = 2;
        } else {
            currball = 0;
        }

        //Rotates the virtual index for the sorter

        if (colorSortRelative > sorterMotor.getCurrentPosition() + 90 || colorMove >= 1) {
            Collections.rotate(sorter, 1);
            colorMove -= 1;
            colorSortRelative += 90;
        } else if (colorSortRelative < sorterMotor.getCurrentPosition() - 90 || colorMove <= -1) {
            Collections.rotate(sorter, -1);
            colorMove += 1;
            colorSortRelative -= 90;
        }

        //Resets the top index to nothing after a shot

        if (shooter && launchServo.getPosition() == servoShootPos) sorter.set(1, 0);

    }

    public void peripherals() {
        if (lookingTarget())   {
            aimLight.setPosition(.5);
        } else {
            aimLight.setPosition(0.3);
        }

    }
}
