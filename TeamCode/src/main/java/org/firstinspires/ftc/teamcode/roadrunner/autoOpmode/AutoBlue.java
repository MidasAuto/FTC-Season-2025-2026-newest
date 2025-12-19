package org.firstinspires.ftc.teamcode.roadrunner.autoOpmode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.PinpointEncoder;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.extraCode.ShootAllThree;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous

public class AutoBlue extends LinearOpMode {

    boolean intakeTest = false, hasMoved = false, sorterMoving = false;
    double tag = 6, target_value;
    List<Integer> sorter = new ArrayList<>(Arrays.asList(0,0,0));
    // The order has list pos "0" as the first slot in the clockwise direction of the intake position, the intake in the lost pos is 2
    // this ignores half steps
    // Nothing = value "0", green = value "1", purple = value "2"

    /// This class detects color and sets the first position in the list
    public class CheckColor {
        ColorSensor checkColorSensor;
        Servo locker;
        DcMotor sorterMotor;
        double targetValue;
        boolean greenTrue, purpleTrue, ballThere = false;
        private HuskyLens huskyLens;
        ElapsedTime timer = new ElapsedTime();
        public CheckColor(HardwareMap hardwareMap) {
            sorterMotor = hardwareMap.get(DcMotor.class, "spedMotor");
            checkColorSensor = hardwareMap.get(ColorSensor.class, "checkColorSensor");
            huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
            locker = hardwareMap.get(Servo.class, "locker");
            timer.startTime();
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        }

        public void checkBall() {

            HuskyLens.Block[] blocks = huskyLens.blocks();

            double currpos = sorterMotor.getCurrentPosition();

            if (currpos <= target_value-7 || currpos >= target_value+7) {
                if (currpos < target_value) {
                    sorterMotor.setPower(.3);
                } else if (currpos > target_value) {
                    sorterMotor.setPower(-0.3);
                }
                sorterMoving = true;
            } else if (currpos >= target_value-7 && currpos <= target_value+7) {
                sorterMotor.setPower(0);
                sorterMoving = false;
            }
            if (currpos >= target_value-35 && currpos <= target_value+35) {
                locker.setPosition(.61);
                hasMoved = false;
            } else {
                locker.setPosition(0.7);
                timer.reset();
            }

            double tagID = 0;

            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].id);

                tagID = blocks[i].id;
            }

            tag = tagID;

            double blueValue = checkColorSensor.blue();
            double greenValue = checkColorSensor.green();
            if (greenValue > 100) {
                ballThere = true;
            }
            if (blueValue > greenValue && ballThere && !sorterMoving) {
                purpleTrue = true;
                greenTrue = false;
                sorter.set(0,1);
                targetValue += 180;
            } else if (blueValue < greenValue && ballThere && !sorterMoving) {
                greenTrue = true;
                purpleTrue = false;
                sorter.set(0,2);
                targetValue += 180;
            } else {
                greenTrue = false;
                purpleTrue = false;
            }
        }

    }

    /// Intake Actions

    // Intake Start
    public class Intake {
        DcMotor intakeMotor, sorterMotor, fR, fL, bR,bL;
        Servo locker;
        ColorSensor colorSensor;
        ElapsedTime timer = new ElapsedTime();
        double counter = 0;
        public Intake(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            sorterMotor = hardwareMap.get(DcMotor.class, "spedMotor");
            locker = hardwareMap.get(Servo.class, "locker");
            colorSensor= hardwareMap.get(ColorSensor.class, "colorSensor");
            fR = hardwareMap.get(DcMotor.class, "rightFront");
            fL = hardwareMap.get(DcMotor.class, "leftFront");
            bL = hardwareMap.get(DcMotor.class, "leftBack");
            bR = hardwareMap.get(DcMotor.class, "rightBack");
            timer.startTime();
            target_value = sorterMotor.getCurrentPosition();
        }

        public class IntakeAction implements Action {

            public boolean run(@NonNull TelemetryPacket packet) {

                double currpos = sorterMotor.getCurrentPosition();
                double GoTO;

                intakeMotor.setPower(1);
                locker.setPosition(0.61);

                if (colorSensor.green() > 100 && !hasMoved && !sorterMoving && counter <= 3) {
                    target_value += 178;
                    counter += 1;
                    hasMoved = true;
                }

                bL.setPower(0.2);
                bR.setPower(0.2);
                fL.setPower(0.2);
                fR.setPower(0.2);

                telemetry.addData("Color", colorSensor.green());
                telemetry.addData("Counter", counter);
                telemetry.addData("Target", target_value);
                telemetry.addData("Currpos", currpos);
                telemetry.addData("hasMoved", hasMoved);
                telemetry.addData("Moving", sorterMoving);
                telemetry.update();

                if (counter >= 3 && !sorterMoving && !hasMoved && (timer.seconds() > 1)) {
                    return false;
                } else {

                    bL.setPower(0);
                    bR.setPower(0);
                    fL.setPower(0);
                    fR.setPower(0);
                    return true;
                }
            }
        }

        public Action intakeAction(){
            return new IntakeAction();
        }

    }

    // IntakeStop

    public class ShootStart {
        DcMotorEx launch1, launch2;
        Servo launch;


        public ShootStart(HardwareMap hardwareMap) {
            launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
            launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
            launch = hardwareMap.get(Servo.class, "launchServo");
        }

        public class ShootStartAction implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {

                double vel = 265;

                launch1.setVelocity(vel, AngleUnit.DEGREES);
                launch2.setVelocity(vel, AngleUnit.DEGREES);

                if (launch1.getVelocity(AngleUnit.DEGREES) > vel -3) {
                    launch.setPosition(0.55);
                } else {
                    launch.setPosition(0.7);
                }


                return false;
            }
        }

        public Action shootStartAction(){
            return new ShootStartAction();
        }

    }

    public class ShootStop {
        DcMotorEx launch1, launch2;

        public ShootStop(HardwareMap hardwareMap) {
            launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
            launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
        }

        public class ShootStopAction implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {

                launch1.setVelocity(0);
                launch2.setVelocity(0);

                return false;
            }
        }

        public Action shootStopAction(){
            return new ShootStopAction();
        }

    }

    /// SorterMoveAction

    public class Shift {
        DcMotor sorterMotor;
        private HuskyLens huskyLens;

        public Shift(HardwareMap hardwareMap) {
            sorterMotor = hardwareMap.get(DcMotor.class, "spedMotor");
            huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        }

        public class ShiftAction implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {

                if (tag == 0) {

                }


                sorterMotor.setPower(1);

                return false;
            }
        }

        public Action shiftAction(){
            return new ShiftAction();
        }

    }

    public void telemetry() {

        telemetry.addData("Pos0", sorter.get(0));
        telemetry.addData("Pos1", sorter.get(1));
        telemetry.addData("Pos2", sorter.get(2));

        telemetry.update();
    }

    /// ShootAction

    public class Shoot {
        DcMotorEx launch1, launch2;
        DcMotorEx sorterMotor;
        Servo launchServo, locker, ballgate;
        ShootAllThree shootThree = new ShootAllThree();
        PinpointDrive drive;
        ElapsedTime timer = new ElapsedTime();
        boolean shootMoveFRL = false;

        public Shoot(HardwareMap hardwareMap, PinpointDrive drive) {
            this.drive = drive;
            launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
            launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
            sorterMotor = hardwareMap.get(DcMotorEx.class, "spedMotor");
            launch2.setDirection(DcMotorEx.Direction.REVERSE);
            launchServo = hardwareMap.get(Servo.class, "launchServo");
            locker = hardwareMap.get(Servo.class, "locker");
            ballgate = hardwareMap.get(Servo.class, "ballGate");
            ballgate.setPosition(0.8);
        }

        public class ShootAction implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                double targetX = 127, targetY = 57, currpos = sorterMotor.getCurrentPosition();
                double xDistance = targetX - drive.pinpoint.getPosX(), yDistance = targetY - drive.pinpoint.getPosX();
                double distance = Math.sqrt((xDistance * xDistance) + (yDistance * yDistance));
                double spinRate = 326;

                if (!shootMoveFRL) {
                    target_value += 90;
                    shootMoveFRL = true;
                }

                if (launch1.getVelocity() > 30) {

                }

                if (timer.seconds() > 1) {
                    telemetry.addData("SpinRate: ", spinRate);
                    telemetry.addData("Vel", ShootAllThree.velocity);
                    telemetry.update();
                    shootThree.shootAllThree(launch1, launch2, launchServo, locker, spinRate, sorterMotor);
                    return false;
                }

                return true;
            }
        }

        public Action shootAction(){
            return new ShootAction();
        }

    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-24,9, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);
        Intake intake = new Intake(hardwareMap);
        ShootStart shootStart = new ShootStart(hardwareMap);
        ShootStop shootStop = new ShootStop(hardwareMap);
        CheckColor checkColor = new CheckColor(hardwareMap);
        Shoot noShoot = new Shoot(hardwareMap, drive);


        waitForStart();
        new Thread(() -> {
            while (opModeIsActive()) {
                checkColor.checkBall();
            }
        }).start();


        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-10, 41, Math.toRadians(90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-15, 56, Math.toRadians(180)), Math.toRadians(180))
                        .stopAndAdd(intake.intakeAction())
                        .splineToLinearHeading(new Pose2d(41, 31, Math.toRadians(90)), Math.toRadians(90))
                        .stopAndAdd(noShoot.shootAction())
                        .build()
        );
    }
}
