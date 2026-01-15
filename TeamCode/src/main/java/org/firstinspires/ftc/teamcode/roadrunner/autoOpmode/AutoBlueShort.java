package org.firstinspires.ftc.teamcode.roadrunner.autoOpmode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous

public class AutoBlueShort extends LinearOpMode {

    boolean intakeTest = false, hasMoved = false, sorterMoving = false, shooter = false;
    public double XPOS, YPOS, XPOSDirect, YPOSDirect, encoderX, encoderY;
    int counter = 0;
    double tag = 6, target_value, spinRate, counterColor = 0;
    Pose2d startPose = new Pose2d(-24,9, Math.toRadians(90));
    List<Integer> sorter = new ArrayList<>(Arrays.asList(0,0,0));
    // The order has list pos "0" as the first slot in the clockwise direction of the intake position, the intake in the lost pos is 2
    // this ignores half steps
    // Nothing = value "0", green = value "1", purple = value "2"

    /// This class detects color and sets the first position in the list
    public class CheckColor {
        ColorSensor checkColorSensor;
        Servo locker,launchServo, ballGate;
        DcMotor sorterMotor;
        DcMotorEx launch1, launch2;
        double targetValue;
        boolean greenTrue, purpleTrue, ballThere = false;
        private HuskyLens huskyLens;
        ElapsedTime timer = new ElapsedTime();
        boolean partOne = false, partTwo = false, firstTime = true, timerRunning = false;
        public CheckColor(HardwareMap hardwareMap) {
            sorterMotor = hardwareMap.get(DcMotor.class, "spedMotor");
            checkColorSensor = hardwareMap.get(ColorSensor.class, "checkColorSensor");
            huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
            locker = hardwareMap.get(Servo.class, "locker");
            launchServo = hardwareMap.get(Servo.class, "launchServo");
            ballGate = hardwareMap.get(Servo.class, "ballGate");
            launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
            launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
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

            if (tagID == 1) {
                tag = 1;
            } if (tagID == 2) {
                tag = 2;
            } if (tagID == 4) {
                tag = 3;
            }

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

            //---------------Shoot
            if (shooter) {
                if (!partOne) {
                    launch1.setVelocity(spinRate, AngleUnit.DEGREES);
                    launch2.setVelocity(spinRate, AngleUnit.DEGREES);
                    timerRunning = true;

                    if (launch1.getVelocity(AngleUnit.DEGREES) > spinRate - 7) {
                        launchServo.setPosition(.55);
                        timerRunning = true;
                        if (timer.milliseconds() > 500) {
                            timerRunning = false;
                            partOne = true;
                            launchServo.setPosition(.7);
                            counter++;
                        }
                    }
                } else if (!partTwo) {
                    if (firstTime) {
                        timerRunning = true;
                        if (timer.milliseconds() > 100) {
                            firstTime = false;
                            timerRunning = false;
                            target_value += 180;
                        }
                    } else {
                        if (sorterMotor.getCurrentPosition() <= target_value - 7 || sorterMotor.getCurrentPosition() >= target_value + 7) {
                            locker.setPosition(.70);
                            timerRunning = false;
                            if (sorterMotor.getCurrentPosition() < target_value) {
                                sorterMotor.setPower(0.3);
                            } else if (sorterMotor.getCurrentPosition() > target_value) {
                                sorterMotor.setPower(-0.3);
                            }
                        } else if (sorterMotor.getCurrentPosition() >= target_value - 7 && sorterMotor.getCurrentPosition() <= target_value + 7) {
                            sorterMotor.setPower(0);
                            timerRunning = true;
                            if (timer.milliseconds() > 100) {
                                timerRunning = false;
                                partTwo = true;
                            }

                        }
                        if (sorterMotor.getCurrentPosition() >= target_value - 35 && sorterMotor.getCurrentPosition() <= target_value + 35) {
                            locker.setPosition(.61);

                        } else {
                            ballGate.setPosition(1);
                        }

                        if (partTwo) {
                            partOne = false;
                            partTwo = false;
                            firstTime = true;
                        }
                    }
                }
                if (!timerRunning) {
                    timer.reset();
                }
                if (counter >= 4) {
                    target_value += 90;
                    shooter = false;
                }
            }
            if (sorterMotor.getCurrentPosition() >= target_value - 14 && sorterMotor.getCurrentPosition() <= target_value + 14 && launch1.getVelocity() >= spinRate - 15) {
                if (shooter) {
                    ballGate.setPosition(0.8);
                } else {
                    ballGate.setPosition(1);
                }
            }

            telemetry.addData("Pos0", sorter.get(0));
            telemetry.addData("Pos1", sorter.get(1));
            telemetry.addData("Pos2", sorter.get(2));
            telemetry.addData("Counter", counter);
            telemetry.addData("Shooter:", shooter);
            telemetry.addData("TagID:", tag);
            telemetry.addData("CounterColor", counterColor);
            telemetry.addData("Target", target_value);
            telemetry.addData("Currpos", currpos);
            telemetry.addData("hasMoved", hasMoved);
            telemetry.addData("Moving", sorterMoving);
            telemetry.addData("BallGateCurrent", ballGate.getPosition());
            telemetry.addData("XPos", XPOS);
            telemetry.addData("YPos", YPOS);
            telemetry.addData("XPosDirect", XPOSDirect);
            telemetry.addData("YPosDirect",YPOSDirect);
            telemetry.addData("XPosActual", encoderX);
            telemetry.addData("XPosActual", encoderY);
            telemetry.update();
        }

    }

    /// Intake Actions

    // Intake Start
    public class Intake {
        DcMotor intakeMotor, sorterMotor, fR, fL, bR,bL;
        Servo locker;
        ColorSensor colorSensor;
        ElapsedTime timer = new ElapsedTime();
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

                intakeMotor.setPower(1);
                locker.setPosition(0.61);

                if (colorSensor.green() > 100 && !hasMoved && !sorterMoving && counterColor <= 3) {
                    target_value += 178;
                    counterColor += 1;
                    hasMoved = true;
                }

                bL.setPower(0.22);
                bR.setPower(0.22);
                fL.setPower(0.22);
                fR.setPower(0.22);
                telemetry.update();

                if (counterColor >= 3 && !sorterMoving && !hasMoved && (timer.seconds() > 1)) {
                    target_value += 90;
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

    /// SorterMoveAction

    public class Shift {
        DcMotor sorterMotor;
        public Shift(HardwareMap hardwareMap) {
            sorterMotor = hardwareMap.get(DcMotor.class, "spedMotor");
        }

        public class ShiftAction implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {

                if (tag == 2) {
                    target_value += 180;
                } else if (tag == 3) {
                    target_value += 270;
                }

                return false;
            }
        }

        public Action shiftAction(){
            return new ShiftAction();
        }

    }

    /// ShootAction

    public class Shoot {
        DcMotorEx launch1, launch2;
        DcMotorEx sorterMotor;
        Servo launchServo, locker, ballgate;
        PinpointDrive drive;


        public Shoot(HardwareMap hardwareMap, PinpointDrive drive) {
            this.drive = drive;
            launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
            launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
            sorterMotor = hardwareMap.get(DcMotorEx.class, "spedMotor");
            launch2.setDirection(DcMotorEx.Direction.REVERSE);
            launchServo = hardwareMap.get(Servo.class, "launchServo");
            locker = hardwareMap.get(Servo.class, "locker");
            ballgate = hardwareMap.get(Servo.class, "ballGate");

        }

        public class ShootAction1 implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                double targetX = 127, targetY = 57, currpos = sorterMotor.getCurrentPosition();
                double xDistance = targetX - drive.pinpoint.getPosX(), yDistance = targetY - drive.pinpoint.getPosX();
                double distance = Math.sqrt((xDistance * xDistance) + (yDistance * yDistance));
                spinRate = 252;
                shooter = true;
                if (counter >= 4) {
                    shooter = false;
                    counter = 0;
                    //target_value += 90;
                    return false;

                }
                return true;

            }
        }

        public class ShootAction2 implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                double targetX = 127, targetY = 57, currpos = sorterMotor.getCurrentPosition();
                double xDistance = targetX - drive.pinpoint.getPosX(), yDistance = targetY - drive.pinpoint.getPosX();
                double distance = Math.sqrt((xDistance * xDistance) + (yDistance * yDistance));
                spinRate = 210;
                shooter = true;
                if (counter >= 4) {
                    shooter = false;
                    counter = 0;
                    //target_value += 90;
                    return false;

                }
                return true;

            }
        }

        public Action shootAction1(){
            return new ShootAction1();
        }

        public Action shootAction2(){
            return new ShootAction2();
        }

    }

    @Override
    public void runOpMode() {
        /// Original -15.5
        Pose2d ballPos3 = new Pose2d(-35, 82.5, Math.toRadians(180));
        Pose2d ballPos2 = new Pose2d(-35, 52.5, Math.toRadians(180));
        Pose2d ballPos1 = new Pose2d(-35, 35.25, Math.toRadians(180));
        Pose2d shootPos2 = new Pose2d(-15, 94, Math.toRadians(135));
        Pose2d shootPos1 = new Pose2d(-12.5, 17, Math.toRadians(113));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);
        drive.pinpoint.resetPosAndIMU();
        drive.pinpoint.setPosition(startPose);
        Shift shift = new Shift(hardwareMap);
        Intake intake = new Intake(hardwareMap);
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
                        .splineToLinearHeading(new Pose2d(-24, 25, Math.toRadians(80)), Math.toRadians(80))
                        .stopAndAdd(shift.shiftAction())
                        .splineToLinearHeading(shootPos2, Math.toRadians(135))
                        .stopAndAdd(noShoot.shootAction2())
                        .splineToLinearHeading(ballPos1, Math.toRadians(180))
                        .stopAndAdd(intake.intakeAction())
                        .splineToLinearHeading(shootPos2, Math.toRadians(135))
                        .stopAndAdd(noShoot.shootAction2())
                        .splineToLinearHeading(ballPos2, Math.toRadians(180))
                        .stopAndAdd(intake.intakeAction())
                        .splineToLinearHeading(shootPos2, Math.toRadians(135))
                        .stopAndAdd(noShoot.shootAction2())
                        .build()
        );
    }
}