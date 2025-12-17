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

    boolean intakeTest = false;
    double tag = 6;
    List<Integer> sorter = new ArrayList<>(Arrays.asList(0,0,0));
    // The order has list pos "0" as the first slot in the clockwise direction of the intake position, the intake in the lost pos is 2
    // this ignores half steps
    // Nothing = value "0", green = value "1", purple = value "2"

    /// This class detects color and sets the first position in the list
    public class CheckColor {
        ColorSensor checkColorSensor;
        double targetValue;
        boolean greenTrue, purpleTrue, ballThere = false, sorterMoving = false;
        private HuskyLens huskyLens;
        public CheckColor(HardwareMap hardwareMap) {
            checkColorSensor = hardwareMap.get(ColorSensor.class, "checkColorSensor");
            huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        }

        public void checkBall() {

            HuskyLens.Block[] blocks = huskyLens.blocks();

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
        double counter = 0, target_value;
        boolean sorterMoving = false, hasMoved = false;
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

                if (colorSensor.green() > 100 && !hasMoved) {
                    target_value += 180;
                    counter += 1;
                    hasMoved = true;
                }

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
                if (currpos >= target_value-7 && currpos <= target_value+7) {
                    locker.setPosition(.61);
                    hasMoved = false;
                } else {
                    locker.setPosition(0.7);
                    timer.reset();
                }
                /*
                bL.setPower(0.3);
                bR.setPower(0.3);
                fL.setPower(0.3);
                fR.setPower(0.3);

                 */

                telemetry.addData("Color", colorSensor.green());
                telemetry.addData("Counter", counter);
                telemetry.addData("Target", target_value);
                telemetry.addData("Currpos", currpos);
                telemetry.addData("hasMoved", hasMoved);
                telemetry.addData("Moving", sorterMoving);
                telemetry.update();

                if (counter == 3 && !sorterMoving && !hasMoved) {
                    return false;
                } else {
                    /*
                    bL.setPower(0);
                    bR.setPower(0);
                    fL.setPower(0);
                    fR.setPower(0);

                     */
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
                HuskyLens.Block[] blocks = huskyLens.blocks();

                double tagID = 0;

                for (int i = 0; i < blocks.length; i++) {
                    telemetry.addData("Block", blocks[i].id);

                    tagID = blocks[i].id;
                }

                tag = tagID;



                sorterMotor.setPower(1);

                return false;
            }
        }

        public Action shiftAction(){
            return new ShiftAction();
        }

    }


    //Sorter


    public class SorterMove {

        double pattern;
        DcMotor sorterMotor;
        CheckColor checkColor;
        private HuskyLens huskyLens;

        public SorterMove(HardwareMap hardwareMap) {
            sorterMotor = hardwareMap.get(DcMotor.class, "spedMotor");
            sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        }

        public class SorterMoveAction implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                double tagID = 0;

                double currPos = sorterMotor.getCurrentPosition(), greenPosNeed = 0;

                for (int i = 0; i < blocks.length; i++) {
                    telemetry.addData("Block", blocks[i].id);

                    tagID = blocks[i].id;
                }

                if (tagID == 1) {
                    pattern = 1;
                } else if (tagID == 2) {
                    pattern = 2;
                } else if (tagID == 3) {
                    pattern = 3;
                } else {
                    pattern = 0;
                }

                if (pattern == 1) {
                    greenPosNeed = 0;
                } else if (pattern == 2) {
                    greenPosNeed = 1;
                } else if (pattern == 3) {
                    greenPosNeed = 2;
                }

                if (greenPosNeed < sorter.indexOf(1)) {
                    checkColor.targetValue += 180;
                    sorter.set(sorter.indexOf(1), 2);
                    sorter.set(sorter.indexOf(1)+1, 1);
                } else if (greenPosNeed > sorter.indexOf(1)) {
                    checkColor.targetValue -= 180;
                    sorter.set(sorter.indexOf(1), 2);
                    sorter.set(sorter.indexOf(1)-1, 1);
                }

                if (currPos <= checkColor.targetValue-7 || currPos >= checkColor.targetValue+7) {
                    if (currPos < checkColor.targetValue) {
                        sorterMotor.setPower(0.3);
                        checkColor.sorterMoving = true;
                    }
                    else if (currPos > checkColor.targetValue) {
                        sorterMotor.setPower(-0.3);
                        checkColor.sorterMoving = true;
                    }
                }
                else if (currPos >= checkColor.targetValue-7 && currPos <= checkColor.targetValue+7){
                    sorterMotor.setPower(0);
                }
                return true;
            }
        }

        public Action sorterMoveAction(){
            return new SorterMoveAction();
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

        public class ShootAction implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                double targetX = 127, targetY = 57;
                double xDistance = targetX - drive.pinpoint.getPosX(), yDistance = targetY - drive.pinpoint.getPosX();
                double distance = Math.sqrt((xDistance * xDistance) + (yDistance * yDistance));
                double spinRate = 1.038 * distance + 204.24;

                if (distance < 95) {

                    ///Change this for short distance

                    spinRate += 15;
                } else {

                    ///Change this for long distance

                    spinRate += 3;
                }

                if (launch1.getVelocity() > 30) {

                }

                shootThree.shootAllThree(launch1, launch2, launchServo, locker, spinRate, sorterMotor);

                return false;
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
        SorterMove sorterMove = new SorterMove(hardwareMap);
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
                        .splineToLinearHeading(new Pose2d(-31, 9, Math.toRadians(90)), Math.toRadians(90))
                        .afterDisp(1, intake.intakeAction())
                        .splineToLinearHeading(new Pose2d(41, 31, Math.toRadians(90)), Math.toRadians(90))
                        .stopAndAdd(noShoot.shootAction())
                        .build()
        );
    }
}
