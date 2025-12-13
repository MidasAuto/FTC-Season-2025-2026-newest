package org.firstinspires.ftc.teamcode.roadrunner.autoOpmode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous


public class AutoBlue extends LinearOpMode {

    boolean intakeTest = false;

    List<Integer> sorter = new ArrayList<>(Arrays.asList(0,0,0));
    // The order has list pos "0" as the first slot in the clockwise direction of the intake position, the intake in the lost pos is 2
    // this ignores half steps
    // Nothing = value "0", green = value "1", purple = value "2"

    /// This class detects color and sets the first position in the list
    public class CheckColor {
        ColorSensor checkColorSensor;
        double targetValue;
        boolean greenTrue, purpleTrue, ballThere = false, sorterMoving = false;
        public CheckColor(HardwareMap hardwareMap) {
            checkColorSensor = hardwareMap.get(ColorSensor.class, "checkColorSensor");
        }

        public void checkBall() {
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
        DcMotor intakeMotor, sorterMotor;
        Servo locker;
        ColorSensor colorSensor;
        ElapsedTime timer = new ElapsedTime();
        double counter = 0;
        boolean sorterMoving = false;

        public Intake(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            sorterMotor = hardwareMap.get(DcMotor.class, "spedMotor");
            locker = hardwareMap.get(Servo.class, "locker");
            colorSensor= hardwareMap.get(ColorSensor.class, "colorSensor");
            timer.startTime();

        }

        public class IntakeAction implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {

                double currpos = sorterMotor.getCurrentPosition(), target_value = sorterMotor.getCurrentPosition();

                intakeMotor.setPower(1);

                if (colorSensor.green() > 200) {
                    target_value += 180;
                    counter += 1;
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
                if (currpos >= target_value-35 && currpos <= target_value+35) {
                    locker.setPosition(.61);
                }

                timer.reset();

                if (counter == 3 && !sorterMoving) {
                    return false;
                }




                return true;
            }
        }

        public Action intakeAction(){
            return new IntakeAction();
        }

    }

    // IntakeStop

    public class IntakeStop {
        DcMotor intakeMotor;

        public IntakeStop(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        }

        public class IntakeStopAction implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {

                intakeMotor.setPower(0);

                return false;
            }
        }

        public Action intakeStopAction(){
            return new IntakeStopAction();
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

                double tagID;

                for (int i = 0; i < blocks.length; i++) {
                    telemetry.addData("Block", blocks[i].id);

                    tagID = blocks[i].id;
                }



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

        public SorterMove(HardwareMap hardwareMap) {
            sorterMotor = hardwareMap.get(DcMotor.class, "spedMotor");
            sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class SorterMoveAction implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {

                double currPos = sorterMotor.getCurrentPosition(), greenPosNeed = 0;

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
        DcMotor sorterMotor;
        Servo launchServo;
        PinpointDrive drive;
        private HuskyLens huskyLens;

        public Shoot(HardwareMap hardwareMap, PinpointDrive drive) {
            this.drive = drive;
            launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
            launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
            sorterMotor = hardwareMap.get(DcMotor.class, "spedMotor");
            launch2.setDirection(DcMotorEx.Direction.REVERSE);
            huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
            launchServo = hardwareMap.get(Servo.class, "launchServo");
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        }

        public class ShootAction implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                double x = drive.getPose().position.x, y = drive.getPose().position.y, heading = drive.pinpoint.getHeading();
                double targetx = 127, targety = 57, pattern, tagID = 0;

                double distance1 = targetx - x, distance2 = targety - y;
                double length = Math.sqrt((distance1 * distance1) + (distance2 * distance2));
                double spinRate = (1.038 * length + 204.24);

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




                telemetry.addData("distance", length);
                telemetry.addData("spinrate", spinRate);
                telemetry.addData("PinpointX", x);
                telemetry.addData("PinpointY", y);
                telemetry.addData("current", launch1.getVelocity(AngleUnit.DEGREES));
                telemetry.update();

                launch1.setVelocity(spinRate, AngleUnit.DEGREES);
                launch2.setVelocity(spinRate, AngleUnit.DEGREES);

                if (launch1.getVelocity(AngleUnit.DEGREES) > (spinRate - 5)) {
                    launchServo.setPosition(0.55);

                    return false;
                } else {
                    launchServo.setPosition(0.7);
                    return true;
                }
            }
        }

        public Action shootAction(){
            return new ShootAction();
        }

    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(9,-24, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);
        Intake intake = new Intake(hardwareMap);
        IntakeStop intakeStop = new IntakeStop(hardwareMap);
        SorterMove sorterMove = new SorterMove(hardwareMap);
        CheckColor checkColor = new CheckColor(hardwareMap);
        Shoot shootStart = new Shoot(hardwareMap, drive);


        waitForStart();
        new Thread(() -> {
            while (opModeIsActive()) {
                checkColor.checkBall();
            }
        }).start();


        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(9, -31, Math.toRadians(180)), Math.toRadians(90))
                        .build()


        );
    }
}
