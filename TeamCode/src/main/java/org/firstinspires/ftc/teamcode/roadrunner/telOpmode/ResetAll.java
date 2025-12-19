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


@TeleOp(name = "ResetAll")
public class ResetAll extends OpMode {

    DcMotor frontRightMotor, intakeMoter, backLeftMotor, backRightMotor, frontLeftMotor;
    DcMotorEx launch1, launch2, sorterMotor;
    ColorSensor checkColorSensor;
    Servo launchServo, ballGate, locker, aimLight;

    ShootAllThree shootThree = new ShootAllThree();

    //----------------------

    double redValue = 1, blueValue = 1, greenValue = 1, alphaValue = 1;
    boolean greenTrue = false, purpleTrue = false;
    /// Start position
    Pose2d currPos = new Pose2d(0, 0, 90);
    //Ball Positions
    List<Integer> holderOne = new ArrayList<>(Arrays.asList(0, 0)), holderTwo = new ArrayList<>(Arrays.asList(2, 0)), holderThree = new ArrayList<>(Arrays.asList(4, 0));
    //timer
    int target_value = 0;
    double distance = 49;
    double spinRate = 1.038 * distance + 204.24;
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



    }
}
