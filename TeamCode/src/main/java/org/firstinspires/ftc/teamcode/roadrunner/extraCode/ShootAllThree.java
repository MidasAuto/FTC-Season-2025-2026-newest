package org.firstinspires.ftc.teamcode.roadrunner.extraCode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ShootAllThree {
    int counter = 0;
    int targetValue = 0;
    ElapsedTime timer = new ElapsedTime();


    boolean partOne = false, partTwo = false, firstTime = true, timerRunning;

    public void shootAllThree(DcMotorEx launch1, DcMotorEx launch2, Servo launchServo, Servo locker, double spinRate, DcMotorEx sorterMotor) {
        while(true) {
            if (!partOne) {
                launch1.setVelocity(spinRate, AngleUnit.DEGREES);
                launch2.setVelocity(spinRate, AngleUnit.DEGREES);
                timerRunning = true;
                /*if (timer.milliseconds() > 3000) {
                    timerRunning = false;
                    break;
                }*/
                if (launch1.getVelocity(AngleUnit.DEGREES) > spinRate-7) {
                    launchServo.setPosition(.55);
                    timerRunning = true;
                    if (timer.milliseconds() > 500) {
                        timerRunning = false;
                        partOne = true;
                        launchServo.setPosition(.7);
                        counter++;
                    }
                }
            }
            else if (!partTwo) {
                if (firstTime) {
                    targetValue = sorterMotor.getCurrentPosition() + 180;
                    timerRunning = true;
                    if (timer.milliseconds() > 100) {
                        firstTime = false;
                        timerRunning = false;
                    }
                }
                else {
                    if (sorterMotor.getCurrentPosition() <= targetValue - 7 || sorterMotor.getCurrentPosition() >= targetValue + 7) {
                        locker.setPosition(.70);
                        timerRunning = false;
                        if (sorterMotor.getCurrentPosition() < targetValue) {
                            sorterMotor.setPower(0.3);
                        } else if (sorterMotor.getCurrentPosition() > targetValue) {
                            sorterMotor.setPower(-0.3);
                        }
                    } else if (sorterMotor.getCurrentPosition() >= targetValue - 7 && sorterMotor.getCurrentPosition() <= targetValue + 7) {
                        sorterMotor.setPower(0);
                        timerRunning = true;
                        if (timer.milliseconds() > 100) {
                            timerRunning = false;
                            partTwo = true;
                        }

                    }
                    if (sorterMotor.getCurrentPosition() >= targetValue - 35 && sorterMotor.getCurrentPosition() <= targetValue + 35) {
                        locker.setPosition(.61);
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
            if (counter == 3) {
                break;
            }
        }
        counter = 0;
        partOne = false;
        partTwo = false;
        firstTime = true;
    }
}