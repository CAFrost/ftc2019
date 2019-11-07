/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.SwitchableLight;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="A:Lab", group="Lab")
public class AutonomousLab extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0 = null, motor1 = null, motor2 = null, motor3 = null, motor4 = null, motor5 = null;
    private Servo servo0 = null, servo1 = null, servo2 = null;
    private boolean turnleft, driveForward1, driveForward2, resetEncoderTurnLeft, resetEncoderDriveForward;
    private int startingPositionDrive1, startingPositionDrive2, startingPositionTurnLeft;


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        motor0 = hardwareMap.get(DcMotor.class, "m0");
        motor1 = hardwareMap.get(DcMotor.class, "m1");
        motor2 = hardwareMap.get(DcMotor.class, "m2");
        motor3 = hardwareMap.get(DcMotor.class, "m3");
        motor4 = hardwareMap.get(DcMotor.class, "m4");
        motor5 = hardwareMap.get(DcMotor.class, "m5");
        servo0 = hardwareMap.get(Servo.class,  "s0");
        servo1 = hardwareMap.get(Servo.class,  "s1");
        servo2 = hardwareMap.get(Servo.class,  "s2");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        driveForward1 = true;
        driveForward2 = false;
        turnleft = false;

    }

    @Override
    public void loop() {
        double leftPower;
        double rightPower;


        leftPower = -0.5;
        rightPower = -0.5;
        if (driveForward1 == true){
            int motorPosition = motor1.getCurrentPosition();
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            if (motorPosition < (startingPositionDrive1 + 300)) {
                motor0.setPower(rightPower);
                motor1.setPower(rightPower);0
                motor2.setPower(leftPower);
                motor3.setPower(leftPower);

            } else {
                motor0.setPower(0);
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                driveForward1 = false;
                turnleft = true;
                resetEncoderTurnLeft = true;

            }

        }
        if (turnleft == true) {
            if (resetEncoderTurnLeft == true) {
                startingPositionTurnLeft = motor1.getCurrentPosition();
                resetEncoderTurnLeft = false;
            }
            int motorPositionLeft = motor1.getCurrentPosition();
            telemetry.addData("TurnLeft", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("TurnLeft", "current (%d), starting (%d)", motorPositionLeft, startingPositionTurnLeft);

            if (motorPositionLeft > (startingPositionTurnLeft - 1100)) {

                motor0.setPower(-rightPower);
                motor1.setPower(-rightPower);
                motor2.setPower(leftPower);
                motor3.setPower(leftPower);
            } else {
                motor0.setPower(0);
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                resetEncoderDriveForward = true;
                driveForward2 = true;
                turnleft = false;

                //REVOLUTION: 360 per turn of wheel


            }
        }
        if (driveForward2 == true){
            if (resetEncoderDriveForward == true){
                startingPositionDrive2 = motor1.getCurrentPosition();
                resetEncoderDriveForward = false;
            }
            int motorPositionForward = motor1.getCurrentPosition();
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Forward", "current (%d), starting (%d)", motorPositionForward, startingPositionDrive2);

            if (motorPositionForward > (startingPositionDrive2 - 500)) {
                motor0.setPower(-rightPower);
                motor1.setPower(-rightPower);
                motor2.setPower(-leftPower);
                motor3.setPower(-leftPower);
            } else {
                motor0.setPower(0);
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                driveForward2 = false;

            }


        }

    }


    @Override
    public void stop() {
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        motor5.setPower(0);
    }

    public void closeClaw(){
        servo1.setPosition(0.5);
        servo0.setPosition(0.5);
        telemetry.addData("Servos", "Left (%d), Right (%d)", 0, 1);
    }


    public void openClaw(){
        servo1.setPosition(0.8);
        servo0.setPosition(0.3);
        telemetry.addData("Servos", "Left (%d), Right (%d)", 1, 0);

    }
    public void wristup() {
        servo2.setPosition(0.0);
        telemetry.addData("Servos", "wrist (%d),", 0);
    }
    public void wristdown() {
        servo2.setPosition(1.0);
        telemetry.addData("servos", "wrist (%d)", 1);
    }


    }


