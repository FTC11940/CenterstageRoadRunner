/**
 * TODO
 * Team:
 * Year:
 * Title:
 * Summary:
 **/

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*. *
 *
 */

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define the Motors (Make them private so they can't be accessed externally)

    // Defines the front right motor
    private DcMotor frontRightMotor = null;

    // Defines the front left motor
    private DcMotor frontLeftMotor = null;

    // Defines the back right motor
    private DcMotor backRightMotor = null;

    // Defines the back left motor
    private DcMotor backLeftMotor = null;

    // Defines the lift motor
    private DcMotor liftMotor = null;

    // Defines the intake motor
    private DcMotor intakeMotor = null;

    // Defines the motor for climbing
    private DcMotor climbMotor = null;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        climbMotor  = myOpMode.hardwareMap.get(DcMotor.class, "climb_Motor");
        intakeMotor = myOpMode.hardwareMap.get(DcMotor.class, "intake_Motor");
        liftMotor   = myOpMode.hardwareMap.get(DcMotor.class, "lift_Motor");
        backLeftMotor  = myOpMode.hardwareMap.get(DcMotor.class, "back_Left_Motor");
        backRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "back_Right_Motor");
        frontLeftMotor   = myOpMode.hardwareMap.get(DcMotor.class, "front_Left_Motor");
        frontRightMotor   = myOpMode.hardwareMap.get(DcMotor.class, "front_Right_Motor");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        climbMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
//       leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
//       rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
//       leftHand.setPosition(MID_SERVO);
//        rightHand.setPosition(MID_SERVO);

        //set motors to use no power
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        climbMotor.setPower(0);
        intakeMotor.setPower(0);
        liftMotor.setPower(0);

        myOpMode.telemetry.addData("->", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * // @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * // @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */

    public void mecanumDrive (double Y, double X, double Rot) {
        /*
         * Declare the power variables for each wheel
         * Includes references to drive input variables
         * 'y' drive, 'x' strafe, and 'rot' rotation for mecanum-style
         * 'y' drive and 'rot' turn would only be needed for tank-style
         * 'mecanumDrive' will be called in the teleop file
         */
        double frontLeftPower = (Y + X + Rot) * DRIVE_POWER;
        double backLeftPower = (Y - X + Rot) * DRIVE_POWER;
        double frontRightPower = (Y - X - Rot) * DRIVE_POWER;
        double backRightPower = (Y + X - Rot) * DRIVE_POWER;

        // Scale values goes here
        /*
         double maxPower = Math.maxPower(Math.abs(frontLeftPower),
                 Math.abs(frontRightPower),
                 Math.abs(backRightPower),
                 Math.abs(backLeftPower));
         if (maxPower > 1.0) {
             frontLeftPower /= maxPower;
             frontRightPower /= maxPower;
             backRightPower /= maxPower;
             backLeftPower /= maxPower;
         }
         */

        // Use the existing function to power the wheels
        setDrivePower(frontLeftPower, frontRightPower, backRightPower, backLeftPower);
    }
    public void setDrivePower(double frontLeftPower, double frontRightPower, double backRightPower, double backLeftPower) {

        // Send calculated power to wheels
        frontLeftMotor.setPower((frontLeftPower) * DRIVE_POWER);
        backLeftMotor.setPower((backLeftPower) * DRIVE_POWER);
        frontRightMotor.setPower((frontRightPower) * DRIVE_POWER);
        backRightMotor.setPower((backRightPower) * DRIVE_POWER);
    }

    /*
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * / @param power driving power (-1.0 to 1.0)
     */

    /*
    * Not currently in use
    *

    public void setArmPower(double power) {armMotor.setPower(power);
    }

    /*
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */

    /*
    * Not currently in use
    *
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
    }

    */
}

/* Copyright (c) 2022 FIRST. All rights reserved.
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