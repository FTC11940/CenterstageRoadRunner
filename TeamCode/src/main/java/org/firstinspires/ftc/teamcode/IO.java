/**
 * TODO
 * Team:
 * Year:
 * Title:
 * Summary:
 **/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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

// Input-Output that's not drive motor or hub related.
// Previously referring to it as a hardware class
public class IO {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    /**
    // Define the Motors (Make them private so they can't be accessed externally)
    // Defines the front right motor
    private DcMotor frontRightMotor = null;
    // Defines the front left motor
    private DcMotor frontLeftMotor = null;
    // Defines the back right motor
    private DcMotor backRightMotor = null;
    // Defines the back left motor
    private DcMotor backLeftMotor = null;
     */

    // Defines the lift motor
    public DcMotor liftMotor = null;

    // Defines the intake motor
    public static DcMotor intake = null;

    // Defines the motors for climbing
    public DcMotor leftSlide = null;
    public DcMotor rightSlide = null;

    public DcMotor armRot = null;

    // TODO put in the configure setup

    // Variables for the servos
    static final double CUP_INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CUP_CYCLE_MS    =   50;     // period of each cycle
    static final double CUP_MAX_POS     =  1.0;     // Maximum rotational position
    static final double CUP_MIN_POS     =  0.0;     // Minimum rotational position

    static final double CUP_DEPLOY_POS     =  1.0;     // TODO check position for pixel deployment
    static final double CUP_INTAKE_POS     =  0.0;     // TODO check position for pixel intake

    static final double TAB_OPEN = 1.0; // TODO check position
    static final double TAB_CLOSE = 0.0; // TODO check position

    // Define class members
    double  cupPos = (CUP_MAX_POS - CUP_MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    // Servo to rotate the cup/pocket/bucket over and back
    public Servo leftCupRotation;
    public Servo rightCupRotation;

    // Servo with a tab to open/close the opening of the cup
    private Servo cupTab;


    // TODO
    public enum SCORE_POSITION{
        INTAKE_POS, // resting or home
        SCORE_LOW, // deploy to lowest
        SCORE_MID, // deploy to mid height
        SCORE_HIGH, // deploy to highest
        CLIMB, // climbing, break off into another method
    }

    public static SCORE_POSITION scorePosition;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public IO (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftSlide  = myOpMode.hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide  = myOpMode.hardwareMap.get(DcMotor.class, "rightSlide");
        intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");
        armRot   = myOpMode.hardwareMap.get(DcMotor.class, "armRot");

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        armRot.setDirection(DcMotor.Direction.FORWARD);

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        leftCupRotation = myOpMode.hardwareMap.get(Servo.class, "leftCupRot");
        rightCupRotation = myOpMode.hardwareMap.get(Servo.class, "rightCupRot");
        cupTab = myOpMode.hardwareMap.get(Servo.class, "cupTab");

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // doesn't need an encoder!
        armRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set motors to use no power
        leftSlide.setPower(0);
        rightSlide.setPower(0);
        intake.setPower(0);
        armRot.setPower(0);

        /** In the 'MecanumDrive'
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

         //set motors to use no power
         backLeftMotor.setPower(0);
         backRightMotor.setPower(0);
         frontRightMotor.setPower(0);
         frontLeftMotor.setPower(0);
         climbMotor.setPower(0);
         intakeMotor.setPower(0);
         liftMotor.setPower(0);

         */
        // Define and initialize ALL installed servos.
//       leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
//       rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
//       leftHand.setPosition(MID_SERVO);
//        rightHand.setPosition(MID_SERVO);


        myOpMode.telemetry.addData("->", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    // TODO
    // Separate classes is probably overkill
    // Consider using a variable that changes for deployment and intake positions
    public void setTabOpen() {

    }

    // TODO
    public void setTabClose() {

    }

    // TODO sets the cup for deployment
    // Separate classes is probably overkill
    // Consider using a variable that changes for deployment and intake positions
    public void setCupDeploy() {

    }

    // TODO Sets the cup for intake position
    public void setCupIntake() {

    }

    // TODO Pushes slide to one of three positions
    public void setDeployment(){

    }

    // TODO
    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * // @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * // @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */

    /** Part of 'MecanimDrive
    public void mecanumDrive (double Y, double X, double Rot) {
        /*
         * Declare the power variables for each wheel
         * Includes references to drive input variables
         * 'y' drive, 'x' strafe, and 'rot' rotation for mecanum-style
         * 'y' drive and 'rot' turn would only be needed for tank-style
         * 'mecanumDrive' will be called in the teleop file
         */

        /*
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
*/

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

} // end of class

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