/**
 * TODO
 * Team:
 * Year:
 * Title:
 * Summary:
 **/

 // Naming conventions https://docs.oracle.com/javase/tutorial/java/nutsandbolts/variables.html //

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Constants {

    // multiplier for the driver motors
    public static final double DRIVE_POWER = 1.0; // Adjust to driver comfort
    // public static double SLOW_DOWN_FACTOR = 1.0; //

    // Drivetrain information needed for accurate auton
    public static final double COUNTS_PER_MOTOR_REV = 537.7 ; // GoBILDA 312 RPM Yellow Jacket
    public static final double DRIVE_GEAR_REDUCTION = 1.0 ; // No External Gearing
    public static final double WHEEL_DIAMETER_INCHES = 4.01575 ; // For figuring circumference
    public static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
     *
     * Two input parameters are required to fully specify the Orientation.
     * The first parameter specifies the direction the printed logo on the Hub is pointing.
     * The second parameter specifies the direction the USB connector on the Hub is pointing.
     * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
     */

    /* The next two lines define Hub orientation.
     * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
     *
     * To Do:  EDIT these two lines to match YOUR mounting configuration.
     */
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;

    // Practice Bot is control up is turned right
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;


}
