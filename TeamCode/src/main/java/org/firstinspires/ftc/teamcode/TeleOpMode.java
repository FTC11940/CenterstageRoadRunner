package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

/**
 * FTC WIRES TeleOp Example
 *
 */

@TeleOp(name = "Cyber Coyotes TeleOp", group = "Teleop")
public class TeleOpMode extends LinearOpMode {

    // TODO test
    IO io = new IO(this);

    @Override
    public void runOpMode() throws InterruptedException {
        // multiplier for the driver motors
        double DRIVE_POWER = 1.0; // Adjust to driver comfort

        telemetry.addData("Initializing FTC Wires (ftcwires.org) TeleOp adopted for Team:","11940");

        telemetry.update();

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                telemetry.addData("Running FTC Wires (ftcwires.org) TeleOp Mode adopted for Team:","11940");
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * DRIVE_POWER,
                                -gamepad1.left_stick_x * DRIVE_POWER
                        ),
                        -gamepad1.right_stick_x * DRIVE_POWER
                ));

                // TODO assign buttons
                if(gamepad1.a) {
                    // turn on intake in forward direction
                    IO.intake.setPower(0.25);
                    break;
                }

                if(gamepad1.b) {
                    // reverse
                }

                drive.updatePoseEstimate();

                //telemetry.addData("LF Encoder", drive.leftFront.getCurrentPosition());
                //telemetry.addData("LB Encoder", drive.leftBack.getCurrentPosition());
                //telemetry.addData("RF Encoder", drive.rightFront.getCurrentPosition());
                //telemetry.addData("RB Encoder", drive.rightBack.getCurrentPosition());

                telemetry.addLine("Current Pose");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
                telemetry.update();
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * DRIVE_POWER,
                                0.0
                        ),
                        -gamepad1.right_stick_x * DRIVE_POWER
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();

            }
        } else {
            throw new AssertionError();
        }
    }
}