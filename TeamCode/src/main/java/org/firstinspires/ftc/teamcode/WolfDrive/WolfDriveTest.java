package org.firstinspires.ftc.teamcode.WolfDrive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(group = "Test")
public class WolfDriveTest extends LinearOpMode {
    public static double TURN_SPEED = 0.75;
    public static double SLOW_TURN_SPEED = 0.3;

    private MecanumDrive baseDrive;
    private WolfDrive drive;

    private double speed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Init
        baseDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive = new WolfDrive(baseDrive);

        // Ready!
        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();

        // On start
        if (opModeIsActive()) {
            resetRuntime();
        }

        // Main loop
        while (opModeIsActive()) {
            double input_x = Math.pow(-gamepad1.left_stick_y, 3) * speed;
            double input_y = Math.pow(-gamepad1.left_stick_x, 3) * speed;
            Vector2d input = new Vector2d(input_x, input_y);

            double input_turn = Math.pow(gamepad1.left_trigger - gamepad1.right_trigger, 3) * TURN_SPEED // Turn via triggers
                              + Math.pow(-gamepad1.right_stick_x, 3) * TURN_SPEED; // Turn via right stick
            if (gamepad1.left_bumper) input_turn += SLOW_TURN_SPEED;
            if (gamepad1.right_bumper) input_turn -= SLOW_TURN_SPEED;
            input_turn = Range.clip(input_turn, -1, 1);

            PoseVelocity2d currentVel = baseDrive.updatePoseEstimate();
            drive.trackPosition(baseDrive.pose);
            drive.driveWithCorrection(new PoseVelocity2d(input, input_turn), currentVel);

            telemetry.addData("Circle stats", drive.getCircleString());
            telemetry.addData("Stick", drive.formatVector(input));
            telemetry.addData("Current vel", drive.formatVector(currentVel.linearVel));
            telemetry.addData("Correction", drive.getCorrectionString());
            telemetry.addData("Drive direction", drive.getDriveDirectionString());
            telemetry.addLine(drive.getWheelPowerString());
        }
    }
}