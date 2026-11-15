
package org.firstinspires.ftc.teamcode.Tellops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.FieldOrientedDriveController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.Controllers.RegressionFlywheelController;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Turret Tellop", group = "TeleOp")

public class TurretTellop2027 extends LinearOpMode {

    // --------------------------------
    // PEDRO
    // --------------------------------

    private Follower follower;
    private FieldOrientedDriveController driveController;

    // --------------------------------
    // MECHANISM CONTROLLERS
    // --------------------------------

    private TurretController turret;
    private IntakeController intakeController;
    private RegressionFlywheelController regressionFlywheelController;

    // --------------------------------
    // TARGET SELECTION
    // --------------------------------

    private boolean manualTarget = false;

    private double selectedGoalX = 57.5;
    private double selectedGoalY = 60.0;

    // --------------------------------
    // BUTTON EDGE DETECTION
    // --------------------------------

    private boolean previousX = false;
    private boolean previousB = false;
    private boolean previousY = false;

    private boolean previousReset = false;
    private boolean previousDpadDown = false;

    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;
    private boolean previousDpadUp = false;

    @Override
    public void runOpMode() {

        // --------------------------------
        // INITIALIZE PEDRO
        // --------------------------------

        follower = Constants.create(hardwareMap);

        driveController =
                new FieldOrientedDriveController(follower);

        // --------------------------------
        // INITIALIZE MECHANISMS
        // --------------------------------

        turret = new TurretController(hardwareMap);
        turret.setTarget(57.5, 60.0);

        intakeController = new IntakeController(hardwareMap);

        regressionFlywheelController =
                new RegressionFlywheelController(hardwareMap);

        telemetry.addLine("Turret Tellop Initializing...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Load final position from Autonomous
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();

        // --------------------------------
        // MAIN TELEOP LOOP
        // --------------------------------

        while (opModeIsActive()) {

            // --------------------------------
            // GAMEPAD 1: FIELD-ORIENTED DRIVING
            // --------------------------------

            driveController.drive(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

            // --------------------------------
            // RESET FIELD HEADING
            // D-PAD UP
            // --------------------------------

            if (gamepad1.dpad_up && !previousReset) {
                follower.setHeading(0.0);
            }

            previousReset = gamepad1.dpad_up;

            // --------------------------------
            // RESET LOCALIZATION
            // D-PAD DOWN
            // --------------------------------

            if (gamepad1.dpad_down && !previousDpadDown) {

                follower.setPose(
                        new Pose(8.0, 57.5, 0.0)
                );
            }

            previousDpadDown = gamepad1.dpad_down;

            // --------------------------------
            // UPDATE PEDRO LOCALIZATION
            // --------------------------------

            follower.update();

            double robotX = follower.pose().x();
            double robotY = follower.pose().y();
            double heading = follower.pose().heading();

            // --------------------------------
            // GAMEPAD 2: TARGET SELECTION
            // --------------------------------

            boolean xPressed = gamepad2.x;
            boolean bPressed = gamepad2.b;
            boolean yPressed = gamepad2.y;

            // X: Manual lower target
            if (xPressed && !previousX) {

                manualTarget = true;

                selectedGoalX = 57.5;
                selectedGoalY = 60.0;
            }

            // B: Manual upper target
            if (bPressed && !previousB) {

                manualTarget = true;

                selectedGoalX = 57.5;
                selectedGoalY = 80.0;
            }

            // Y: Automatic target selection
            if (yPressed && !previousY) {
                manualTarget = false;
            }

            previousX = xPressed;
            previousB = bPressed;
            previousY = yPressed;

            // --------------------------------
            // SELECT ACTIVE TARGET
            // --------------------------------

            double goalX;
            double goalY;

            if (manualTarget) {

                goalX = selectedGoalX;
                goalY = selectedGoalY;

            } else if (robotY >= 72.0) {

                goalX = 57.5;
                goalY = 80.0;

            } else {

                goalX = 57.5;
                goalY = 60.0;
            }

            // --------------------------------
            // INTAKE AND FEEDER CONTROL
            // --------------------------------

            if (gamepad1.right_trigger > 0.1) {

                intakeController.intake();

            } else if (gamepad1.left_trigger > 0.1) {

                intakeController.outtake();

            } else if (gamepad1.a) {

                intakeController.shoot();

            } else {

                intakeController.stop();
            }

            // --------------------------------
            // TURRET MANUAL OVERRIDES
            // --------------------------------

            // D-pad LEFT: Turret to 0 degrees
            if (gamepad2.dpad_left && !previousDpadLeft) {
                turret.setManualAngle(0.0);
            }

            // D-pad RIGHT: Turret to 180 degrees
            if (gamepad2.dpad_right && !previousDpadRight) {
                turret.setManualAngle(-180.0);
            }

            // D-pad UP: Return to automatic targeting
            if (gamepad2.dpad_up && !previousDpadUp) {
                turret.setAutomatic();
            }

            previousDpadLeft = gamepad2.dpad_left;
            previousDpadRight = gamepad2.dpad_right;
            previousDpadUp = gamepad2.dpad_up;

            // --------------------------------
            // FLYWHEEL CONTROL
            // --------------------------------

            // Y: Calculate target velocity from goal distance
            if (gamepad1.y) {

                regressionFlywheelController.shootAtGoal(
                        robotX,
                        robotY,
                        goalX,
                        goalY
                );

            }

            // X: Stop flywheels
            else if (gamepad1.x) {

                regressionFlywheelController.stop();
            }

            // --------------------------------
            // BANG-BANG FLYWHEEL UPDATE
            // --------------------------------

            // Continuously monitors flywheel velocity.
            // Uses 100% power for recovery.
            // Switches back to velocity control
            // within 30 ticks/sec of target.

            regressionFlywheelController.updateBangBangController();

            // --------------------------------
            // UPDATE TURRET
            // --------------------------------

            turret.setTarget(goalX, goalY);

            turret.update(
                    robotX,
                    robotY,
                    heading
            );

            // --------------------------------
            // DISTANCE CALCULATION FOR TELEMETRY
            // --------------------------------

            double goalDistance =
                    regressionFlywheelController.getDistanceToGoal(
                            robotX,
                            robotY,
                            goalX,
                            goalY
                    );

            // --------------------------------
            // TELEMETRY
            // --------------------------------

            telemetry.addLine("=== ROBOT POSITION ===");

            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Heading", Math.toDegrees(heading));

            telemetry.addLine("=== TARGET ===");

            telemetry.addData("Manual Target", manualTarget);
            telemetry.addData("Active Goal X", goalX);
            telemetry.addData("Active Goal Y", goalY);
            telemetry.addData("Distance to Goal", "%.2f", goalDistance);

            telemetry.addLine("=== TURRET ===");

            telemetry.addData("Turret Target Angle", turret.getTargetAngle());
            telemetry.addData("Turret Servo Position", turret.getServoPosition());
            telemetry.addData("Target Reachable", turret.isTargetReachable());
            telemetry.addData("Turret Manual", turret.isManualOverride());

            telemetry.addLine("=== FLYWHEELS ===");

            telemetry.addData("Target Velocity", regressionFlywheelController.getTargetVelocity());
            telemetry.addData("Flywheel 1 Actual", regressionFlywheelController.getFlywheel1Velocity());
            telemetry.addData("Flywheel 2 Actual", regressionFlywheelController.getFlywheel2Velocity());

            telemetry.update();
        }

        // --------------------------------
        // STOP MECHANISMS
        // --------------------------------

        regressionFlywheelController.stop();
        intakeController.stop();
        driveController.stop();
    }
}
