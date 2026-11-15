
package org.firstinspires.ftc.teamcode.Tellops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Controllers.FieldOrientedDriveController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Tellop Bad 2027", group = "TeleOp")

public class TellopBad2027 extends LinearOpMode {

//    Driving Controls
//
//    Gamepad 1 Y: Flywheels run at 1400 ticks/sec.
//    Gamepad 1 X: Stops flywheels.
//    Gamepad 1 D-pad Up: Resets heading to 0°.
//    Gamepad 1 D-pad Down: Resets Pedro position to (8, 57.5, 0°).
//    Gamepad 1 Right Trigger: Intake.
//    Gamepad 1 Left Trigger: Outtake.
//    Gamepad 1 A: Shoot.
//    Gamepad 2 X: Lower target.
//    Gamepad 2 B: Upper target.
//    Gamepad 2 Y: Automatic target selection.
//    Gamepad 2 D-pad Left/Right: Turret manual positioning.
//    Gamepad 2 D-pad Up: Turret automatic mode.


    // Pedro
    private Follower follower;
    private FieldOrientedDriveController driveController;

    // Mechanism controllers
    private TurretController turret;
    private IntakeController intakeController;

    // Flywheels
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    private static final double FLYWHEEL_VELOCITY = 1625;

    // Target selection
    private boolean manualTarget = false;

    private double selectedGoalX = 57.5;
    private double selectedGoalY = 60.0;

    // Button edge detection
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
        // INITIALIZE FLYWHEELS
        // --------------------------------

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        telemetry.addLine("Pedro initializing...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // This Loads the final position from Autonomous
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

            // Update Pedro localization
            follower.update();

            // Get robot position
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

            // Y: Return to automatic mode
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

            if (gamepad1.y) {

                flywheel1.setVelocity(FLYWHEEL_VELOCITY);
                flywheel2.setVelocity(FLYWHEEL_VELOCITY);

            } else if (gamepad1.x) {

                flywheel1.setVelocity(0);
                flywheel2.setVelocity(0);
            }

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
            // TELEMETRY
            // --------------------------------

            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Heading", Math.toDegrees(heading));

            telemetry.addData("Manual Override", manualTarget);
            telemetry.addData("Active Goal X", goalX);
            telemetry.addData("Active Goal Y", goalY);

            telemetry.addData("Turret Target Angle",
                    turret.getTargetAngle());

            telemetry.addData("Turret Servo Position",
                    turret.getServoPosition());

            telemetry.addData("Target Reachable",
                    turret.isTargetReachable());

            telemetry.addData("Turret Manual",
                    turret.isManualOverride());

            telemetry.addData("Flywheel Velocity",
                    flywheel1.getVelocity());

            telemetry.addData("Flywheel 1 Target", FLYWHEEL_VELOCITY);
            telemetry.addData("Flywheel 1 Actual", flywheel1.getVelocity());

            telemetry.addData("Flywheel 2 Target", FLYWHEEL_VELOCITY);
            telemetry.addData("Flywheel 2 Actual", flywheel2.getVelocity());

            telemetry.update();
        }

        // --------------------------------
        // STOP MECHANISMS
        // --------------------------------

        flywheel1.setVelocity(0);
        flywheel2.setVelocity(0);

        intakeController.stop();
        driveController.stop();
    }
}
