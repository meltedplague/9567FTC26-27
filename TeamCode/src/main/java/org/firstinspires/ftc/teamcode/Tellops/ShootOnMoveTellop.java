
package org.firstinspires.ftc.teamcode.Tellops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controllers.FieldOrientedDriveController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.Controllers.ShootOnMoveController;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Shoot On The Move Tellop", group = "TeleOp")

public class ShootOnMoveTellop extends LinearOpMode {

    // --------------------------------
    // PEDRO
    // --------------------------------

    private Follower follower;
    private FieldOrientedDriveController driveController;

    // --------------------------------
    // MECHANISM CONTROLLERS
    // --------------------------------

    private TurretController turret;
    private ShootOnMoveController shootOnMove;
    private IntakeController intakeController;

    // --------------------------------
    // FLYWHEELS
    // --------------------------------

    private DcMotorEx flywheel1;

    private static final double FLYWHEEL_VELOCITY = 1525;

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
        // INITIALIZE FLYWHEELS
        // --------------------------------

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");

        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        shootOnMove = new ShootOnMoveController(
                follower,
                turret
        );

        intakeController = new IntakeController(hardwareMap);

        telemetry.addLine("Pedro initializing...");
        telemetry.addLine("Shoot On Move initializing...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --------------------------------
        // LOAD AUTONOMOUS END POSITION
        // --------------------------------

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
            // UPDATE PEDRO
            // --------------------------------

            follower.update();

            // --------------------------------
            // GET ROBOT POSITION
            // --------------------------------

            double robotX = follower.pose().x();
            double robotY = follower.pose().y();
            double heading = follower.pose().heading();

            // --------------------------------
            // GAMEPAD 2: TARGET SELECTION
            // --------------------------------

            boolean xPressed = gamepad2.x;
            boolean bPressed = gamepad2.b;
            boolean yPressed = gamepad2.y;

            // X: LOWER TARGET

            if (xPressed && !previousX) {

                manualTarget = true;

                selectedGoalX = 57.5;
                selectedGoalY = 60.0;
            }

            // B: UPPER TARGET

            if (bPressed && !previousB) {

                manualTarget = true;

                selectedGoalX = 57.5;
                selectedGoalY = 80.0;
            }

            // Y: AUTOMATIC TARGET SELECTION

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

            // D-PAD LEFT: TURRET TO 0 DEGREES

            if (gamepad2.dpad_left && !previousDpadLeft) {

                turret.setManualAngle(0.0);
            }

            // D-PAD RIGHT: TURRET TO -180 DEGREES

            if (gamepad2.dpad_right && !previousDpadRight) {

                turret.setManualAngle(-180.0);
            }

            // D-PAD UP: AUTOMATIC TURRET MODE

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

                if (flywheel1.getVelocity() > FLYWHEEL_VELOCITY - 40) {

                    flywheel1.setVelocity(FLYWHEEL_VELOCITY);

                } else {

                    flywheel1.setPower(1);
                }

            } else if (gamepad1.x) {

                flywheel1.setVelocity(0);
            }

            // --------------------------------
            // SHOOT ON MOVE
            // --------------------------------

            shootOnMove.setTarget(goalX, goalY);

            if (turret.isManualOverride()) {

                turret.update(
                        robotX,
                        robotY,
                        heading
                );

            } else {

                shootOnMove.update();
            }

            // --------------------------------
            // TELEMETRY
            // --------------------------------

            telemetry.addLine("=== ROBOT POSITION ===");

            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Heading", Math.toDegrees(heading));

            telemetry.addLine("=== TARGET ===");

            telemetry.addData("Manual Target", manualTarget);
            telemetry.addData("Goal X", goalX);
            telemetry.addData("Goal Y", goalY);

            telemetry.addLine("=== SHOOT ON MOVE ===");

            telemetry.addData("Robot VX",
                    shootOnMove.getVelocityX());

            telemetry.addData("Robot VY",
                    shootOnMove.getVelocityY());

            telemetry.addData("Compensated X",
                    shootOnMove.getCompensatedTargetX());

            telemetry.addData("Compensated Y",
                    shootOnMove.getCompensatedTargetY());

            telemetry.addLine("=== TURRET ===");

            telemetry.addData("Target Angle",
                    turret.getTargetAngle());

            telemetry.addData("Servo Position",
                    turret.getServoPosition());

            telemetry.addData("Target Reachable",
                    turret.isTargetReachable());

            telemetry.addData("Manual Override",
                    turret.isManualOverride());

            telemetry.addLine("=== FLYWHEEL ===");

            telemetry.addData("Flywheel Target",
                    FLYWHEEL_VELOCITY);

            telemetry.addData("Flywheel Actual",
                    flywheel1.getVelocity());

            telemetry.update();
        }

        // --------------------------------
        // STOP MECHANISMS
        // --------------------------------

        flywheel1.setVelocity(0);

        intakeController.stop();

        driveController.stop();
    }
}
