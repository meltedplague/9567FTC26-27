
package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.Controllers.RampController;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Red 1 Tip", group = "Autonomous")
public class Red1Tip extends LinearOpMode {

    private Follower follower;
    private TurretController turret;
    private IntakeController intake;
    private RampController ramp;

    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    private static final double FLYWHEEL_VELOCITY = 1625;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose robotStartingspot = poseFactory.of(42, 7, 90);
    private final Pose path1 = poseFactory.of(34.5, 8.2, 180);
    private final Pose point2Start = poseFactory.of(14, 7.2, 180);
    private final Pose point2 = poseFactory.of(42, 10, 140);
    private final Pose point3Start = poseFactory.of(42, 16, 140);
    private final Pose point3 = poseFactory.of(34.5, 46.4115, 180);
    private final Pose point3Control1 = poseFactory.of(38.2133, 50.1304, 0);
    private final Pose point4 = poseFactory.of(42, 10, 122.9445);
    private final Pose point4Control1 = poseFactory.of(21.319, 47.9136, 0);
    private final Pose point5Start = poseFactory.of(42, 10, 140);
    private final Pose point5 = poseFactory.of(34, 12.0, 90);
    private final Pose point5Control1 = poseFactory.of(16.7021, 76.6657, 0);
    private final Pose point6 = poseFactory.of(47.5316, 125, 90);
    private final Pose point6Control1 = poseFactory.of(30.3363, 97.837, 0);
    private final Pose point7Start = poseFactory.of(47.5316, 128, 90);
    private final Pose point7 = poseFactory.of(24/*12.6279*/, 93.9808, 0);
    private final Pose point7Control1 = poseFactory.of(38.7456, 94.0947, 0);


    // Autonomous routine
    public Command autoRoutine() {

        return sequential(

                // Initial shooting period
                waitMs(500),
                instant(() -> intake.shoot()),
                waitMs(4000),

                // INTAKE #1 - Path 1 (Ramp UP)
                follow(follower, path1())
                        .raceWith(infinite(() -> intake.intake())),
                waitMs(2000),

                // Path 2 - Drive to shooting position
                follow(follower, path2()),
                instant(() -> intake.shoot()),
                waitMs(4000),

                // INTAKE #2 - Path 3 (Ramp DOWN)
                instant(() -> ramp.down()),
                follow(follower, path3())
                        .raceWith(infinite(() -> intake.intake())),
                waitMs(2000),

                // Path 4 - Drive to shooting position
                follow(follower, path4()),
                instant(() -> intake.shoot()),
                waitMs(2000),

                // Path 5 is currently skipped

                // INTAKE #3 - Path 6 (Ramp DOWN)
                instant(() -> ramp.down()),
                follow(follower, path6())
                        .raceWith(infinite(() -> intake.intake())),
                waitMs(2000),

                // Shooting period #4
                instant(() -> intake.shoot()),
                waitMs(1000),
                instant(() -> intake.stop()),

                // Path 7 (Ramp Up)
                instant(() -> ramp.up()),
                follow(follower, path7())
        );
    }


    // Flywheel commands
    public void startFlywheel() {
        flywheel1.setVelocity(FLYWHEEL_VELOCITY);
        flywheel2.setVelocity(FLYWHEEL_VELOCITY);
    }

    public void stopFlywheel() {
        flywheel1.setVelocity(0);
        flywheel2.setVelocity(0);
    }


    @Override
    public void runOpMode() {

        // Flywheel initialization
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Follower initialization
        follower = Constants.create(hardwareMap);
        follower.setPose(robotStartingspot);
        follower.update();

        // Turret initialization
        turret = new TurretController(hardwareMap);
        turret.setTarget(57.5, 60.0);

        // Intake initialization
        intake = new IntakeController(hardwareMap);

        // Ramp initialization (starts UP)
        ramp = new RampController(hardwareMap);
        ramp.up();

        waitForStart();

        if (isStopRequested()) return;

        // Start flywheels
        startFlywheel();

        // Start autonomous routine
        schedule(autoRoutine());

        while (opModeIsActive()) {

            follower.update();

            // Update turret position
            turret.update(
                    follower.pose().x(),
                    follower.pose().y(),
                    follower.pose().heading()
            );

            Scheduler.execute();

            // Drive telemetry
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            // Turret telemetry
            telemetry.addData("Turret Target Angle", turret.getTargetAngle());
            telemetry.addData("Turret Servo Position", turret.getServoPosition());
            telemetry.addData("Turret Reachable", turret.isTargetReachable());

            telemetry.update();
        }

        // Save final autonomous position for TeleOp
        OpModeStorage.autonomousEndPose = follower.pose();

        // Stop mechanisms when autonomous ends
        stopFlywheel();
        intake.stop();
        ramp.up();
    }


    // Path 1: Intake
    public Path path1() {
        return Paths.line(robotStartingspot, path1).constant(path1);
    }

    // Path 2: Drive to shooting position
    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    // Path 3: Intake from flower
    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3)
                .linear(point3Start, point3);
    }

    // Path 4: Drive to shooting position
    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).reverseTangent();
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5)
                .linear(point5Start, point5);
    }

    // Path 6: Intake
    public Path path6() {
        return Paths.curve(point5, point6Control1, point6)
                .constant(point6);
    }

    // Path 7
    public Path path7() {
        return Paths.curve(point7Start, point7Control1, point7)
                .linear(point7Start, point7);
    }
}
