
package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.TurretController;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "blue Park Auto", group = "Autonomous")

public class BluePark extends LinearOpMode {

    // Pedro
    private Follower follower;

    // Turret
    private TurretController turret;

    // Pose factory
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Starting position
    private final Pose robotStart = poseFactory.of(7.5, 134.1, 0);

    // Parking position
    private final Pose parkPosition = poseFactory.of(12, 120, 0);

    // Turret target
    private final double targetX = 57.5;
    private final double targetY = 60.0;

    // --------------------------------
    // AUTONOMOUS ROUTINE
    // --------------------------------

    public Command autoRoutine() {
        return sequential(
                waitMs(1000),
                follow(follower, path1())
        );
    }

    // --------------------------------
    // PARKING PATH
    // --------------------------------

    public Path path1() {
        return Paths.line(robotStart, parkPosition)
                .linear(robotStart, parkPosition);
    }

    // --------------------------------
    // RUN AUTONOMOUS
    // --------------------------------

    @Override
    public void runOpMode() {

        // Initialize Pedro
        follower = Constants.create(hardwareMap);

        follower.setPose(robotStart);
        follower.update();

        // Initialize turret
        turret = new TurretController(hardwareMap);
        turret.setTarget(targetX, targetY);

        telemetry.addLine("Park Auto Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Start autonomous path
        autoRoutine().schedule();

        while (opModeIsActive()) {

            // Update Pedro
            follower.update();

            // Update turret tracking
            turret.update(
                    follower.pose().x(),
                    follower.pose().y(),
                    follower.pose().heading()
            );

            // Run Ivy scheduler
            Scheduler.execute();

            // Telemetry
            telemetry.addData("Robot X", follower.pose().x());
            telemetry.addData("Robot Y", follower.pose().y());
            telemetry.addData("Heading",
                    Math.toDegrees(follower.pose().heading()));

            telemetry.addData("Turret Target Angle",
                    turret.getTargetAngle());

            telemetry.addData("Turret Servo Position",
                    turret.getServoPosition());

            telemetry.addData("Turret Reachable",
                    turret.isTargetReachable());

            telemetry.update();
        }

        // Save final position for TeleOp
        OpModeStorage.autonomousEndPose = follower.pose();
    }
}

