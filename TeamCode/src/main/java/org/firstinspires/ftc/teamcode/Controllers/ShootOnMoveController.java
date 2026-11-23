
package org.firstinspires.ftc.teamcode.Controllers;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

public class ShootOnMoveController {

    // --------------------------------
    // CONFIGURATION
    // --------------------------------

    private static final double TIME_OF_FLIGHT = 0.92;

    // --------------------------------
    // CONTROLLERS
    // --------------------------------

    private final Follower follower;
    private final TurretController turret;

    // --------------------------------
    // FIELD TARGET
    // --------------------------------

    private double targetX = 57.5;
    private double targetY = 60.0;

    // --------------------------------
    // COMPENSATED TARGET
    // --------------------------------

    private double compensatedTargetX;
    private double compensatedTargetY;

    private double velocityX;
    private double velocityY;

    // --------------------------------
    // CONSTRUCTOR
    // --------------------------------

    public ShootOnMoveController(
            Follower follower,
            TurretController turret) {

        this.follower = follower;
        this.turret = turret;
    }

    // --------------------------------
    // SET TARGET
    // --------------------------------

    public void setTarget(double x, double y) {

        targetX = x;
        targetY = y;
    }

    // --------------------------------
    // MAIN UPDATE
    // --------------------------------

    public void update() {

        // Get robot position
        Pose pose = follower.pose();

        // Get robot velocity
        Velocity velocity = follower.velocity();

        velocityX = velocity.vx;
        velocityY = velocity.vy;

        // Calculate movement compensation
        compensatedTargetX =
                targetX - (velocityX * TIME_OF_FLIGHT);

        compensatedTargetY =
                targetY - (velocityY * TIME_OF_FLIGHT);

        // Send corrected target to turret
        turret.setTarget(
                compensatedTargetX,
                compensatedTargetY
        );

        // Update existing turret controller
        turret.update(
                pose.x(),
                pose.y(),
                pose.heading()
        );
    }

    // --------------------------------
    // GETTERS
    // --------------------------------

    public double getCompensatedTargetX() {
        return compensatedTargetX;
    }

    public double getCompensatedTargetY() {
        return compensatedTargetY;
    }

    public double getVelocityX() {
        return velocityX;
    }

    public double getVelocityY() {
        return velocityY;
    }

    public double getFlightTime() {
        return TIME_OF_FLIGHT;
    }
}
