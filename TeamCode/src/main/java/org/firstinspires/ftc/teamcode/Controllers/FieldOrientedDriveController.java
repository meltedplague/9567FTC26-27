
package org.firstinspires.ftc.teamcode.Controllers;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.drivetrain.DrivePowers;

public class FieldOrientedDriveController {

    private final Follower follower;

    public FieldOrientedDriveController(Follower follower) {
        this.follower = follower;
    }

    // Field-oriented driving with automatic position holding
    public void drive(double strafe, double forward, double turn) {

        // Convert joystick inputs into field-centric drive powers
        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                -strafe,
                turn,
                follower.pose().heading()
        );

        // Automatically hold position when sticks are released
        ManualDrive.driveOrHold(follower, powers);
    }

    // Stop the drivetrain
    public void stop() {
        follower.stop();
    }
}
