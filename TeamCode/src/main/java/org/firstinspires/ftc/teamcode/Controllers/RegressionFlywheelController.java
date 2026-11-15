
package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class RegressionFlywheelController {

    private static final double RECOVERY_TOLERANCE = 50;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    // =================================
    // REGRESSION EQUATION
    // =================================

    private double calculateVelocity(double distance) {

        // i'm using LINEAR REGRESSION right now but i'll change it later probably
        return 32.4513 * distance + 552.952;
    }

    private static final double MIN_DISTANCE = 12.0;
    private static final double MAX_DISTANCE = 47.0;

    private double targetVelocity = 0;

    //goal target
    double goalX = 57.5;
    double goalY = 60.0;

    public RegressionFlywheelController(HardwareMap hardwareMap) {

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shootFromDistance(double distance) {

        distance = Math.max(MIN_DISTANCE,
                Math.min(MAX_DISTANCE, distance));

        targetVelocity = calculateVelocity(distance);
    }

    public void updateBangBangController() {

        if (targetVelocity <= 0) {
            flywheel1.setPower(0);
            flywheel2.setPower(0);
            return;
        }

        double averageVelocity =
                (flywheel1.getVelocity() + flywheel2.getVelocity()) / 2.0;

        double velocityError = targetVelocity - averageVelocity;

        if (velocityError > RECOVERY_TOLERANCE) {

            flywheel1.setPower(1.0);
            flywheel2.setPower(1.0);

        } else {

            flywheel1.setVelocity(targetVelocity);
            flywheel2.setVelocity(targetVelocity);
        }
    }

    public double getDistanceToGoal(
            double robotX,
            double robotY,
            double goalX,
            double goalY) {

        double dx = goalX - robotX;
        double dy = goalY - robotY;

        return Math.sqrt(dx * dx + dy * dy);
    }

    public void shootAtGoal(
            double robotX,
            double robotY,
            double goalX,
            double goalY) {

        double distance = getDistanceToGoal(
                robotX, robotY, goalX, goalY);

        shootFromDistance(distance);
    }

    public void stop() {
        targetVelocity = 0;
        flywheel1.setVelocity(0);
        flywheel2.setVelocity(0);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getFlywheel1Velocity() {
        return flywheel1.getVelocity();
    }

    public double getFlywheel2Velocity() {
        return flywheel2.getVelocity();
    }
}
