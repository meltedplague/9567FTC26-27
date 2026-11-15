
package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;
import java.util.TreeMap;

public class InterpolatingFlywheelController {

    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    //todo IMPORTANT: Set this to encoder ticks per flywheel output revolution.
    private static final double MOTOR_TICKS_PER_REV = 1.0; // CHANGE THIS

    // Goal coordinates in Pedro's field coordinate system (inches).
    private double goalX = 57.5;
    private double goalY = 60.0;

    // Distance in inches | Flywheel speed in RPM
    private final TreeMap<Double, Double> launchFlywheelSpeedMap =
            new TreeMap<>();

    private double distanceToGoal;
    private double targetRPM;

    public InterpolatingFlywheelController(HardwareMap hardwareMap) {

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        // FLYWHEEL TUNING VALUES: Inches | RPM
        launchFlywheelSpeedMap.put(12.0, 1000.0);
        launchFlywheelSpeedMap.put(16.0, 1100.0);
        launchFlywheelSpeedMap.put(20.0, 1250.0);
        launchFlywheelSpeedMap.put(25.0, 1380.0);
        launchFlywheelSpeedMap.put(30.0, 1400.0);
        launchFlywheelSpeedMap.put(32.0, 1560.0);
        launchFlywheelSpeedMap.put(35.0, 1600.0);
        launchFlywheelSpeedMap.put(38.0, 1740.0);
        launchFlywheelSpeedMap.put(42.0, 1800.0);
        launchFlywheelSpeedMap.put(43.0, 1930.0);
        launchFlywheelSpeedMap.put(44.0, 2000.0);
        launchFlywheelSpeedMap.put(45.0, 2100.0);
        launchFlywheelSpeedMap.put(47.0, 2250.0);
    }

    // Set the goal's field coordinates.
    public void setGoalPosition(double x, double y) {
        goalX = x;
        goalY = y;
    }

    // Call repeatedly inside your TeleOp loop.
    public void update(double robotX, double robotY) {

        double dx = goalX - robotX;
        double dy = goalY - robotY;

        distanceToGoal = Math.hypot(dx, dy);

        targetRPM = getInterpolatedRPM(distanceToGoal);

        double targetTicksPerSecond =
                targetRPM * MOTOR_TICKS_PER_REV / 60.0;

        flywheel1.setVelocity(targetTicksPerSecond);
        flywheel2.setVelocity(targetTicksPerSecond);

    }

    // Linear interpolation between nearest distances.
    private double getInterpolatedRPM(double distance) {

        if (launchFlywheelSpeedMap.isEmpty()) {
            return 0;
        }

        Map.Entry<Double, Double> lower =
                launchFlywheelSpeedMap.floorEntry(distance);

        Map.Entry<Double, Double> upper =
                launchFlywheelSpeedMap.ceilingEntry(distance);

        if (lower == null) {
            return launchFlywheelSpeedMap.firstEntry().getValue();
        }

        if (upper == null) {
            return launchFlywheelSpeedMap.lastEntry().getValue();
        }

        if (lower.getKey().equals(upper.getKey())) {
            return lower.getValue();
        }

        double lowerDistance = lower.getKey();
        double upperDistance = upper.getKey();

        double lowerRPM = lower.getValue();
        double upperRPM = upper.getValue();

        double ratio =
                (distance - lowerDistance) /
                        (upperDistance - lowerDistance);

        return lowerRPM + ratio * (upperRPM - lowerRPM);
    }

    public void stop() {
        flywheel1.setVelocity(0);
        flywheel2.setVelocity(0);
        targetRPM = 0;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getActualRPM() {

        double rpm1 = Math.abs(flywheel1.getVelocity())
                * 60.0 / MOTOR_TICKS_PER_REV;

        double rpm2 = Math.abs(flywheel2.getVelocity())
                * 60.0 / MOTOR_TICKS_PER_REV;

        return (rpm1 + rpm2) / 2.0;
    }
}
