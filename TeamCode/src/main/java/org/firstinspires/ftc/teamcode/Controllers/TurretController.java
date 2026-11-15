
package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretController {

    private Servo turret1;
    private Servo turret2;

    // Servo calibration
    private static final double CENTER = 0.8;
    private static final double CW_LIMIT = 0.01;
    private static final double CCW_LIMIT = 0.99;

    // Turret mechanical angle limits
    private static final double CW_ANGLE = -240.0;
    private static final double CCW_ANGLE = 20.0;

    // Measured calibration point
    private static final double NEG_180_SERVO_POSITION = 0.29;

    // Field target
    private double targetX = 57.5;
    private double targetY = 60.0;

    private double fieldBearing;
    private double targetAngle;
    private double servoPosition = CENTER;

    private boolean targetReachable = true;

    // Manual override
    private boolean manualOverride = false;
    private double manualAngle = 0.0;

    public TurretController(HardwareMap hardwareMap) {
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        setServoPosition(CENTER);
    }

    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    public void setManualAngle(double angle) {
        manualOverride = true;
        manualAngle = angle;
    }

    public void setAutomatic() {
        manualOverride = false;
    }

    public void update(double robotX, double robotY,
                       double robotHeadingRadians) {

        if (manualOverride) {
            targetAngle = manualAngle;
            targetReachable =
                    targetAngle >= CW_ANGLE &&
                            targetAngle <= CCW_ANGLE;

            if (targetReachable) {
                calculateServoPosition();
            }
            return;
        }

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        fieldBearing = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading =
                Math.toDegrees(robotHeadingRadians);

        double angle = normalizeAngle(fieldBearing - robotHeading);

        if (angle > CCW_ANGLE) {
            angle -= 360.0;
        }

        targetAngle = angle;

        targetReachable =
                targetAngle >= CW_ANGLE &&
                        targetAngle <= CCW_ANGLE;

        if (!targetReachable) {
            return;
        }

        calculateServoPosition();
    }

    private void calculateServoPosition() {

        if (targetAngle <= 0) {

            if (targetAngle >= -180.0) {
                servoPosition = CENTER +
                        (targetAngle / -180.0) *
                                (NEG_180_SERVO_POSITION - CENTER);

            } else {
                servoPosition = NEG_180_SERVO_POSITION +
                        ((targetAngle + 180.0) / -60.0) *
                                (CW_LIMIT - NEG_180_SERVO_POSITION);
            }

        } else {
            servoPosition = CENTER +
                    (targetAngle / CCW_ANGLE) *
                            (CCW_LIMIT - CENTER);
        }

        setServoPosition(servoPosition);
    }

    private void setServoPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position));

        servoPosition = position;
        turret1.setPosition(position);
        turret2.setPosition(position);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getFieldBearing() {
        return fieldBearing;
    }

    public double getServoPosition() {
        return servoPosition;
    }

    public boolean isTargetReachable() {
        return targetReachable;
    }

    public boolean isManualOverride() {
        return manualOverride;
    }
}
