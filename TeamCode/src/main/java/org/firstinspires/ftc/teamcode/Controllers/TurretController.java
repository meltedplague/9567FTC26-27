
package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretController {

    private Servo turret1;
    private Servo turret2;

    // --------------------------------
    // SERVO CALIBRATION
    // --------------------------------

    private static final double CENTER = 0.35;

    private static final double CW_LIMIT = 0.01;
    private static final double CCW_LIMIT = 0.99;

    // --------------------------------
    // MECHANICAL ANGLE LIMITS
    // --------------------------------

    // Clockwise is POSITIVE
    // Counterclockwise is NEGATIVE

    private static final double CW_ANGLE = 45.0;
    private static final double CCW_ANGLE = -270.0;

    // --------------------------------
    // MEASURED CALIBRATION POINT
    // --------------------------------

    // Servo position at 180 degrees CCW
    private static final double NEG_180_SERVO_POSITION = 0.887;

    // --------------------------------
    // FIELD TARGET
    // --------------------------------

    private double targetX = 57.5;
    private double targetY = 60.0;

    private double fieldBearing;
    private double targetAngle;

    private double servoPosition = CENTER;

    private boolean targetReachable = true;

    // --------------------------------
    // MANUAL OVERRIDE
    // --------------------------------

    private boolean manualOverride = false;
    private double manualAngle = 0.0;

    // --------------------------------
    // CONSTRUCTOR
    // --------------------------------

    public TurretController(HardwareMap hardwareMap) {

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

    }

    // --------------------------------
    // SET FIELD TARGET
    // --------------------------------

    public void setTarget(double x, double y) {

        targetX = x;
        targetY = y;
    }

    // --------------------------------
    // MANUAL CONTROL
    // --------------------------------

    public void setManualAngle(double angle) {

        manualOverride = true;
        manualAngle = angle;
    }

    public void setAutomatic() {

        manualOverride = false;
    }

    // --------------------------------
    // MAIN UPDATE
    // --------------------------------

    public void update(double robotX,
                       double robotY,
                       double robotHeadingRadians) {

        // --------------------------------
        // MANUAL OVERRIDE
        // --------------------------------

        if (manualOverride) {

            targetAngle = manualAngle;

            targetReachable =
                    targetAngle >= CCW_ANGLE &&
                            targetAngle <= CW_ANGLE;

            if (targetReachable) {
                calculateServoPosition();
            }

            return;
        }

        // --------------------------------
        // CALCULATE FIELD BEARING
        // --------------------------------

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        fieldBearing =
                Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading =
                Math.toDegrees(robotHeadingRadians);

        // --------------------------------
        // CLOCKWISE-POSITIVE ANGLE
        // --------------------------------

        double clockwiseAngle =
                normalize360(robotHeading - fieldBearing);

        // --------------------------------
        // SELECT MECHANICAL ROTATION
        // --------------------------------

        if (clockwiseAngle <= CW_ANGLE) {

            // Clockwise rotation
            targetAngle = clockwiseAngle;

        } else if (clockwiseAngle >= 90.0) {

            // Counterclockwise rotation
            targetAngle = clockwiseAngle - 360.0;

        } else {

            // Target is inside the mechanical blind zone
            targetAngle = clockwiseAngle;
        }

        // --------------------------------
        // CHECK MECHANICAL LIMITS
        // --------------------------------

        targetReachable =
                targetAngle >= CCW_ANGLE &&
                        targetAngle <= CW_ANGLE;

        if (!targetReachable) {
            return;
        }

        // --------------------------------
        // CALCULATE SERVO POSITION
        // --------------------------------

        calculateServoPosition();
    }

    // --------------------------------
    // SERVO POSITION CALCULATION
    // --------------------------------

    private void calculateServoPosition() {

        if (targetAngle < 0) {

            // --------------------------------
            // COUNTERCLOCKWISE
            // --------------------------------

            if (targetAngle >= -180.0) {

                // Center to -180 degrees

                servoPosition = CENTER +
                        (targetAngle / -180.0) *
                                (NEG_180_SERVO_POSITION - CENTER);

            } else {

                // -180 to -270 degrees

                servoPosition = NEG_180_SERVO_POSITION +
                        ((targetAngle + 180.0) / -90.0) *
                                (CCW_LIMIT - NEG_180_SERVO_POSITION);
            }

        } else {

            // --------------------------------
            // CLOCKWISE
            // --------------------------------

            // Center to +45 degrees

            servoPosition = CENTER +
                    (targetAngle / CW_ANGLE) *
                            (CW_LIMIT - CENTER);
        }

        setServoPosition(servoPosition);
    }

    // --------------------------------
    // SERVO OUTPUT
    // --------------------------------

    private void setServoPosition(double position) {

        position = Math.max(0.0, Math.min(1.0, position));

        servoPosition = position;

        turret1.setPosition(position);
        turret2.setPosition(position);
    }

    // --------------------------------
    // NORMALIZE ANGLE TO 0 - 360
    // --------------------------------

    private double normalize360(double angle) {

        while (angle < 0) {
            angle += 360.0;
        }

        while (angle >= 360) {
            angle -= 360.0;
        }

        return angle;
    }

    // --------------------------------
    // GETTERS
    // --------------------------------

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
