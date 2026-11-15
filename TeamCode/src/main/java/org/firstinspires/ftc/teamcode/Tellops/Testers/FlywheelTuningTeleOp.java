
package org.firstinspires.ftc.teamcode.Tellops.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel Tuning", group = "Testing")
public class FlywheelTuningTeleOp extends LinearOpMode {

    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_INCREMENT = 25.0;

    private double targetRPM = 0;

    private boolean previousA = false;
    private boolean previousB = false;

    @Override
    public void runOpMode() {

        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && !previousA) {
                targetRPM += RPM_INCREMENT;
            }

            if (gamepad1.b && !previousB) {
                targetRPM = Math.max(0, targetRPM - RPM_INCREMENT);
            }

            if (gamepad1.y) {
                targetRPM = 0;
            }

            previousA = gamepad1.a;
            previousB = gamepad1.b;

            double targetTicksPerSecond =
                    targetRPM * TICKS_PER_REV / 60.0;

            flywheel1.setVelocity(targetTicksPerSecond);
            flywheel2.setVelocity(targetTicksPerSecond);

            double rpm1 = Math.abs(flywheel1.getVelocity())
                    * 60.0 / TICKS_PER_REV;

            double rpm2 = Math.abs(flywheel2.getVelocity())
                    * 60.0 / TICKS_PER_REV;

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", (rpm1 + rpm2) / 2.0);
            telemetry.update();
        }

        flywheel1.setVelocity(0);
        flywheel2.setVelocity(0);
    }
}
