
package org.firstinspires.ftc.teamcode.Tellops.Testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Ramp Position Tuner", group = "Test")
public class RampPositionTuner extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo ramp = hardwareMap.get(Servo.class, "ramp");

        double position = 0.5;

        boolean previousUp = false;
        boolean previousDown = false;

        ramp.setPosition(position);

        telemetry.addLine("Ramp Position Tuner Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean upPressed = gamepad1.dpad_up;
            boolean downPressed = gamepad1.dpad_down;

            if (upPressed && !previousUp) {
                position += 0.01;
            }

            if (downPressed && !previousDown) {
                position -= 0.01;
            }

            position = Math.max(0.0, Math.min(1.0, position));

            ramp.setPosition(position);

            previousUp = upPressed;
            previousDown = downPressed;

            telemetry.addData("Ramp Position", "%.2f", position);
            telemetry.addLine("D-pad UP: Increase position");
            telemetry.addLine("D-pad DOWN: Decrease position");
            telemetry.addData("Servo Position", ramp.getPosition());
            telemetry.update();

            sleep(20);
        }
    }
}
