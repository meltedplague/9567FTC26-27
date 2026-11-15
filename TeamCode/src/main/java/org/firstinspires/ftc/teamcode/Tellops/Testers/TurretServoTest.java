package org.firstinspires.ftc.teamcode.Tellops.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret Servo Test")
public class TurretServoTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo turret1 = hardwareMap.get(Servo.class, "turret1");
        Servo turret2 = hardwareMap.get(Servo.class, "turret2");

        waitForStart();

        double position = 0.5;

        turret1.setPosition(position);
        turret2.setPosition(position);

        while (opModeIsActive()) {

            if (gamepad1.a) {
                position += 0.001;
            }

            if (gamepad1.b) {
                position -= 0.001;
            }

            position = Math.max(0.0, Math.min(1.0, position));

            turret1.setPosition(position);
            turret2.setPosition(position);

            telemetry.addData("Servo Position", position);
            telemetry.update();
        }
    }
}