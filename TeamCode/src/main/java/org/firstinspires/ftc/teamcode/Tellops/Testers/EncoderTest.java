
package org.firstinspires.ftc.teamcode.Tellops.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret Encoder Test")
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Read the encoder through motor port 0
        DcMotor turretEncoder =
                hardwareMap.get(DcMotorEx.class, "lf");

        // Reset the encoder count
        turretEncoder.setMode(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        // Return to normal encoder-reading mode
        turretEncoder.setMode(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        telemetry.addLine("Encoder Test Ready");
        telemetry.addLine("Rotate turret by hand after pressing START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            int counts = turretEncoder.getCurrentPosition();

            telemetry.addData("Encoder Counts", counts);
            telemetry.addData("Absolute Counts", Math.abs(counts));

            telemetry.update();
        }
    }
}
