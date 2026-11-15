
package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeController {

    private DcMotor intake;
    private DcMotor feeder;

    public IntakeController(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotor.class, "intake");
        feeder = hardwareMap.get(DcMotor.class, "feeder");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    // Intake: intake forward, feeder backward
    public void intake() {
        intake.setPower(1.0);
        feeder.setPower(-0.75);
    }

    // Outtake: both backward
    public void outtake() {
        intake.setPower(-1.0);
        feeder.setPower(-1.0);
    }

    // Shoot: both forward
    public void shoot() {
        intake.setPower(1.0);
        feeder.setPower(1.0);
    }

    // Stop both motors
    public void stop() {
        intake.setPower(0);
        feeder.setPower(0);
    }
}
