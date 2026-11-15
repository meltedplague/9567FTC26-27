
package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RampController {

    private Servo ramp;

    private double rampPosition = 0.0;

    public RampController(HardwareMap hardwareMap) {
        ramp = hardwareMap.get(Servo.class, "ramp");
        setPosition(0.0);
    }

    public void setPosition(double position) {
        rampPosition = Math.max(0.0, Math.min(1.0, position));
        ramp.setPosition(rampPosition);
    }

    public void up() {
        setPosition(0.0);
    }

    public void down() {
        setPosition(1.0);
    }

    public double getPosition() {
        return rampPosition;
    }
}
