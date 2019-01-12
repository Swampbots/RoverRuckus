package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@TeleOp(name = "Lock Test", group ="Testing")
public class TestLock extends OpMode {

    RoverHardware hardware = new RoverHardware();

    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {

    }
}
