package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Flipper Test", group = "Testing")
public class FlipperTest extends OpMode {

    RoverHardware hardware = new RoverHardware();

    private final double FLIPPER_SPEED_BASE = 0.5;


    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {

        
    }
}
