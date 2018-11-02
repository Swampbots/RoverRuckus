package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pivot Test", group = "Testing")
public class RoverPivotTest extends OpMode {

    RoverHardware hardware = new RoverHardware();

    public void init() {
        hardware.init(hardwareMap);
    }


    public void loop() {



    }
}
