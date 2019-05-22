package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ITS WORKING", group = "adsf")
public class WirelessDebugTest extends OpMode {
    public void init() {}
    public void loop() {
        telemetry.addLine("IT WORK");
        telemetry.update();
    }
}
