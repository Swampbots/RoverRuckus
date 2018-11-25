package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_REV_HD;

@TeleOp(name = "Flipper Test", group = "Testing")
public class FlipperTest extends OpMode {

    RoverHardware hardware = new RoverHardware();

    private final double FLIPPER_SPEED_BASE = 0.5;
    private final int COUNTS_PER_REV_HD_20 = COUNTS_PER_REV_HD / 2; // Converting from a 40:1 motor to a 20:1
    private final double GEAR_REDUCTION_FLIPPER = 1.0; // 1:1 gear ratio
    private final int COUNTS_PER_DEGREE_REV_HD_20 = (int)((COUNTS_PER_REV_HD_20 * GEAR_REDUCTION_FLIPPER) / 480);



    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {

        
    }
}
