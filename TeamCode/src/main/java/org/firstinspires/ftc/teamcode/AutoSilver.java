package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name = "Silver", group = "Autonomous")
public class AutoSilver extends OpMode {

    RoverHardware hardware = new RoverHardware();

    public final double RAMP_STOWED = 0.0;
    public final double LOCK_LOCKED = 0.0;

    public void init() {
        hardware.init(hardwareMap);

    }

    public void init_loop() {

    }

    public void start() {

    }

    public void loop() {

    }



}
