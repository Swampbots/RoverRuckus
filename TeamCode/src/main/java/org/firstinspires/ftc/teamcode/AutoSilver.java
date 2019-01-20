package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_RIGHT;


@Autonomous(name = "Silver", group = "Autonomous")
public class AutoSilver extends OpMode {

    RoverHardware hardware = new RoverHardware();

    public final double RAMP_STOWED = 0.0;
    public final double LOCK_LOCKED = 0.0;

    public void init() {
        hardware.init(hardwareMap);

        hardware.latch.setPosition(LATCH_RIGHT);
        hardware.ramp.setPosition(RAMP_STOWED);
//        hardware.george.setPosition(GEORGE_STOWED);
        hardware.setLockPosition(LOCK_LOCKED);



    }

    public void init_loop() {

    }

    public void start() {

    }

    public void loop() {

    }



}
