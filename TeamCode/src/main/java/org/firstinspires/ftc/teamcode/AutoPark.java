package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoPark extends LinearOpMode {

    public void runOpMode() {
        RoverHardware hardware = new RoverHardware();

        hardware.init(hardwareMap);


        waitForStart();

        telemetry.addLine("Started!");
        telemetry.update();
    }
}
