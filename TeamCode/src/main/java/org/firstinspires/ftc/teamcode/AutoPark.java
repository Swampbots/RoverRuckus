package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Silver", group = "Autonomous")
public class AutoPark extends LinearOpMode {

    public void runOpMode() {
        RoverHardware hardware = new RoverHardware();

        hardware.init(hardwareMap);


        waitForStart();

        telemetry.addLine("Started!");
        telemetry.update();
    }
}
