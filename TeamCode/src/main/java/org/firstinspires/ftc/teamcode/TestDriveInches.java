package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TestDriveInches extends OpMode {

    RoverHardware hardware = new RoverHardware();


    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {
        if(gamepad1.a)  hardware.driveInches(2.0, 0.8);
        if(gamepad1.b)  hardware.driveInches(4.0, 0.8);
        if(gamepad1.x)  hardware.driveInches(-2.0, 0.8);
        if(gamepad1.y)  hardware.driveInches(-4.0, 0.8);
    }
}
