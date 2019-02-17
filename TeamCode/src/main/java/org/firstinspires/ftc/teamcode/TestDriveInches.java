package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test Drive Inches", group = "Testing")
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

        telemetry.addLine("Encoders:");
        telemetry.addData("Front left", hardware.frontLeft.getCurrentPosition());
        telemetry.addData("Front right", hardware.frontLeft.getCurrentPosition());
        telemetry.addData("Rear left", hardware.frontLeft.getCurrentPosition());
        telemetry.addData("Rear right.", hardware.frontLeft.getCurrentPosition());
        telemetry.update();
    }
}
