package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_INCH_DRIVE_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_INCH_DRIVE_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_DRIVE_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_DRIVE_REAR;

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
        telemetry.addData("Front right", hardware.frontRight.getCurrentPosition());
        telemetry.addData("Rear left", hardware.rearLeft.getCurrentPosition());
        telemetry.addData("Rear right", hardware.rearRight.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("CPI-Front", COUNTS_PER_INCH_DRIVE_FRONT);
        telemetry.addData("CPI-Rear", COUNTS_PER_INCH_DRIVE_REAR);
        telemetry.addLine();
        telemetry.addData("GRD-Front", GEAR_REDUCTION_DRIVE_FRONT);
        telemetry.addData("GRD-Rear", GEAR_REDUCTION_DRIVE_REAR);
        telemetry.addLine();

        telemetry.update();
    }
}
