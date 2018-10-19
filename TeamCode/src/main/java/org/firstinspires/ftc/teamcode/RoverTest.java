package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Rover Test", group = "Testing")
public class RoverTest extends OpMode {

    RoverHardware hardware = new RoverHardware();

    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {
        hardware.setLeftPower   (gamepad1.left_stick_y);
        hardware.setRightPower  (gamepad1.right_stick_y);

        hardware.rearPivot.setPower (gamepad2.left_stick_y);
        hardware.frontPivot.setPower(gamepad2.right_stick_y);

        hardware.latch.setPosition(gamepad1.right_trigger);

        if(gamepad1.a)  hardware.resetDriveEncoders();
        if(gamepad1.b)  hardware.resetPivotEncoders();


        telemetry.addData("Rear left encoder",      hardware.rearLeft.getCurrentPosition());
        telemetry.addData("Rear right encoder",     hardware.rearRight.getCurrentPosition());
        telemetry.addData("Front left encoder",     hardware.frontLeft.getCurrentPosition());
        telemetry.addData("Front right encoder",    hardware.frontRight.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Front pivot encoder",    hardware.frontPivot.getCurrentPosition());
        telemetry.addData("Rear pivot encoder",     hardware.rearPivot.getCurrentPosition());
        telemetry.addLine();
        telemetry.update();
    }
}
