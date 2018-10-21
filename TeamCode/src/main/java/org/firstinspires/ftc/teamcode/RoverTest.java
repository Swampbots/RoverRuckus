package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Rover Test", group = "Testing")
public class RoverTest extends OpMode {

    RoverHardware hardware = new RoverHardware();

    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {

        // Motor controls
        hardware.setLeftPower   (gamepad1.left_stick_y);
        hardware.setRightPower  (gamepad1.right_stick_y);

        hardware.rearPivot.setPower (gamepad2.left_stick_y);
        hardware.frontPivot.setPower(gamepad2.right_stick_y);

        if(gamepad1.dpad_up)        hardware.winch.setPower(1.0);
        else if(gamepad1.dpad_down) hardware.winch.setPower(-1.0);
        else                        hardware.winch.setPower(0);

        



        // Motor encoder controls
        if(gamepad2.a)  hardware.resetDriveEncoders();
        if(gamepad2.b)  hardware.resetPivotEncoders();


        // Servo controls
        hardware.latch.setPosition(gamepad1.right_trigger);


        // Telemetry
        telemetry.addData("Front Pivot power", hardware.frontPivot.getPower());
        telemetry.addData("Rear Pivot power", hardware.rearPivot.getPower());
        telemetry.addLine();
        telemetry.addData("Front pivot encoder",    hardware.frontPivot.getCurrentPosition());
        telemetry.addData("Rear pivot encoder",     hardware.rearPivot.getCurrentPosition());
        telemetry.addLine();
        telemetry.update();
    }
}
