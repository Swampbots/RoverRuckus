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


        hardware.latch.setPosition(gamepad1.right_trigger);




        telemetry.addData("Latch Position", hardware.latch.getPosition());
        telemetry.addLine();
        telemetry.update();
    }
}
