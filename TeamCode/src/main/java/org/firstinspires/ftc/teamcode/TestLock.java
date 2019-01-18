package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@TeleOp(name = "Lock Test", group ="Testing")
public class TestLock extends OpMode {

    RoverHardware hardware = new RoverHardware();


    public void init() {
        hardware.init(hardwareMap);

        hardware.setLockPosition(0.5);
    }

    public void loop() {

        if(gamepad1.a)      hardware.setLockPosition(0.0);
        else if(gamepad1.b) hardware.setLockPosition(1.0);





        telemetry.addData("Left lock position", hardware.lockLeft.getPosition());
        telemetry.addData("Right lock position", hardware.lockRight.getPosition());
        telemetry.update();
    }
}
