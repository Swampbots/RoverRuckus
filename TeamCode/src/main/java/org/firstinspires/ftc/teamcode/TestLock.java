package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@TeleOp(name = "Lock Test", group ="Testing")
public class TestLock extends OpMode {

    RoverHardware hardware = new RoverHardware();
    GamepadCooldowns cooldowns = new GamepadCooldowns();

    double runtime = 0.0;
    final double STEP = 0.05;

    public void init() {
        hardware.init(hardwareMap);

        hardware.setLockPosition(0.5);

        cooldowns.setCooldown(0.500); // 500 milliseconds
    }

    public void loop() {

        runtime = getRuntime();

        if(gamepad1.a)      hardware.setLockPosition(0.0);
        else if(gamepad1.b) hardware.setLockPosition(1.0);





        telemetry.addData("Left lock position", hardware.lockLeft.getPosition());
        telemetry.addData("Right lock position", hardware.lockRight.getPosition());
        telemetry.update();
    }
}
