package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.RoverHardware.GEORGE_DEPLOY;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEORGE_STOW;

@Disabled
@Autonomous(name = "George Test", group = "Testing")
public class TestGeorge extends LinearOpMode {

    RoverHardware hardware = new RoverHardware();

    public void runOpMode() {
        hardware.init(hardwareMap);


        telemetry.addLine("Press play to start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.dpad_up) hardware.george.setPosition(GEORGE_DEPLOY);
            else if(gamepad1.dpad_down) hardware.george.setPosition(GEORGE_STOW);

            telemetry.addData("George", hardware.george.getPosition());
            telemetry.addLine();
            telemetry.addData("Deployed", GEORGE_DEPLOY);
            telemetry.addData("Stow", GEORGE_STOW);
            telemetry.update();
        }
    }
}
