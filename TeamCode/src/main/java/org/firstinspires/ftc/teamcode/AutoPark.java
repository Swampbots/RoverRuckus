package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.RAMP_DOWN;

@Autonomous(name = "Silver", group = "Autonomous")
public class AutoPark extends LinearOpMode {

    public void runOpMode() {
        RoverHardware hardware = new RoverHardware();

        hardware.init(hardwareMap);


        waitForStart();

        // Descend from lander
        hardware.setLockPosition(RoverHardware.LOCK_OPEN);
        sleep(1500);
        hardware.setLockPosition(RoverHardware.LOCK_CLOSED);

        // Deploy wheels
        hardware.frontPivot.setTargetPosition(PIV_KNEEL_FRONT);
        hardware.frontPivot.setTargetPosition(PIV_KNEEL_REAR);

        // Release latch
        hardware.latch.setPosition(LATCH_OPEN);

        // Lower ramp
        hardware.ramp.setPosition(RAMP_DOWN);

        // Snorfle in
        hardware.snorfler.setPower(-1.0);

        // Drive through center sample to crater
        hardware.driveInches(24.0, 0.8);


        telemetry.addLine("Finished.");
        telemetry.update();
    }
}
