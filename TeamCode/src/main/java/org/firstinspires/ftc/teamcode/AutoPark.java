package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_CLOSED;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.LOCK_CLOSED;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.RAMP_DOWN;
import static org.firstinspires.ftc.teamcode.RoverHardware.RAMP_UP;

@Autonomous(name = "Silver", group = "Autonomous")
public class AutoPark extends LinearOpMode {

    public void runOpMode() {
        RoverHardware hardware = new RoverHardware();

        hardware.init(hardwareMap);


        waitForStart();

        // Descend from lander
        hardware.setLockPosition(RoverHardware.LOCK_OPEN);
        sleep(3000);
        hardware.setLockPosition(RoverHardware.LOCK_CLOSED);
        sleep(2500);

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
        hardware.driveInches(48.0, 0.8);


        telemetry.addLine("Finished.");
        telemetry.update();
    }
}
