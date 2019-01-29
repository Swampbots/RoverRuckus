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

@Autonomous(name = "Gold", group = "Autonomous")
public class AutoClaim extends LinearOpMode {


    public void runOpMode() {
        RoverHardware hardware = new RoverHardware();

        hardware.init(hardwareMap);




        waitForStart();

        // Descend from lander
        hardware.setLockPosition(RoverHardware.LOCK_OPEN);
        sleep(4500);
        hardware.setLockPosition(RoverHardware.LOCK_CLOSED);

        // Deploy wheels
        hardware.frontPivot.setTargetPosition(PIV_KNEEL_FRONT);
        hardware.rearPivot.setTargetPosition(PIV_KNEEL_REAR);

        hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.frontPivot.setPower(1.0);
        hardware.rearPivot.setPower(1.0);

        sleep(1000);

        // Release latch
        hardware.latch.setPosition(LATCH_OPEN);

        // Lower ramp
        hardware.ramp.setPosition(RAMP_DOWN);

        // Drive through center sample to crater
        hardware.driveInches(48.0, 0.8);

        // Snorfle out
        hardware.snorfler.setPower(1.0);


        telemetry.addLine("Finished.");
        telemetry.update();
    }
}
