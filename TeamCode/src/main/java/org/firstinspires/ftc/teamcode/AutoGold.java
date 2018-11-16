package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_LEFT;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_RIGHT;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_MINE_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_MINE_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_OMNI_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STD_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STD_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.STOP_CLOSED;

@Autonomous(name = "Gold", group = "Autonomous")
public class AutoGold extends LinearOpMode {

    RoverHardware hardware = new RoverHardware();

    public final double DEPLOY_SPEED = 0.5;

    public final int DEPLOY_COUNTS = 250;
    public final int DRIVE_COUNTS = 1000;
    public final double DRIVE_SPEED = 1.0;


    public void runOpMode() {

        hardware.init(hardwareMap);

        hardware.latch.setPosition(LATCH_LEFT);
        hardware.stop.setPosition(STOP_CLOSED);

        telemetry.addLine("Hardware initialized");
        telemetry.addLine("Press play to start");
        telemetry.update();





        waitForStart();
        /*
        STEP 1: release stop
        STEP 2: deploy wheels
        STEP 3: open latch
        STEP 4: drive forward to depot
        STEP 5: deposit marker
         */

        // STEP 1
        telemetry.addLine("Step 1");
        telemetry.update();
        sleep(1000);

        hardware.stop.setPosition(hardware.STOP_OPEN);

        // STEP 2
        telemetry.addLine("Step 2");
        telemetry.update();
        sleep(1000);

        deployWheels(DEPLOY_SPEED);


        // STEP 3
        telemetry.addLine("Step 3");
        telemetry.update();
        sleep(1000);

        hardware.latch.setPosition(LATCH_OPEN);
        sleep(500);
        hardware.latch.setPosition(LATCH_LEFT);
        sleep(500);
        hardware.latch.setPosition(LATCH_OPEN);
        sleep(500);
        hardware.latch.setPosition(LATCH_LEFT);
        sleep(500);
        hardware.latch.setPosition(LATCH_OPEN);

        sleep(2000);


        // STEP 4
        telemetry.addLine("Step 4");
        telemetry.update();
        sleep(1000);

        driveToPosition(DEPLOY_COUNTS, DRIVE_SPEED);

    }
}
