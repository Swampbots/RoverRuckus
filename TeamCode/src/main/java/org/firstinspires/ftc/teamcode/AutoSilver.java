package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Silver", group = "Autonomous")
public class AutoSilver extends LinearOpMode {

    RoverHardware hardware = new RoverHardware();

    public final int WINCH_COUNTS = 7000;
    public final double WINCH_SPEED = -0.5;

    public final double DEPLOY_SPEED = 0.5;

    public final double LATCH_OPEN = 0.6;

    public final int DRIVE_COUNTS = 5000;
    public final double DRIVE_SPEED = 1.0;


    public void runOpMode() {

        hardware.init(hardwareMap);

        telemetry.addLine("Hardware initialized");
        telemetry.addLine("Press play to start");
        telemetry.update();


        waitForStart();
        /*
        STEP 1: release stop
        STEP 2: raise wheels
        STEP 3: open latch
        STEP 4: drive forward to crater
         */


        // STEP 1
        telemetry.addLine("Step 1");
        telemetry.update();
        sleep(1000);

        hardware.stop.setPosition(0.7);

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

        sleep(2000);


//        // STEP 4
//        telemetry.addLine("Step 4");
//        telemetry.update();
//        sleep(1000);
//
//        driveToPosition(DRIVE_COUNTS, DRIVE_SPEED);


        // FINISHED

        while(opModeIsActive()) {
            telemetry.addData("Latch position", hardware.latch.getPosition());
            telemetry.addData("Winch counts", WINCH_COUNTS);
            telemetry.addData("Drive counts", DRIVE_COUNTS);
            telemetry.addLine();
            telemetry.addData("Winch position", hardware.winch.getCurrentPosition());
            telemetry.addData("fl_drive position", hardware.frontLeft.getCurrentPosition());
            telemetry.update();
        }
    }


    public void runToPosition(DcMotor motor, int target, double speed) {
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);

        while(opModeIsActive() && motor.isBusy()) {
            telemetry.addLine("Deploying to " + motor.getTargetPosition());
            telemetry.addData("Winch position", motor.getCurrentPosition());
            telemetry.update();
        }

        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void deployWheels(double speed) {
        hardware.frontPivot.setTargetPosition(-2400);
        hardware.rearPivot.setTargetPosition(-1050);

        hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.frontPivot.setPower(speed);
        hardware.rearPivot.setPower(speed);


        while (opModeIsActive() && hardware.rearPivot.isBusy()) {
            telemetry.addLine("Deploying front to " + hardware.frontPivot.getTargetPosition());
            telemetry.addLine("Deploying rear to " + hardware.rearPivot.getTargetPosition());
            telemetry.addLine();
            telemetry.addData("Front position", hardware.frontPivot.getCurrentPosition());
            telemetry.addData("Rear position", hardware.rearPivot.getCurrentPosition());
            telemetry.update();
        }

        hardware.rearPivot.setPower(0);
        hardware.rearPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && hardware.frontPivot.isBusy()) {
            telemetry.addLine("Deploying front to " + hardware.frontPivot.getTargetPosition());
            telemetry.addLine("Deploying rear to " + hardware.rearPivot.getTargetPosition());
            telemetry.addLine();
            telemetry.addData("Front position", hardware.frontPivot.getCurrentPosition());
            telemetry.addData("Rear position", hardware.rearPivot.getCurrentPosition());
            telemetry.update();
        }

        hardware.frontPivot.setPower(0);
        hardware.frontPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void driveToPosition(int target, double speed) {
        hardware.frontLeft.setTargetPosition    (hardware.frontLeft.getCurrentPosition()    + target);
        hardware.rearLeft.setTargetPosition     (hardware.rearLeft.getCurrentPosition()     + target);
        hardware.frontRight.setTargetPosition   (hardware.frontRight.getCurrentPosition()   - target);
        hardware.rearRight.setTargetPosition    (hardware.rearRight.getCurrentPosition()    - target);

        hardware.frontLeft.setMode  (DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearLeft.setMode   (DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontRight.setMode (DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearRight.setMode  (DcMotor.RunMode.RUN_TO_POSITION);

        hardware.frontLeft.setPower (speed);
        hardware.rearLeft.setPower  (speed);
        hardware.frontRight.setPower(speed);
        hardware.rearRight.setPower (speed);

        while(opModeIsActive() &&
                hardware.frontLeft  .isBusy() &&
                hardware.rearLeft   .isBusy() &&
                hardware.frontRight .isBusy() &&
                hardware.rearRight  .isBusy());

        hardware.frontLeft  .setPower(0);
        hardware.rearLeft   .setPower(0);
        hardware.frontRight .setPower(0);
        hardware.rearRight  .setPower(0);

        hardware.frontLeft.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rearLeft.setMode   (DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.frontRight.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rearRight.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
