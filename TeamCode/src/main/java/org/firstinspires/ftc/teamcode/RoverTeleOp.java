package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STOWED;

@TeleOp(name = "Driver Control", group = "TeleOp")
public class RoverTeleOp extends OpMode {

    RoverHardware hardware = new RoverHardware();


    public final double SLOW = 0.4;
    public final double NORMAL = 0.6;
    public final double FAST = 0.9;

    public double flipperSpeedModifier;





    private int frontTarget = PIV_STOWED[0];
    private int rearTarget = PIV_STOWED[1];

    private final double PIV_SPEED_BASE = 0.7;
    private final double PIV_SPEED_FRONT = PIV_SPEED_BASE;
    private final double PIV_SPEED_REAR = PIV_SPEED_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);

    private final int GAMEPAD_SENSITIVITY = 20;


    private final double PIV_SPEED_SCALER_FRONT = 0.6;
    private final double PIV_SPEED_SCALER_REAR = PIV_SPEED_SCALER_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);


    public void init() {
        flipperSpeedModifier = NORMAL;
        hardware.init(hardwareMap);
    }

    public void loop() {


        // Speed modifier
        if(gamepad2.left_bumper) {
            flipperSpeedModifier = FAST;
        } else if(gamepad2.right_bumper) {
            flipperSpeedModifier = SLOW;
        } else {
            flipperSpeedModifier = NORMAL;
        }




        // Drive motor controls
        hardware.setLeftPower   (-gamepad1.left_stick_y);
        hardware.setRightPower  (-gamepad1.right_stick_y);




        // Set pivot target, run mode, and speed
        if(Math.abs(gamepad2.right_stick_y) < 0.05) {
            hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontPivot.setPower(PIV_SPEED_FRONT);
        } else {
            hardware.frontPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.frontPivot.setPower(-gamepad2.right_stick_y * PIV_SPEED_SCALER_FRONT);
            frontTarget = hardware.frontPivot.getCurrentPosition();
        }

        if(Math.abs(gamepad2.left_stick_y) < 0.05) {
            hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.rearPivot.setPower(PIV_SPEED_REAR);
        } else {
            hardware.rearPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.rearPivot.setPower(-gamepad2.left_stick_y * PIV_SPEED_SCALER_REAR);
            rearTarget = hardware.rearPivot.getCurrentPosition();
        }

        // Set pivot targets
        hardware.frontPivot.setTargetPosition(frontTarget);
        hardware.rearPivot.setTargetPosition(rearTarget);


        // Flipper controls
        if(gamepad2.dpad_up)        hardware.flipper.setPower(flipperSpeedModifier);
        else if(gamepad2.dpad_down) hardware.flipper.setPower(-flipperSpeedModifier);
        else hardware.flipper.setPower(0.0);

        // Snorfler controls
        if(gamepad2.left_trigger > gamepad2.right_trigger) hardware.snorfler.setPower(-gamepad2.left_trigger);
        else if(gamepad2.left_trigger < gamepad2.right_trigger) hardware.snorfler.setPower(gamepad2.right_trigger);
        else hardware.snorfler.setPower(0);



        // Latch servo controls
        hardware.latch.setPosition(gamepad1.right_trigger * LATCH_OPEN);   // This will scale with the latch settings

        // Ramp servo
        hardware.ramp.setPosition(gamepad2.a ? 0.0 : 1.0);

        // Lifter servo
        hardware.lifter.setPosition(gamepad2.b ? 0.0 : 1.0);




        // Telemetry
        telemetry.addData("Front Pivot", hardware.frontPivot.getCurrentPosition());
        telemetry.addData("Rear Pivot", hardware.rearPivot.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Flipper Position", hardware.snorfler.getCurrentPosition());
        telemetry.addData("Flipper speed mod", flipperSpeedModifier);
        telemetry.addLine();
//        telemetry.addData("Flipper Scalar", flipperScalar);
        telemetry.update();
    }






//    // METHODS
//
//
//    public void setPivots(int[] targets, double speed) {
//        hardware.stopAllMotors();
//
//        hardware.frontPivot.setTargetPosition(targets[0]);
//        hardware.rearPivot.setTargetPosition(targets[1]);
//
//        hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if(hardware.frontPivot.getCurrentPosition() - targets[0] < 0) {
//            hardware.frontPivot.setPower(speed);
//            hardware.rearPivot.setPower(speed);
//        } else {
//            hardware.frontPivot.setPower(-speed);
//            hardware.rearPivot.setPower(-speed);
//        }
//
//        while (hardware.rearPivot.isBusy()) {
//            telemetry.addLine("Deploying front to " + hardware.frontPivot.getTargetPosition());
//            telemetry.addLine("Deploying rear to " + hardware.rearPivot.getTargetPosition());
//            telemetry.addLine();
//            telemetry.addData("Front position", hardware.frontPivot.getCurrentPosition());
//            telemetry.addData("Rear position", hardware.rearPivot.getCurrentPosition());
//            telemetry.update();
//        }
//
//        hardware.rearPivot.setPower(0);
//        hardware.rearPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        while (hardware.frontPivot.isBusy()) {
//            telemetry.addLine("Deploying front to " + hardware.frontPivot.getTargetPosition());
//            telemetry.addLine("Deploying rear to " + hardware.rearPivot.getTargetPosition());
//            telemetry.addLine();
//            telemetry.addData("Front position", hardware.frontPivot.getCurrentPosition());
//            telemetry.addData("Rear position", hardware.rearPivot.getCurrentPosition());
//            telemetry.update();
//        }
//
//        hardware.frontPivot.setPower(0);
//        hardware.frontPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }

}
