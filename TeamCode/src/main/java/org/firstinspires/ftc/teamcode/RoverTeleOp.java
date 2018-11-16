package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_INCH_CORE;
import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_REV_CORE;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_MINE;
import static org.firstinspires.ftc.teamcode.RoverHardware.STOP_CLOSED;
import static org.firstinspires.ftc.teamcode.RoverHardware.STOP_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STOWED;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_OMNI;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STD;

@TeleOp(name = "Driver Control", group = "TeleOp")
public class RoverTeleOp extends OpMode {

    RoverHardware hardware = new RoverHardware();

    private final double MINERAL_TOP = 0.0;
    private final double MINERAL_BOTTOM = -650.0;


    private int frontTarget = PIV_STOWED[0];
    private int rearTarget = PIV_STOWED[1];

    private final double PIV_SPEED_BASE = 0.7;
    private final double PIV_SPEED_FRONT = PIV_SPEED_BASE;
    private final double PIV_SPEED_REAR = PIV_SPEED_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);

    private final int GAMEPAD_SENSITIVITY = 20;





    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {

        // Drive motor controls
        hardware.setLeftPower   (-gamepad1.left_stick_y);
        hardware.setRightPower  (-gamepad1.right_stick_y);

        // Snorfler motors controls
        if(gamepad2.left_trigger > gamepad2.right_trigger) hardware.mineral.setPower(-gamepad2.left_trigger);
        else if(gamepad2.left_trigger < gamepad2.right_trigger) hardware.mineral.setPower(gamepad2.right_trigger);
        else hardware.mineral.setPower(0);


        // Pivot motor controls

        // Handle pivot targets
        if(gamepad1.a) {
            frontTarget = PIV_STOWED[0];
            rearTarget = PIV_STOWED[1];
        } else if(gamepad1.b) {
            frontTarget = PIV_KNEEL[0];
            rearTarget = PIV_KNEEL[1];
        } else if(gamepad1.x) {
            frontTarget = PIV_OMNI[0];
            rearTarget = PIV_OMNI[1];
        } else if(gamepad1.y) {
            frontTarget = PIV_STD[0];
            rearTarget = PIV_STD[1];
        } else if(gamepad1.dpad_left) {
            frontTarget = PIV_MINE[0];
            rearTarget = PIV_MINE[1];
        }

        frontTarget += (gamepad2.left_stick_y * GAMEPAD_SENSITIVITY);
        rearTarget += (gamepad2.right_stick_y * GAMEPAD_SENSITIVITY * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT));

        // Set pivot targets
        hardware.frontPivot.setTargetPosition(frontTarget);
        hardware.rearPivot.setTargetPosition(rearTarget);


        // Set target, run mode, and speed
        if(Math.abs(gamepad2.right_stick_y) < 0.05) {
            hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.frontPivot.setPower(PIV_SPEED_FRONT);
        } else {
            hardware.frontPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.frontPivot.setPower(gamepad2.right_stick_y);
            hardware.frontPivot.setTargetPosition(hardware.frontPivot.getCurrentPosition());
        }

        if(Math.abs(gamepad2.left_stick_y) < 0.05) {
            hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.rearPivot.setPower(PIV_SPEED_REAR);
        } else {
            hardware.rearPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.rearPivot.setPower(gamepad2.left_stick_y);
            hardware.rearPivot.setTargetPosition(hardware.rearPivot.getCurrentPosition());
        }




        // Latch servo controls
        hardware.latch.setPosition(gamepad1.right_trigger * LATCH_OPEN);   // This will scale with the latch settings


        // Telemetry
        telemetry.addData("Front Pivot", hardware.frontPivot.getCurrentPosition());
        telemetry.addData("Rear Pivot", hardware.rearPivot.getCurrentPosition());
        telemetry.update();
    }


    // METHODS


    public void setPivots(int[] targets, double speed) {
        hardware.stopAllMotors();

        hardware.frontPivot.setTargetPosition(targets[0]);
        hardware.rearPivot.setTargetPosition(targets[1]);

        hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(hardware.frontPivot.getCurrentPosition() - targets[0] < 0) {
            hardware.frontPivot.setPower(speed);
            hardware.rearPivot.setPower(speed);
        } else {
            hardware.frontPivot.setPower(-speed);
            hardware.rearPivot.setPower(-speed);
        }

        while (hardware.rearPivot.isBusy()) {
            telemetry.addLine("Deploying front to " + hardware.frontPivot.getTargetPosition());
            telemetry.addLine("Deploying rear to " + hardware.rearPivot.getTargetPosition());
            telemetry.addLine();
            telemetry.addData("Front position", hardware.frontPivot.getCurrentPosition());
            telemetry.addData("Rear position", hardware.rearPivot.getCurrentPosition());
            telemetry.update();
        }

        hardware.rearPivot.setPower(0);
        hardware.rearPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (hardware.frontPivot.isBusy()) {
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

}
