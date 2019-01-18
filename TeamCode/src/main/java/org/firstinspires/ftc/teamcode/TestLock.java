package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STOWED;

//@Disabled
@TeleOp(name = "Lock Test", group ="Testing")
public class TestLock extends OpMode {

    RoverHardware hardware = new RoverHardware();



    private int frontTarget = PIV_STOWED[0];
    private int rearTarget = PIV_STOWED[1];

    private final double PIV_SPEED_BASE = 0.7;
    private final double PIV_SPEED_FRONT = PIV_SPEED_BASE;
    private final double PIV_SPEED_REAR = PIV_SPEED_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);

    private final double PIV_SPEED_SCALER_FRONT = 0.6;
    private final double PIV_SPEED_SCALER_REAR = PIV_SPEED_SCALER_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);


    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {

        if(gamepad1.a)      hardware.setLockPosition(0.0);
        else if(gamepad1.b) hardware.setLockPosition(1.0);

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



        telemetry.addData("Left lock position", hardware.lockLeft.getPosition());
        telemetry.addData("Right lock position", hardware.lockRight.getPosition());
        telemetry.addLine();
        telemetry.addData("Front pivot", hardware.frontPivot.getCurrentPosition());
        telemetry.addData("Rear pivot", hardware.rearPivot.getCurrentPosition());
        telemetry.update();
    }
}
