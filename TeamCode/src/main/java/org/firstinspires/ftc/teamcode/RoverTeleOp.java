package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RoverHardware.FAST;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.NORMAL;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STOWED;
import static org.firstinspires.ftc.teamcode.RoverHardware.SLOW;

@TeleOp(name = "Driver Control", group = "TeleOp")
public class RoverTeleOp extends OpMode {

    RoverHardware hardware = new RoverHardware();

    public double flipperSpeedModifier;



    private int frontTarget = PIV_STOWED[0];
    private int rearTarget = PIV_STOWED[1];

    private int flipperTarget = 0;



    private final double PIV_SPEED_BASE = 1.0;
    private final double PIV_SPEED_FRONT = PIV_SPEED_BASE;
    private final double PIV_SPEED_REAR = PIV_SPEED_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);

    private final double PIV_SPEED_SCALER_FRONT = 1.0;
    private final double PIV_SPEED_SCALER_REAR = PIV_SPEED_SCALER_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);

    ButtonCooldown gp2_b = new ButtonCooldown();
    ButtonCooldown gp2_x = new ButtonCooldown();



    public void init() {
        flipperSpeedModifier = NORMAL;
        hardware.init(hardwareMap);

        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardware.flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gp2_b.setCooldown(0.175);
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
        if(gamepad2.dpad_up) {
            hardware.flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.flipper.setPower(flipperSpeedModifier);
            flipperTarget = hardware.flipper.getCurrentPosition();
        } else if(gamepad2.dpad_down) {
            hardware.flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.flipper.setPower(-flipperSpeedModifier);
            flipperTarget = hardware.flipper.getCurrentPosition();
        } else {
            hardware.flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.flipper.setPower(flipperSpeedModifier);
        }
//        if(gamepad2.dpad_up)        hardware.flipper.setPower(flipperSpeedModifier);
//        else if(gamepad2.dpad_down) hardware.flipper.setPower(-flipperSpeedModifier);
//        else hardware.flipper.setPower(0.0);

        // Snorfler controls
        if(gamepad2.left_trigger > gamepad2.right_trigger) hardware.snorfler.setPower(-gamepad2.left_trigger);
        else if(gamepad2.left_trigger < gamepad2.right_trigger) hardware.snorfler.setPower(gamepad2.right_trigger);
        else hardware.snorfler.setPower(0);



        // Latch servo controls
        hardware.latch.setPosition(gamepad1.right_trigger * LATCH_OPEN);   // This will scale with the latch settings


        double runtime = getRuntime();
        // Ramp servo
        if(gamepad2.b && gp2_b.ready(runtime)) {
            hardware.ramp.setPosition(Math.abs(hardware.ramp.getPosition() - 1.0));
            gp2_b.updateSnapshot(runtime);
        }

        // Bucket servo
        if(gamepad2.x && gp2_x.ready(runtime)) {
            hardware.bucket.setPosition(Math.abs(hardware.bucket.getPosition() - 1.0));
            gp2_x.updateSnapshot(runtime);
        }

        // Telemetry
        telemetry.addData("Front Pivot", hardware.frontPivot.getCurrentPosition());
        telemetry.addData("Rear Pivot", hardware.rearPivot.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Flipper Position", hardware.flipper.getCurrentPosition());
        telemetry.addData("Flipper speed mod", flipperSpeedModifier);
        telemetry.update();
    }

}
