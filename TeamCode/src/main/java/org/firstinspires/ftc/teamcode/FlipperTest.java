package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_REV_HD;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_MINE;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_OMNI;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STD;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STOWED;

//@Disabled
@TeleOp(name = "Flipper Test", group = "Testing")
public class FlipperTest extends OpMode {

    RoverHardware hardware = new RoverHardware();

    private final double FLIPPER_SPEED_BASE = 0.05;
    private final double FLIPPER_SPEED_SCALER = 0.6;

    int frontTarget = PIV_STOWED[0];
    int rearTarget = PIV_STOWED[1];


    private final int COUNTS_PER_REV_HD_20 = COUNTS_PER_REV_HD / 2; // Converting from a 40:1 motor to a 20:1
    private final double GEAR_REDUCTION_FLIPPER = 1.0; // 1:1 gear ratio
    private final int COUNTS_PER_DEGREE_REV_HD_20 = (int)((COUNTS_PER_REV_HD_20 * GEAR_REDUCTION_FLIPPER) / 480);



    private final double PIV_SPEED_BASE = 0.7;
    private final double PIV_SPEED_FRONT = PIV_SPEED_BASE;
    private final double PIV_SPEED_REAR = PIV_SPEED_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);

    private final int GAMEPAD_SENSITIVITY = 20;


    private final double PIV_SPEED_SCALER_FRONT = 0.6;
    private final double PIV_SPEED_SCALER_REAR = PIV_SPEED_SCALER_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);


    public int flipperTarget = 0;

    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {
//        if(gamepad1.a) flipperTarget = 0;
//        if(gamepad1.y) flipperTarget = COUNTS_PER_DEGREE_REV_HD_20 * 180;
//
//        hardware.flipper.setTargetPosition(flipperTarget);
//
//        // Handle flipper target, run mode, and speed
//        if(Math.abs(gamepad1.right_stick_y) < 0.05) {
//            hardware.flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hardware.flipper.setPower(FLIPPER_SPEED_BASE);
//        } else {
//            hardware.flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            hardware.flipper.setPower(gamepad1.left_stick_y * FLIPPER_SPEED_BASE);
//            flipperTarget = hardware.flipper.getCurrentPosition();
//        }

//        double flipperScalar = 0.6;
//
//        if(gamepad2.a) flipperScalar = 1.0;
//        else if(gamepad2.b) flipperScalar = 0.4;
//        else if(gamepad2.x) flipperScalar = 0.6;
//        else if(gamepad2.y) flipperScalar = 0.8;
//
//        hardware.snorfler.setPower(gamepad2.left_stick_y * flipperScalar);
//        hardware.flipper.setPower(gamepad2.right_stick_y);
//
//
//        telemetry.addData("Launcher Position", hardware.snorfler.getCurrentPosition());
//        telemetry.addData("Flipper Scalar", flipperScalar);
//        telemetry.update();




        // Handle pivot targets
        if(gamepad2.a) {
            frontTarget = PIV_KNEEL[0];
            rearTarget = PIV_KNEEL[1];
        } else if(gamepad2.b) {
            frontTarget = PIV_STOWED[0];
            rearTarget = PIV_STOWED[1];
        } else if(gamepad2.x) {
            frontTarget = PIV_OMNI[0];
            rearTarget = PIV_OMNI[1];
        } else if(gamepad2.y) {
            frontTarget = PIV_STD[0];
            rearTarget = PIV_STD[1];
        } else if(gamepad2.dpad_left) {
            frontTarget = PIV_MINE[0];
            rearTarget = PIV_MINE[1];
        }

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
    }
}
