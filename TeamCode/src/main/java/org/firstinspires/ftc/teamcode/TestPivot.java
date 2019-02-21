package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_MINE;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STOWED;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STD;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_OMNI;

@TeleOp(name = "Pivot Test", group = "Testing")
public class TestPivot extends LinearOpMode {

    RoverHardware hardware = new RoverHardware();

    private int frontTarget = PIV_STOWED[0];
    private int rearTarget = PIV_STOWED[1];

    private final double PIV_SPEED_FRONT = 1.0;
    private final double PIV_SPEED_REAR = PIV_SPEED_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);

    private final double PIV_SPEED_SCALER_FRONT = 1.0;
    private final double PIV_SPEED_SCALER_REAR = PIV_SPEED_SCALER_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);


    public void runOpMode() {
        hardware.init(hardwareMap);

        hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(opModeIsActive()) {
            // Set drive motor powers
            hardware.setLeftPower   (-gamepad1.left_stick_y);
            hardware.setRightPower  (-gamepad1.right_stick_y);


            // Handle pivot states
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

            if(Math.abs(gamepad2.right_stick_y) < 0.05) {
                hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.frontPivot.setPower(PIV_SPEED_FRONT);
            } else {
                hardware.frontPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hardware.frontPivot.setPower(-gamepad2.right_stick_y * PIV_SPEED_SCALER_FRONT);
                frontTarget = hardware.frontPivot.getCurrentPosition();
            }

            if(Math.abs(gamepad2.left_stick_y) < 0.05) {
                hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.rearPivot.setPower(PIV_SPEED_REAR);
            } else {
                hardware.rearPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hardware.rearPivot.setPower(-gamepad2.left_stick_y * PIV_SPEED_SCALER_REAR);
                rearTarget = hardware.rearPivot.getCurrentPosition();
            }

            // Set pivot targets
            hardware.frontPivot.setTargetPosition(frontTarget);
            hardware.rearPivot.setTargetPosition(rearTarget);


            // TELEMETRY
            telemetry.addData("Front Target",   frontTarget);
            telemetry.addData("Rear Target",    rearTarget);
            telemetry.addLine();
            telemetry.update();
        }
    }

}
