package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Rover Test", group = "Testing")
public class RoverTest extends OpMode {




    RoverHardware hardware = new RoverHardware();

    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {

        // Motor controls
        hardware.setLeftPower   (gamepad1.left_stick_y);
        hardware.setRightPower  (gamepad1.right_stick_y);

        hardware.rearPivot.setPower (gamepad2.left_stick_y);
        hardware.frontPivot.setPower(gamepad2.right_stick_y);

        if(gamepad1.dpad_up)        hardware.winch.setPower(1.0);
        else if(gamepad1.dpad_down) hardware.winch.setPower(-1.0);
        else                        hardware.winch.setPower(0);



//        // START PIVOT CODE
//
//
//        boolean aCooldownOver = ((getRuntime() - runtimeSnapshot) >= GP1_A_COOLDOWN);
//
//        if(gamepad1.a && aCooldownOver) {
//
//            runtimeSnapshot = getRuntime();
//
//            hardware.rearPivot.setTargetPosition    ((int)hardware.STANDARD);
//            hardware.frontPivot.setTargetPosition   ((int)hardware.STANDARD);
//
//            // Both motors move the same direction
//            frontRotForward = true;
//            rearRotForward = true;
//        }
//
//        boolean frontNotReached = ( hardware.frontPivot.getTargetPosition() -
//                hardware.frontPivot.getCurrentPosition() >
//                PIVOT_ERROR);
//
//        boolean rearNotReached = ( hardware.frontPivot.getTargetPosition() -
//                hardware.frontPivot.getCurrentPosition() >
//                PIVOT_ERROR);
//
//        // If the pivot is not busy and the target is not reached (to exclude case where target is reached)
//        if(!hardware.frontPivot.isBusy() && frontNotReached) {
//            hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hardware.frontPivot.setPower(1.0);
//        } else if(hardware.frontPivot.isBusy() && frontNotReached) { // (still finding target)
//
//        } else { // (target reached and pivot is not busy)
//            hardware.frontPivot.setPower(0.0);
//            hardware.frontPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        }
//
//        if(!hardware.rearPivot.isBusy()) {
//            hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hardware.rearPivot.setPower(1.0);
//        } else {
//            hardware.rearPivot.setPower(0.0);
//            hardware.rearPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        }
//
//        // END PIVOT CODE



        // Motor encoder controls
        if(gamepad2.a)  hardware.resetDriveEncoders();
        if(gamepad2.b)  hardware.resetPivotEncoders();


        // Servo controls
        hardware.latch.setPosition(gamepad1.right_trigger);


        // Telemetry
        telemetry.addData("Front Pivot power", hardware.frontPivot.getPower());
        telemetry.addData("Rear Pivot power", hardware.rearPivot.getPower());
        telemetry.addLine();
        telemetry.addData("Front pivot encoder",    hardware.frontPivot.getCurrentPosition());
        telemetry.addData("Rear pivot encoder",     hardware.rearPivot.getCurrentPosition());
        telemetry.addLine();
        telemetry.update();
    }
}
