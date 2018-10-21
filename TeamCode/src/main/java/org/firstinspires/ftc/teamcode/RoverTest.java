package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Rover Test", group = "Testing")
public class RoverTest extends OpMode {

    RoverHardware hardware = new RoverHardware();

    private final double MINERAL_TOP = 0;
    private final double MINERAL_BOTTOM = -650.0;

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

        if(hardware.mineral.getCurrentPosition() < MINERAL_TOP && gamepad2.a) {
            hardware.mineral.setPower(0.2);
        } else if(hardware.mineral.getCurrentPosition() > MINERAL_BOTTOM && gamepad2.b) {
            hardware.mineral.setPower(-0.2);
        } else hardware.mineral.setPower(0);

        if(gamepad2.a) setPivots(0, 0.6);
        


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

    public void setPivots(int targetCounts, double speed) {
        hardware.stopAllMotors();

        hardware.frontPivot.setTargetPosition(targetCounts);
        hardware.rearPivot.setTargetPosition(targetCounts);

        hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(hardware.frontPivot.getCurrentPosition() - targetCounts < 0) {
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
