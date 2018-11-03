package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_INCH_CORE;
import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_REV_CORE;

@TeleOp(name = "Rover Test", group = "Testing")
public class RoverTest extends OpMode {

    RoverHardware hardware = new RoverHardware();

    private final double MINERAL_TOP = 0.0;
    private final double MINERAL_BOTTOM = -650.0;

    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {

        // Motor controls
        hardware.setLeftPower   (-gamepad1.left_stick_y);
        hardware.setRightPower  (-gamepad1.right_stick_y);

        hardware.rearPivot.setPower (gamepad2.left_stick_y);
        hardware.frontPivot.setPower(gamepad2.right_stick_y);

        if(gamepad1.dpad_up)        hardware.winch.setPower(1.0);
        else if(gamepad1.dpad_down) hardware.winch.setPower(-1.0);
        else                        hardware.winch.setPower(0);

        if(gamepad2.left_trigger > gamepad2.right_trigger) hardware.mineral.setPower(-gamepad2.left_trigger);
        else if(gamepad2.left_trigger < gamepad2.right_trigger) hardware.mineral.setPower(gamepad2.right_trigger);
        else hardware.mineral.setPower(0);

//        if(gamepad2.a) setPivots(PIV_STOWED,    0.6);
//        if(gamepad2.b) setPivots(PIV_OMNI,      0.6);
//        if(gamepad2.x) setPivots(PIV_KNEEL,     0.6);
//        if(gamepad2.y) setPivots(PIV_STD,       0.6);

        if(gamepad1.dpad_left) hardware.driveCounts(COUNTS_PER_INCH_CORE * 6, 0.6); // 6 inches at 60% power



        // Servo controls
        hardware.latch.setPosition(gamepad1.right_trigger);
        if(gamepad2.a) hardware.stop.setPosition(1.0);
        else hardware.stop.setPosition(0.7);


        // Telemetry
        telemetry.addData("fl power", hardware.frontLeft.getPower());
        telemetry.addData("rl power", hardware.rearLeft.getPower());
        telemetry.addData("fr power", hardware.frontRight.getPower());
        telemetry.addData("rr power", hardware.rearRight.getPower());
        telemetry.addLine();
        telemetry.addData("fl encoder", hardware.frontLeft.getCurrentPosition());
        telemetry.addData("rl encoder", hardware.rearLeft.getCurrentPosition());
        telemetry.addData("fr encoder", hardware.frontRight.getCurrentPosition());
        telemetry.addData("rr encoder", hardware.rearRight.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Counts per rev (Core)", COUNTS_PER_REV_CORE);
        telemetry.addData("Counts per inch (Core)", COUNTS_PER_INCH_CORE);
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
