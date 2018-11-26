package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_REV_HD;

@TeleOp(name = "Flipper Test", group = "Testing")
public class FlipperTest extends OpMode {

    RoverHardware hardware = new RoverHardware();

    private final double FLIPPER_SPEED_BASE = 0.05;
    private final double FLIPPER_SPEED_SCALER = 0.6;


    private final int COUNTS_PER_REV_HD_20 = COUNTS_PER_REV_HD / 2; // Converting from a 40:1 motor to a 20:1
    private final double GEAR_REDUCTION_FLIPPER = 1.0; // 1:1 gear ratio
    private final int COUNTS_PER_DEGREE_REV_HD_20 = (int)((COUNTS_PER_REV_HD_20 * GEAR_REDUCTION_FLIPPER) / 480);


    public int flipperTarget = 0;

    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {

        // Set the flipper power (using the snorfler motor lol)
        hardware.mineral.setPower(gamepad1.left_stick_y * FLIPPER_SPEED_BASE);

        if(gamepad1.a) flipperTarget = 0;
        if(gamepad1.y) flipperTarget = COUNTS_PER_DEGREE_REV_HD_20 * 180;

        hardware.mineral.setTargetPosition(flipperTarget);

        // Handle flipper target, run mode, and speed
        if(Math.abs(gamepad1.right_stick_y) < 0.05) {
            hardware.mineral.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.mineral.setPower(FLIPPER_SPEED_BASE);
        } else {
            hardware.mineral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.mineral.setPower(gamepad1.right_stick_y * FLIPPER_SPEED_SCALER);
            flipperTarget = hardware.mineral.getCurrentPosition();
        }


    }
}
