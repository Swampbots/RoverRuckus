package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@TeleOp(name = "Pivot Test", group = "Testing")
public class TestPivot extends OpMode {

    RoverHardware hardware = new RoverHardware();

    private int frontTarget = PIV_STOWED[0];
    private int rearTarget = PIV_STOWED[1];

    private final double PIV_SPEED_FRONT = 1.0;
    private final double PIV_SPEED_REAR = PIV_SPEED_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);

    private final int GAMEPAD_SENSITIVITY = 20;



    public void init() {
        hardware.init(hardwareMap);

        hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void loop() {

        // Set drive motor powers
        hardware.setLeftPower   (-gamepad1.left_stick_y);
        hardware.setRightPower  (-gamepad1.right_stick_y);


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

        // Set speed
        hardware.frontPivot.setPower(PIV_SPEED_FRONT);
        hardware.rearPivot.setPower(PIV_SPEED_REAR);


        // END TARGET CONTROLS




        // TELEMETRY
        telemetry.addData("Front Target",   frontTarget);
        telemetry.addData("Rear Target",    rearTarget);
        telemetry.addLine();
        telemetry.update();
    }

}
