package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STOWED;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STD;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_OMNI;

@TeleOp(name = "Pivot Test", group = "Testing")
public class RoverPivotTest extends OpMode {

    RoverHardware hardware = new RoverHardware();

    private int frontTarget = 0;
    private int rearTarget = 0;

    public void init() {
        hardware.init(hardwareMap);
    }


    public void loop() {
        // Drive Motors
        hardware.setLeftPower   (-gamepad1.left_stick_y);
        hardware.setRightPower  (-gamepad1.right_stick_y);

        // Wheel subassembly pivots
        hardware.rearPivot.setPower (gamepad2.left_stick_y);
        hardware.frontPivot.setPower(gamepad2.right_stick_y);

        if(gamepad1.a) {
            frontTarget = PIV_STOWED;
            rearTarget = PIV_STOWED;
        } else if(gamepad1.b) {
            frontTarget = PIV_KNEEL;
            rearTarget = PIV_KNEEL;
        } else if(gamepad1.x) {
            frontTarget = PIV_OMNI;
            rearTarget = PIV_STD;
        } else if(gamepad1.y) {
            frontTarget = PIV_STD;
            rearTarget = PIV_OMNI;
        }


    }
}
