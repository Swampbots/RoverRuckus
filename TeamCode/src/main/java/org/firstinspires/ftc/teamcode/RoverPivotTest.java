package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STOWED;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STD;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_OMNI;

@Disabled
@TeleOp(name = "Pivot Test", group = "Testing")
public class RoverPivotTest extends OpMode {

    RoverHardware hardware = new RoverHardware();

    private int frontTarget = PIV_STOWED[0];
    private int rearTarget = PIV_STOWED[1];



    public void init() {
        hardware.init(hardwareMap);
    }


    public void loop() {

        // Set drive motor powers
        hardware.setLeftPower   (-gamepad1.left_stick_y);
        hardware.setRightPower  (-gamepad1.right_stick_y);



//        // Set pivot powers
//        hardware.rearPivot.setPower (gamepad2.left_stick_y);
//        hardware.frontPivot.setPower(gamepad2.right_stick_y);

        // Handle pivot targets
        if(gamepad1.a) {
            frontTarget = PIV_STOWED[0];
            rearTarget = PIV_STOWED[1];
        } else if(gamepad1.b) {
            frontTarget = PIV_KNEEL[0];
            rearTarget = PIV_KNEEL[1];
        } else if(gamepad1.x) {
            frontTarget = PIV_OMNI[0];
            rearTarget = PIV_STD[1];
        } else if(gamepad1.y) {
            frontTarget = PIV_STD[0];
            rearTarget = PIV_OMNI[1];
        }

        // Set pivot targets
        hardware.frontPivot.setTargetPosition(frontTarget);
        hardware.rearPivot.setTargetPosition(rearTarget);




        // TELEMETRY
        telemetry.addData("Front Target",   frontTarget);
        telemetry.addData("Rear Target",    rearTarget);
        telemetry.addLine();
        telemetry.addData("PIV_STOWED", PIV_STOWED);
        telemetry.addData("PIV_KNEEL",  PIV_KNEEL);
        telemetry.addData("PIV_STD",    PIV_STD);
        telemetry.addData("PIV_OMNI",   PIV_OMNI);
        telemetry.update();


    }
}
