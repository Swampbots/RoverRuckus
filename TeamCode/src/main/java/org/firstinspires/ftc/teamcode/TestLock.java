package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.LOCK_CLOSED;
import static org.firstinspires.ftc.teamcode.RoverHardware.LOCK_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STOWED;
import static org.firstinspires.ftc.teamcode.RoverHardware.RAMP_DOWN;
import static org.firstinspires.ftc.teamcode.RoverHardware.RAMP_UP;

//@Disabled
@TeleOp(name = "Lock Test", group ="Testing")
public class TestLock extends OpMode {

    RoverHardware hardware = new RoverHardware();



    private int frontTarget = PIV_STOWED[0];
    private int rearTarget = PIV_STOWED[1];

    private final double PIV_SPEED_BASE = 0.7;
    private final double PIV_SPEED_FRONT = PIV_SPEED_BASE;
    private final double PIV_SPEED_REAR = PIV_SPEED_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);

    private final double PIV_SPEED_SCALER_FRONT = 0.6;
    private final double PIV_SPEED_SCALER_REAR = PIV_SPEED_SCALER_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);


    public void init() {
        hardware.init(hardwareMap);
    }

    public void loop() {

        if(gamepad2.a)      hardware.setLockPosition(LOCK_OPEN);
        else if(gamepad2.b) hardware.setLockPosition(LOCK_CLOSED);

        if(gamepad2.x) hardware.ramp.setPosition(RAMP_UP);
        else hardware.ramp.setPosition(RAMP_DOWN);

        // Latch servo controls
        hardware.latch.setPosition(gamepad2.right_trigger * LATCH_OPEN);   // This will scale with the latch settings



        telemetry.addData("Left lock position", hardware.lockLeft.getPosition());
        telemetry.addData("Right lock position", hardware.lockRight.getPosition());
        telemetry.addLine();
        telemetry.addData("Front pivot", hardware.frontPivot.getCurrentPosition());
        telemetry.addData("Rear pivot", hardware.rearPivot.getCurrentPosition());
        telemetry.update();
    }
}
