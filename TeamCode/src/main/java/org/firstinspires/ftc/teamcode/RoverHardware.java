package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RoverHardware {

    HardwareMap hwMap;



    // Speed button variables
    public final double SLOW = 0.1;
    public final double NORMAL = 0.2;
    public final double FAST = 0.4;



    // Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor rearLeft;
    public DcMotor rearRight;

    public DcMotor rearPivot;
    public DcMotor frontPivot;



    // Servos
    public Servo latch;



    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;


        // Initialize motors
        frontLeft   = hwMap.dcMotor.get("fl_drive");
        frontRight  = hwMap.dcMotor.get("fr_drive");
        rearLeft    = hwMap.dcMotor.get("rl_drive");
        rearRight   = hwMap.dcMotor.get("rr_drive");

        rearPivot   = hwMap.dcMotor.get("rear_pivot");
        frontPivot  = hwMap.dcMotor.get("front_pivot");



        // Initialize servos
        latch   = hwMap.servo.get("latch");

    }

}
