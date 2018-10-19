package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RoverHardware {

    HardwareMap hwMap;


    // Hardware dimensions
    public static final double WHEEL_CIRCUMFERENCE = 4.0 * Math.PI; // 4-inch diameter * pi = circumference (about 12.57 inches)




    // Speed button variables
    public final double SLOW = 0.1;
    public final double NORMAL = 0.2;
    public final double FAST = 0.4;


    // Encoder variables (REV Core Hex Motors)
    public static final double COUNTS_PER_REV_CORE  = 288.0;
    public static final double GEAR_REDUCTION_CORE  = 4.16; // 4.16 rotations of the motor shaft is 1 rotation of the drive pivot
    public static final double COUNTS_PER_INCH_CORE = (COUNTS_PER_REV_CORE * GEAR_REDUCTION_CORE) / WHEEL_CIRCUMFERENCE; // To be confirmed

    // Encoder variables (REV HD Hex Motors)
    public static final double COUNTS_PER_REV_HD   = 1120.0;
    public static final double GEAR_REDUCTION_HD   = 4.16; // 4.16 rotations of the motor shaft is 1 rotation of the drive pivot
    public static final double COUNTS_PER_INCH_HD  = (COUNTS_PER_REV_HD * GEAR_REDUCTION_HD) / WHEEL_CIRCUMFERENCE; // To be confirmed




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



    public void setLeftPower(double power) {
        frontLeft   .setPower(power);
        rearLeft    .setPower(power);
    }

    public void setRightPower(double power) {
        frontRight  .setPower(power);
        rearRight   .setPower(power);
    }
}
