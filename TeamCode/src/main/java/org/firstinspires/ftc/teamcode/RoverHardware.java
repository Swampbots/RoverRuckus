package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    // Pivot encoder positions
    public final double STOWED      = 0.0;
    public final double OMNI        = -1050.0;
    public final double STANDARD    = -2400.0;
    public final double KNEELING    = -2000.0;



    // Encoder variables (REV Core Hex Motors)
    public static final double COUNTS_PER_REV_CORE  = 288.0;
    public static final double GEAR_REDUCTION_CORE  = 1.0; // No reduction on drive motors currently
    public static final double COUNTS_PER_INCH_CORE = (COUNTS_PER_REV_CORE * GEAR_REDUCTION_CORE) / WHEEL_CIRCUMFERENCE; // To be confirmed

    // Encoder variables (REV HD Hex Motors)
    public static final double COUNTS_PER_REV_HD   = 1120.0;
    public static final double GEAR_REDUCTION_HD   = 4.16; // 4.16 rotations of the motor shaft is 1 rotation of the drive pivot
    public static final double COUNTS_PER_INCH_HD  = (COUNTS_PER_REV_HD * GEAR_REDUCTION_HD) / WHEEL_CIRCUMFERENCE; // To be confirmed


    // Pivot encoder variables
    public static final int PIV_STOWED  = 0;
    public static final int PIV_OMNI    = -1050;
    public static final int PIV_KNEEL   = -2000;
    public static final int PIV_STD     = -2500;

    // Motors
    public DcMotor frontLeft;
    public DcMotor rearLeft;
    public DcMotor frontRight;
    public DcMotor rearRight;

    public DcMotor rearPivot;
    public DcMotor frontPivot;

    public DcMotor winch;

    public DcMotor mineral;



    // Servos
    public Servo latch;



    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;


        // Initialize motors
        frontLeft   = hwMap.dcMotor.get("fl_drive");
        rearLeft    = hwMap.dcMotor.get("rl_drive");
        frontRight  = hwMap.dcMotor.get("fr_drive");
        rearRight   = hwMap.dcMotor.get("rr_drive");


        rearPivot   = hwMap.dcMotor.get("rear_pivot");
        frontPivot  = hwMap.dcMotor.get("front_pivot");

        winch       = hwMap.dcMotor.get("winch");

        mineral     = hwMap.dcMotor.get("mineral");



        // Set motor directions
        frontLeft.setDirection  (DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection   (DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection (DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection  (DcMotorSimple.Direction.REVERSE);

        rearPivot.setDirection  (DcMotorSimple.Direction.FORWARD);
        frontPivot.setDirection (DcMotorSimple.Direction.FORWARD);

        winch.setDirection      (DcMotorSimple.Direction.FORWARD);

        mineral.setDirection    (DcMotorSimple.Direction.FORWARD);


        // Set motor zero-power behaviors
        frontLeft.setZeroPowerBehavior  (DcMotor.ZeroPowerBehavior.FLOAT);
        rearLeft.setZeroPowerBehavior   (DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.FLOAT);
        rearRight.setZeroPowerBehavior  (DcMotor.ZeroPowerBehavior.FLOAT);

        rearPivot.setZeroPowerBehavior  (DcMotor.ZeroPowerBehavior.BRAKE);
        frontPivot.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);

        winch.setZeroPowerBehavior      (DcMotor.ZeroPowerBehavior.BRAKE);

        mineral.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.BRAKE);


        // Reset all encoders
        resetMotorEncoders();




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

    public void resetDriveEncoders() {
        frontLeft.setMode   (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode    (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode  (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode   (DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode   (DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode    (DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode   (DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetPivotEncoders() {
        frontPivot.setMode   (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearPivot.setMode    (DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontPivot.setMode   (DcMotor.RunMode.RUN_USING_ENCODER);
        rearPivot.setMode    (DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetMotorEncoders() {
        resetPivotEncoders();
        resetDriveEncoders();
    }

    public void stopAllMotors() {
        frontLeft.setPower  (0);
        rearLeft.setPower   (0);
        frontRight.setPower (0);
        rearRight.setPower  (0);

        rearPivot.setPower  (0);
        frontPivot.setPower (0);

        winch.setPower      (0);

        mineral.setPower    (0);
    }
}
