package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RoverHardware {

    HardwareMap hwMap;


    // Hardware dimensions
    public static final double WHEEL_CIRCUMFERENCE = 4.0 * Math.PI; // 4-inch wheel diameter * pi = circumference (about 12.57 inches)




    // Speed button variables
    public static final double SLOW = 0.1;
    public static final double NORMAL = 0.2;
    public static final double FAST = 0.4;


    // Encoder variables (REV Core Hex Motors)
    public static final int     COUNTS_PER_REV_CORE  = 288;
    public static final double  GEAR_REDUCTION_CORE  = 1.0; // No reduction on drive motors currently
    public static final int     COUNTS_PER_INCH_CORE = (int)((COUNTS_PER_REV_CORE * GEAR_REDUCTION_CORE) / WHEEL_CIRCUMFERENCE);

    // Encoder variables (Pivot motors)
    public static final int     COUNTS_PER_REV_HD   = 1120;

    public static final double  GEAR_REDUCTION_HD_REAR      = (24.0 / 16.0) * (26.0 / 15.0); // 16-t (output shaft) -> 24-t (transfer shaft) -- 15-t (same shaft) -> 26-t (pivot shaft)
    public static final double  GEAR_REDUCTION_HD_FRONT     = (24.0 / 16.0) * (26.0 / 15.0); // 16-t (output shaft) -> 24-t (transfer shaft) -- 15-t (same shaft) -> 26-t (pivot shaft)


    public static final int     COUNTS_PER_DEGREE_HD_REAR   = (int)((COUNTS_PER_REV_HD * GEAR_REDUCTION_HD_REAR) / 360);
    public static final int     COUNTS_PER_DEGREE_HD_FRONT  = (int)((COUNTS_PER_REV_HD * GEAR_REDUCTION_HD_FRONT) / 360);

    public static final int     COUNTS_PER_DEGREE_HD_FLIPPER = (int)(COUNTS_PER_REV_HD / 360);


    // Encoder variables (Drive motors)
    public static final double  GEAR_REDUCTION_DRIVE_REAR = (15.0 / 20.0);
    public static final double  GEAR_REDUCTION_DRIVE_FRONT = 1.0;

    public static final int     COUNTS_PER_INCH_DRIVE_REAR     = (int)((COUNTS_PER_REV_HD * GEAR_REDUCTION_DRIVE_REAR) / WHEEL_CIRCUMFERENCE);
    public static final int     COUNTS_PER_INCH_DRIVE_FRONT    = (int)((COUNTS_PER_REV_HD * GEAR_REDUCTION_DRIVE_FRONT) / WHEEL_CIRCUMFERENCE);






    public static final int PIV_DEGREE_OFFSET = 10;


//    // Rear pivot position variables
//    public static final int PIV_STOWED_REAR = 0;                                                            // Always start at 0
//    public static final int PIV_OMNI_REAR   = -COUNTS_PER_DEGREE_HD_REAR * (90 + PIV_DEGREE_OFFSET);        // 90 degrees out from start
//    public static final int PIV_KNEEL_REAR  = -COUNTS_PER_DEGREE_HD_REAR * (135 + PIV_DEGREE_OFFSET);       // 135 for both wheels on the ground
//    public static final int PIV_STD_REAR    = -COUNTS_PER_DEGREE_HD_REAR * (180 + PIV_DEGREE_OFFSET);       // 180 for standard wheel on the ground
//    public static final int PIV_MINE_REAR   = -1450;                                                        // Exact degrees TBD
//
//    // Front pivot position variables
//    public static final int PIV_STOWED_FRONT    = 0;                                                        // Always start at 0
//    public static final int PIV_OMNI_FRONT      = -COUNTS_PER_DEGREE_HD_FRONT * (90 + PIV_DEGREE_OFFSET);   // 90 degrees out from start
//    public static final int PIV_KNEEL_FRONT     = -COUNTS_PER_DEGREE_HD_FRONT * (135 + PIV_DEGREE_OFFSET);  // 135 for both wheels on the ground
//    public static final int PIV_STD_FRONT       = -COUNTS_PER_DEGREE_HD_FRONT * (180 + PIV_DEGREE_OFFSET);  // 180 for standard wheel on the ground
//    public static final int PIV_MINE_FRONT      = (int)(-600 * GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);                                                     // Exact degrees TBD

    public static final int PIV_STOWED_REAR = 0;
    public static final int PIV_OMNI_REAR   = -700;
    public static final int PIV_KNEEL_REAR  = -900;
    public static final int PIV_STD_REAR    = -1400;
    public static final int PIV_MINE_REAR   = -1;

    public static final int PIV_STOWED_FRONT    = 0;
    public static final int PIV_OMNI_FRONT      = -700;
    public static final int PIV_KNEEL_FRONT     = -900;
    public static final int PIV_STD_FRONT       = -1400;
    public static final int PIV_MINE_FRONT      = -1;


    // Pivot state variables
    public static final int[] PIV_STOWED    = {PIV_STOWED_FRONT, PIV_STOWED_REAR};
    public static final int[] PIV_OMNI      = {PIV_OMNI_FRONT, PIV_OMNI_REAR};
    public static final int[] PIV_KNEEL     = {PIV_KNEEL_FRONT, PIV_KNEEL_REAR};
    public static final int[] PIV_STD       = {PIV_STD_FRONT, PIV_STD_REAR};
    public static final int[] PIV_MINE      = {PIV_MINE_FRONT, PIV_MINE_REAR};




    // Flipper state variables

    public static final int FLIPPER_STOW = 0;
    public static final int FLIPPER_LOAD = -COUNTS_PER_DEGREE_HD_FLIPPER * 35;
    public static final int FLIPPER_FLIP = -COUNTS_PER_DEGREE_HD_FLIPPER * 90;

    public void nextFlipper() {
        if(snorfler.getCurrentPosition() == FLIPPER_STOW) snorfler.setTargetPosition(FLIPPER_LOAD);
        else if(snorfler.getCurrentPosition() == FLIPPER_LOAD) snorfler.setTargetPosition(FLIPPER_FLIP);
    }

    public void prevFlipper() {
        if(snorfler.getCurrentPosition() == FLIPPER_FLIP) snorfler.setTargetPosition(FLIPPER_LOAD);
        else if(snorfler.getCurrentPosition() == FLIPPER_LOAD) snorfler.setTargetPosition(FLIPPER_STOW);
    }




    // Servo position variables
    public static final double LOCK_OPEN    = 1.0;
    public static final double LOCK_CLOSED  = 0.0;

    public static final double LATCH_CLOSED = 0.0;
    public static final double LATCH_OPEN   = 0.6;

    public static final double RAMP_UP   = 1.0;
    public static final double RAMP_DOWN = 0.0;


    // Autonomous PID variables
    public final double MAX_SPEED = 0.4;
    public final double P = 0.045;
    public final double I = 0.01;
    public final double D = 0.045;
    public final double TOLERANCE = 2;

    public final SynchronousPID pid = new SynchronousPID(P, I, D);


    // Motors
    public DcMotor frontLeft;
    public DcMotor rearLeft;
    public DcMotor frontRight;
    public DcMotor rearRight;

    public DcMotor rearPivot;
    public DcMotor frontPivot;

    public DcMotor snorfler;

    public DcMotor flipper;



    // Servos
    public Servo latch;
    public Servo stop;

    public Servo lifter;
    public Servo ramp;

    public Servo lockLeft;
    public Servo lockRight;



    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;


        // Initialize motors
        frontLeft   = hwMap.dcMotor.get("fl_drive");
        rearLeft    = hwMap.dcMotor.get("rl_drive");
        frontRight  = hwMap.dcMotor.get("fr_drive");
        rearRight   = hwMap.dcMotor.get("rr_drive");


        rearPivot   = hwMap.dcMotor.get("rear_pivot");
        frontPivot  = hwMap.dcMotor.get("front_pivot");

        snorfler = hwMap.dcMotor.get("snorfler");

        flipper = hwMap.dcMotor.get("flipper");



        // Set motor directions
        frontLeft.setDirection  (DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection   (DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection (DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection  (DcMotorSimple.Direction.FORWARD);

        rearPivot.setDirection  (DcMotorSimple.Direction.REVERSE);
        frontPivot.setDirection (DcMotorSimple.Direction.REVERSE);

        snorfler.setDirection      (DcMotorSimple.Direction.FORWARD);

        flipper.setDirection    (DcMotorSimple.Direction.FORWARD);


        // Set motor zero-power behaviors
        frontLeft.setZeroPowerBehavior  (DcMotor.ZeroPowerBehavior.FLOAT);
        rearLeft.setZeroPowerBehavior   (DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.FLOAT);
        rearRight.setZeroPowerBehavior  (DcMotor.ZeroPowerBehavior.FLOAT);

        rearPivot.setZeroPowerBehavior  (DcMotor.ZeroPowerBehavior.BRAKE);
        frontPivot.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);

        snorfler.setZeroPowerBehavior      (DcMotor.ZeroPowerBehavior.BRAKE);

        flipper.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.BRAKE);


        // Reset all encoders
        resetMotorEncoders();




        // Initialize servos
        latch   = hwMap.servo.get("latch");
        stop = hwMap.servo.get("stop");

        lifter = hwMap.servo.get("lifter");
        ramp = hwMap.servo.get("ramp");

        lockLeft = hwMap.servo.get("lock_left");
        lockRight = hwMap.servo.get("lock_right");

        ramp.setPosition(0.0);

        lockLeft.scaleRange(0.25, 0.75);
        lockRight.scaleRange(0.25, 0.75);
    }




    // Motor methods

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

        snorfler.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        snorfler.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopAllMotors() {
        frontLeft.setPower  (0);
        rearLeft.setPower   (0);
        frontRight.setPower (0);
        rearRight.setPower  (0);

        rearPivot.setPower  (0);
        frontPivot.setPower (0);

        snorfler.setPower   (0);

        flipper.setPower    (0);
    }



    public void driveCounts(int frontTarget, int rearTarget, double speed) {
        frontLeft.setTargetPosition    (frontLeft.getCurrentPosition()    + (int)(frontTarget * GEAR_REDUCTION_DRIVE_FRONT));
        rearLeft.setTargetPosition     (rearLeft.getCurrentPosition()     + (int)(rearTarget * GEAR_REDUCTION_DRIVE_REAR));
        frontRight.setTargetPosition   (frontRight.getCurrentPosition()   + (int)(frontTarget * GEAR_REDUCTION_DRIVE_FRONT));
        rearRight.setTargetPosition    (rearRight.getCurrentPosition()    + (int)(rearTarget * GEAR_REDUCTION_DRIVE_REAR));

        frontLeft.setMode  (DcMotor.RunMode.RUN_TO_POSITION);
        rearLeft.setMode   (DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode (DcMotor.RunMode.RUN_TO_POSITION);
        rearRight.setMode  (DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower (speed);
        rearLeft.setPower  (speed);
        frontRight.setPower(-speed);
        rearRight.setPower (-speed);

        while(  frontLeft  .isBusy() &&
                rearLeft   .isBusy() &&
                frontRight .isBusy() &&
                rearRight  .isBusy());

        frontLeft  .setPower(0);
        rearLeft   .setPower(0);
        frontRight .setPower(0);
        rearRight  .setPower(0);

        frontLeft.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode   (DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveInches(double inches, double speed) {
        driveCounts(
                (int) (inches * COUNTS_PER_INCH_DRIVE_FRONT),
                (int) (inches * COUNTS_PER_INCH_DRIVE_REAR),
                speed
        );
    }






    // Servo methods

    public void setLockPosition(double position) {
        lockLeft.setPosition(position);
        lockRight.setPosition(position);
    }
}
