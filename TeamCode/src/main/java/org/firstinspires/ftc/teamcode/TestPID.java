package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STOWED;

/**
 * Created by SwampbotsAdmin on 10/22/2017.
 */

//@Disabled
@Autonomous(name = "PID test", group = "Testing")
public class TestPID extends LinearOpMode {

    // IMU object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    RoverHardware hardware = new RoverHardware();

    // Button cooldowns
    GamepadCooldowns cooldowns = new GamepadCooldowns();

    // Variable for thresholding LT and RT inputs, e.g. if(gamepad1.left_trigger > TRIGGER_THRESHOLD)
    public final double TRIGGER_THRESHOLD = 0.7;

    // Local runtime snapshot
    private double runtime;

    // PID coefficients
    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;

    private final double K_STEP = 0.005;



    private final double PIV_SPEED_BASE = 1.0;
    private final double PIV_SPEED_FRONT = PIV_SPEED_BASE;
    private final double PIV_SPEED_REAR = PIV_SPEED_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);

    private final double PIV_SPEED_SCALER_FRONT = 1.0;
    private final double PIV_SPEED_SCALER_REAR = PIV_SPEED_SCALER_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);



    private int frontTarget = PIV_STOWED[0];
    private int rearTarget = PIV_STOWED[1];


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Gyro initializing! Do not touch the robot!");
        telemetry.update();

        hardware.init(hardwareMap);

        cooldowns.setCooldown(0.150); // 150 milliseconds

        // Set up the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // IMU parameters
        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IMUParameters.loggingEnabled      = true;
        IMUParameters.loggingTag          = "IMU";
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Pass in the parameters
        imu.initialize(IMUParameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addLine("Hardware initialized.");
        telemetry.addLine("Press play to start.");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            //--------------------------------------------------------------------------------------
            // START PID COEFFICIENT CONTROLS
            //--------------------------------------------------------------------------------------

                /*
                    CONTROLS: (increase, decrease)

                    P: gp1.up,      gp1.down
                    I: gp1.right,   gp1.left
                    D: gp1.lb,      gp1.lt
                */

            runtime = getRuntime();


            // Proportional coefficient-------------------------------------------------------------
            if(gamepad1.dpad_up && cooldowns.dpUp.ready(runtime)) {
                kP += K_STEP;
                cooldowns.dpUp.updateSnapshot(runtime);
            }

            if(gamepad1.dpad_down && cooldowns.dpDown.ready(runtime)) {
                if(kP < K_STEP) kP = 0.0;
                else            kP -= K_STEP;
                cooldowns.dpDown.updateSnapshot(runtime);
            }


            // Integral coefficient-----------------------------------------------------------------
            if(gamepad1.dpad_right && cooldowns.dpRight.ready(runtime)) {
                kI += K_STEP;
                cooldowns.dpRight.updateSnapshot(runtime);
            }

            if(gamepad1.dpad_left && cooldowns.dpLeft.ready(runtime)) {
                if(kI < K_STEP) kI = 0.0;
                else            kI -= K_STEP;
                cooldowns.dpLeft.updateSnapshot(runtime);
            }


            // Derivative coefficient---------------------------------------------------------------
            if(gamepad1.left_bumper && cooldowns.lb.ready(runtime)) {
                kD += K_STEP;
                cooldowns.lb.updateSnapshot(runtime);
            }

            if(gamepad1.left_trigger > TRIGGER_THRESHOLD && cooldowns.lt.ready(runtime)) {
                if(kD < K_STEP) kD = 0.0;
                else            kD -= K_STEP;
                cooldowns.lt.updateSnapshot(runtime);
            }

            //--------------------------------------------------------------------------------------
            // END PID COEFFICIENT CONTROLS
            //--------------------------------------------------------------------------------------

            // Set PID coefficients
            hardware.pid.setPID(kP, kI, kD);


                /*
                    CONTROLS: (Target heading listed)

                    0:      gp1.y
                    45:     gp1.b
                    90:     gp1.x
                */

            if(gamepad1.y) turnToHeadingPID(0);
            else if(gamepad1.b) turnToHeadingPID(45);
            else if(gamepad1.x) turnToHeadingPID(90);


            // Drive motor and pivot motor controls
            // Drive motor controls
            hardware.setLeftPower   (-gamepad1.left_stick_y);
            hardware.setRightPower  (-gamepad1.right_stick_y);




            // Set pivot target, run mode, and speed
            if(Math.abs(gamepad2.right_stick_y) < 0.05) {
                hardware.frontPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.frontPivot.setPower(PIV_SPEED_FRONT);
            } else {
                hardware.frontPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardware.frontPivot.setPower(-gamepad2.right_stick_y * PIV_SPEED_SCALER_FRONT);
                frontTarget = hardware.frontPivot.getCurrentPosition();
            }

            if(Math.abs(gamepad2.left_stick_y) < 0.05) {
                hardware.rearPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.rearPivot.setPower(PIV_SPEED_REAR);
            } else {
                hardware.rearPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardware.rearPivot.setPower(-gamepad2.left_stick_y * PIV_SPEED_SCALER_REAR);
                rearTarget = hardware.rearPivot.getCurrentPosition();
            }

            // Set pivot targets
            hardware.frontPivot.setTargetPosition(frontTarget);
            hardware.rearPivot.setTargetPosition(rearTarget);


            telemetry.addData("kP", hardware.pid.getP());
            telemetry.addData("kI", hardware.pid.getI());
            telemetry.addData("kD", hardware.pid.getD());
            telemetry.addLine();
            telemetry.addLine("Headings:");
            telemetry.addData("Heading", heading());
            telemetry.update();
        }

    }





    public void turnToHeadingPID(int target) throws InterruptedException {

        telemetry.addData("Turning to target", target);
        telemetry.update();
        sleep(250);

        hardware.pid.setSetpoint(target);                                       // Set target final heading relative to current
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);   // Set maximum motor power
        hardware.pid.setDeadband(hardware.TOLERANCE);                           // Set how far off you can safely be from your target

        while (opModeIsActive()) {
            double error = normalize180(target - heading());
            double power = hardware.pid.calculateGivenError(error);

            hardware.setLeftPower(power);
            hardware.setRightPower(-power);

            if (Math.abs(error) < hardware.TOLERANCE) {
                break;
            }

            Thread.sleep(1);
        }

        hardware.setLeftPower(0);
        hardware.setRightPower(0);
    }

    public double normalize180(double angle) {
        while(angle > 180) {
            angle -= 360;
        }
        while(angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    public double heading() {
        return imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
    }
}