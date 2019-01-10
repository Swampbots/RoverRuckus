package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by SwampbotsAdmin on 10/22/2017.
 */

//@Disabled
@Autonomous(name = "PID test", group = "Testing")
//@Disabled
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






    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("DO NOT PRESS PLAY! Initializing hardware.");
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
                    180:    gp1.a
                */

            if(gamepad1.y) turnToHeadingPID(0);
            else if(gamepad1.y) turnToHeadingPID(0);
            else if(gamepad1.b) turnToHeadingPID(45);
            else if(gamepad1.x) turnToHeadingPID(90);
            else if(gamepad1.a) turnToHeadingPID(180);

            
            telemetry.addData("kP", hardware.pid.getP());
            telemetry.addData("kI", hardware.pid.getI());
            telemetry.addData("kD", hardware.pid.getD());
            telemetry.addLine();
            telemetry.addData("Heading", heading());
            telemetry.update();
        }

    }





    public void turnToHeadingPID(int target) throws InterruptedException {
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
        return imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}