package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_HD_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_STOWED;

@Autonomous(name = "PID Test (2)", group = "Testing")
public class TestPID2 extends LinearOpMode {


    private final double PIV_SPEED_BASE = 1.0;
    private final double PIV_SPEED_FRONT = PIV_SPEED_BASE;
    private final double PIV_SPEED_REAR = PIV_SPEED_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);

    private final double PIV_SPEED_SCALER_FRONT = 1.0;
    private final double PIV_SPEED_SCALER_REAR = PIV_SPEED_SCALER_FRONT * (GEAR_REDUCTION_HD_REAR / GEAR_REDUCTION_HD_FRONT);



    private int frontTarget = PIV_STOWED[0];
    private int rearTarget = PIV_STOWED[1];

    RoverHardware hardware = new RoverHardware();


    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        hardware.init(hardwareMap);

        composeTelemetry();



        waitForStart();


        while (opModeIsActive()) {
            telemetry.update(); // Also updates angles variable

            /*
                    CONTROLS: (Target heading listed)

                    0:      gp1.y
                    45:     gp1.b
                    90:     gp1.x
                */



            // Controls
            // Set drive motor power
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

            if(gamepad1.y) turnToHeadingPID(0);
            else if(gamepad1.b) turnToHeadingPID(45);
            else if(gamepad1.x) turnToHeadingPID(90);


        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    //----------------------------------------------------------------------------------------------
    // PID controller methods
    //----------------------------------------------------------------------------------------------

    public void turnToHeadingPID(int target) throws InterruptedException {

        telemetry.addData("Turning to target", target);
        telemetry.addLine("Press dpad_down to stop.");

        hardware.pid.setSetpoint(target);                                       // Set target final heading relative to current
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);   // Set maximum motor power
        hardware.pid.setDeadband(hardware.TOLERANCE);                           // Set how far off you can safely be from your target

        while (opModeIsActive()) {
            double error = normalize180(target - heading());
            double power = hardware.pid.calculateGivenError(error);

            telemetry.addData("Current error", error);
            telemetry.addData("Current power", power);

            hardware.setLeftPower(power);
            hardware.setRightPower(-power);

            if (Math.abs(error) < hardware.TOLERANCE || gamepad1.dpad_down) {
                break;
            }

            Thread.sleep(1);

            telemetry.update();
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

    public float heading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
