package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_CLOSED;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL_REAR;


@Autonomous(name = "Silver_CV", group = "Autonomous")
public class AutoSilver extends OpMode {

    RoverHardware hardware = new RoverHardware();

    GoldContourPipeline vision = new GoldContourPipeline();

    _GoldPlacement goldPlacement = _GoldPlacement.UNKNOWN;


    // IMU object
    BNO055IMU imu;

    Orientation angles;


    public final double DROP_TIME       = 3.0; // Seconds

    public final double DRIVE_DIST      = 4.0;  // Inches
    public final double SAMPLE_DIST     = 26.0;  // Inches
    public final double CRATER_DIST     = 11.5;  // Inches

    public final int SAMPLE_LEFT        = -40;  // Degrees
    public final int SAMPLE_RIGHT       = 40;  // Degrees



    // Servo positions
    public final double RAMP_STOWED = 0.0;

    public final double LOCK_OPEN   = 1.0;
    public final double LOCK_LOCKED = 0.0;




    // HSV Threshold input variables
    private final double THRESHOLD_STEP = 1.0;

    private final double HSV_MAX = 255.0;
    private final double HSV_MIN = 0.0;

    private double[] hsvHue = new double[]{100.0, 120.0};
    private double[] hsvSat = new double[]{0.0, 255.0};
    private double[] hsvVal = new double[]{0.0, 255.0};


    ButtonCooldown dpUp     = new ButtonCooldown();
    ButtonCooldown dpDown   = new ButtonCooldown();
    ButtonCooldown dpLeft   = new ButtonCooldown();
    ButtonCooldown dpRight  = new ButtonCooldown();

    ButtonCooldown a    = new ButtonCooldown();
    ButtonCooldown b    = new ButtonCooldown();
    ButtonCooldown x    = new ButtonCooldown();
    ButtonCooldown y    = new ButtonCooldown();

    ButtonCooldown lb   = new ButtonCooldown();
    ButtonCooldown rb   = new ButtonCooldown();
    ButtonCooldown lt   = new ButtonCooldown();
    ButtonCooldown rt   = new ButtonCooldown();

    private final double CTR_MAX_Y = 192.0;

    private final double CTR_MIN_Y = 0.0;

    private final int CTR_LEFT  = (int) ((CTR_MAX_Y + CTR_MIN_Y) / 3.0);        // 1/3 of the width to bound the left third     [ |  ]
    private final int CTR_RIGHT = (int) ((CTR_MAX_Y + CTR_MIN_Y) * 2.0 / 3.0);  // 2/3 of the width to bound the center third   [  | ]

    private double ctrXThreshold = CTR_MAX_Y;

    // Variable for thresholding LT and RT inputs, e.g. if(gamepad1.left_trigger > TRIGGER_THRESHOLD)
    public final double TRIGGER_THRESHOLD = 0.7;

    //----------------------------------------------------------------------------------------------

    public void init() {

        // Hardware and servos
        hardware.init(hardwareMap);


        hardware.latch.setPosition(LATCH_CLOSED);
        hardware.ramp.setPosition(RAMP_STOWED);
//        hardware.george.setPosition(GEORGE_STOWED);
        hardware.setLockPosition(LOCK_LOCKED);


        // Vision pipeline
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        vision.enable();


//        // IMU
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
//        IMUParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        IMUParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        IMUParameters.loggingEnabled      = true;
//        IMUParameters.loggingTag          = "IMU";
//        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu.initialize(IMUParameters);
//
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    //----------------------------------------------------------------------------------------------

    public void init_loop() {
        //-----------------------------------------------------------------------------------
        // START HSV THRESHOLD CONTROLS
        //-----------------------------------------------------------------------------------

            /*
                CONTROLS: (increase, decrease)

                Hue min: gp1.up,    gp1.down
                Hue max: gp1.y,     gp1.a

                Sat min: gp1.right, gp1.left
                Sat max: gp1.b,     gp1.x

                Val min: gp1.lb,    gp1.lt
                Val max: gp1.rb,    gp1.rt
             */

        // Modify threshold variables if the buttons are pressed and thresholds are within outer limits 0 & 255

        // Update runtime once every cycle
        double runtime = getRuntime();

        // HUE MINIMUM
        if(gamepad1.dpad_down && dpDown.ready(runtime)) {
            if (hsvHue[0] > HSV_MIN)   hsvHue[0] -= THRESHOLD_STEP;
            else                        hsvHue[0] = HSV_MIN;
            dpDown.updateSnapshot(runtime);
        }

        if(gamepad1.dpad_up && dpUp.ready(runtime)) {
            if(hsvHue[0] < hsvHue[1])  hsvHue[0] += THRESHOLD_STEP;
            else                        hsvHue[0] = hsvHue[1];
            dpUp.updateSnapshot(runtime);
        }


        // HUE MAXIMUM
        if(gamepad1.y && y.ready(runtime)) {
            if (hsvHue[1] < HSV_MAX)   hsvHue[1] += THRESHOLD_STEP;
            else                        hsvHue[1] = HSV_MAX;
            y.updateSnapshot(runtime);
        }

        if(gamepad1.a && a.ready(runtime)) {
            if(hsvHue[1] > hsvHue[0])  hsvHue[1] -= THRESHOLD_STEP;
            else                        hsvHue[1] = hsvHue[0];
            a.updateSnapshot(runtime);
        }




        // SAT MINIMUM
        if(gamepad1.dpad_left && dpLeft.ready(runtime)) {
            if (hsvSat[0] > HSV_MIN)   hsvSat[0] -= THRESHOLD_STEP;
            else                        hsvSat[0] = HSV_MIN;
            dpLeft.updateSnapshot(runtime);
        }

        if(gamepad1.dpad_right && dpRight.ready(runtime)) {
            if(hsvSat[0] < hsvSat[1])  hsvSat[0] += THRESHOLD_STEP;
            else                        hsvSat[0] = hsvSat[1];
            dpRight.updateSnapshot(runtime);
        }


        // SAT MAXIMUM
        if(gamepad1.b && b.ready(runtime)) {
            if (hsvSat[1] < HSV_MAX)   hsvSat[1] += THRESHOLD_STEP;
            else                        hsvSat[1] = HSV_MAX;
            b.updateSnapshot(runtime);
        }

        if(gamepad1.x && x.ready(runtime)) {
            if(hsvSat[1] > hsvSat[0])  hsvSat[1] -= THRESHOLD_STEP;
            else                        hsvSat[1] = hsvSat[0];
            x.updateSnapshot(runtime);
        }




        // VAL MINIMUM
        if(gamepad1.left_trigger > TRIGGER_THRESHOLD && lt.ready(runtime)) {
            if (hsvVal[0] > HSV_MIN)   hsvVal[0] -= THRESHOLD_STEP;
            else                        hsvVal[0] = HSV_MIN;
            lt.updateSnapshot(runtime);
        }

        if(gamepad1.left_bumper && lb.ready(runtime)) {
            if(hsvVal[0] < hsvVal[1])  hsvVal[0] += THRESHOLD_STEP;
            else                        hsvVal[0] = hsvVal[1];
            lb.updateSnapshot(runtime);
        }



        // VAL MAXIMUM
        if(gamepad1.right_trigger > TRIGGER_THRESHOLD && rt.ready(runtime)) {
            if (hsvVal[1] > hsvVal[0])  hsvVal[1] -= THRESHOLD_STEP;
            else                        hsvVal[1] = hsvVal[0];
            rt.updateSnapshot(runtime);
        }

        if(gamepad1.right_bumper && rb.ready(runtime)) {
            if(hsvVal[1] < HSV_MAX)     hsvVal[1] += THRESHOLD_STEP;
            else                        hsvVal[1] = HSV_MAX;
            rb.updateSnapshot(runtime);
        }

        //-----------------------------------------------------------------------------------
        // END HSV THRESHOLD CONTROLS
        //-----------------------------------------------------------------------------------




        // SET HSV THRESHOLDS
        vision.setHsvHue(hsvHue);
        vision.setHsvSat(hsvSat);
        vision.setHsvVal(hsvVal);



        // Contour array
        List<MatOfPoint> contours = vision.findContoursOutput();

        int contourHeightMid;
        int contourX;

        // Tally of contourPlacements for all visible contours this cycle
        // (Set all to 0 so they start over each cycle)
        int leftTally = 0;
        int centerTally = 0;
        int rightTally = 0;




        // TELEMETRY
        telemetry.addData("Hue min", hsvHue[0]);
        telemetry.addData("Hue max", hsvHue[1]);
        telemetry.addLine();
        telemetry.addData("Sat min", hsvSat[0]);
        telemetry.addData("Sat max", hsvSat[1]);
        telemetry.addLine();
        telemetry.addData("Val min", hsvVal[0]);
        telemetry.addData("Val max", hsvVal[1]);
        telemetry.addLine();
        telemetry.addLine();
        try {
            if(contours != null) {
                if(contours.size() > 0) {
                    for(int i = 0; i < contours.size(); i++) {
                        Rect boundingRect = Imgproc.boundingRect(contours.get(i));
                        contourX = (boundingRect.x + boundingRect.width) / 2;
                        contourHeightMid = (boundingRect.y + boundingRect.height) / 2;

                        if( contourX > ctrXThreshold){ // Make sure the contour isn't from gold in the crater

                            if (contourHeightMid < CTR_LEFT) {
                                leftTally++;
                            } else if (contourHeightMid < CTR_RIGHT) {
                                centerTally++;
                            } else {
                                rightTally++;
                            }
                        }
                    }
                }
            }
        } catch(Exception e) {
            e.printStackTrace();
            telemetry.addData("Exception", e.getMessage());
        }


        int[] ctrTallies = {leftTally, centerTally, rightTally};

        int highest = highestTally(ctrTallies);

        if(highest == leftTally) {
            goldPlacement = _GoldPlacement.LEFT;
        } else if(highest == centerTally) {
            goldPlacement = _GoldPlacement.CENTER;
        } else {
            goldPlacement = _GoldPlacement.RIGHT;
        }

        telemetry.addData("Left tally", leftTally);
        telemetry.addData("Center tally", centerTally);
        telemetry.addData("Right tally", rightTally);
        telemetry.addData("Highest", highest);
        telemetry.addLine();
        telemetry.addData("Gold Placement", goldPlacement);
        telemetry.update();
    }



    //----------------------------------------------------------------------------------------------



    public void start() {

        hardware.latch.setPosition(LATCH_CLOSED);

        // Open lock
        hardware.setLockPosition(LOCK_OPEN);

        // Wait to drop
        double runtime = getRuntime();
        while(getRuntime() - runtime < DROP_TIME) {
            telemetry.addLine("Waiting...");
            telemetry.addData("Elapsed Time", getRuntime() - runtime);
            telemetry.update();
        }

        // Lock lock
        hardware.setLockPosition(LOCK_LOCKED);

        // Deploy the wheels
        hardware.frontPivot.setTargetPosition(PIV_KNEEL_FRONT);
        hardware.rearPivot.setTargetPosition(PIV_KNEEL_REAR);

        // Open latch
        hardware.latch.setPosition(LATCH_OPEN);

        // Drive away from lander
//        hardware.driveInches(DRIVE_DISTANCE);

//        // Turn towards gold sample
//        switch(goldPlacement) {
//            case LEFT:
//                hardware.frontPivot.setTargetPosition(PIV_OMNI_FRONT);
//                hardware.rearPivot.setTargetPosition(PIV_STD_REAR);
//
//                try {
//                    turnToHeadingPID(SAMPLE_LEFT);
//                } catch(InterruptedException e) {
//                    telemetry.addLine("PID turn interrupted");
//                    telemetry.update();
//
//                    stop();
//                }
//
//                hardware.frontPivot.setTargetPosition(PIV_MINE_FRONT);
//                hardware.rearPivot.setTargetPosition(PIV_STD_REAR);
//
//                hardware.snorfler.setPower(-1.0);
//
//                //hardware.driveInches(SAMPLE_DIST);
//
//                break;
//
//            case RIGHT:
//                hardware.frontPivot.setTargetPosition(PIV_OMNI_FRONT);
//                hardware.rearPivot.setTargetPosition(PIV_STD_REAR);
//
//                try {
//                    turnToHeadingPID(SAMPLE_LEFT);
//                } catch(InterruptedException e) {
//                    telemetry.addLine("PID turn interrupted");
//                    telemetry.update();
//
//                    stop();
//                }
//
//                hardware.frontPivot.setTargetPosition(PIV_MINE_FRONT);
//                hardware.rearPivot.setTargetPosition(PIV_STD_REAR);
//
//                hardware.snorfler.setPower(-1.0);
//
//                //hardware.driveInches(SAMPLE_DIST);
//
//                break;
//
//            default:    // Center and unknown are considered default
//                hardware.frontPivot.setTargetPosition(PIV_MINE_FRONT);
//                hardware.rearPivot.setTargetPosition(PIV_STD_REAR);
//
//                hardware.snorfler.setPower(-1.0);
//
//                //hardware.driveInches(SAMPLE_DIST);
//
//                break;
//        }
//
//        // Drive to crater
//        //hardware.driveInches(CRATER_DIST);

    }



    //----------------------------------------------------------------------------------------------



    public void loop() {

    }

    public void stop() {
        vision.disable();
    }




    private int highestTally(int[] tallies) {
        int highest;

        if(tallies[0] > tallies[1] && tallies[0] > tallies[2]) highest = tallies[0];
        else if(tallies[1] > tallies[2]) highest = tallies[1];
        else highest = tallies[2];

        return highest;
    }




    public void turnToHeadingPID(int target) throws InterruptedException {

        hardware.pid.setSetpoint(target);                                       // Set target final heading relative to current
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);   // Set maximum motor power
        hardware.pid.setDeadband(hardware.TOLERANCE);                           // Set how far off you can safely be from your target

        double error = normalize180(target - heading());

        while (Math.abs(error) < hardware.TOLERANCE) {
            error = normalize180(target - heading());
            double power = hardware.pid.calculateGivenError(error);

            hardware.setLeftPower(power);
            hardware.setRightPower(-power);

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
