package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_INCH_DRIVE_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_INCH_DRIVE_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.CTR_MAX_X;
import static org.firstinspires.ftc.teamcode.RoverHardware.CTR_MAX_Y;
import static org.firstinspires.ftc.teamcode.RoverHardware.CTR_MIN_X;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_DRIVE_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_DRIVE_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEORGE_DEPLOY;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEORGE_STOW;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_CLOSED;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.LOCK_CLOSED;
import static org.firstinspires.ftc.teamcode.RoverHardware.LOCK_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_OMNI_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.RAMP_DOWN;
import static org.firstinspires.ftc.teamcode.RoverHardware.RAMP_UP;


/*
    This is our autonomous program for when we are starting on the gold side of the field (nearest the depot).
    Before pressing play, we dial in the vision software using the gamepads. The vision software makes a decision as soon as we press play.
    Once the decision is finalized, we descend, move away from the lander, turn towards the gold mineral, displace it, move into the depot, place the marker, and move away.
    Once finished, it displays telemetry on the starting position of the gold mineral and the current heading of the robot.
 */

@Autonomous(name = "Gold_CV", group = "Autonomous") // Register this class as an Autonomous program
public class AutoGold extends LinearOpMode {

    RoverHardware hardware = new RoverHardware();   // Make a copy of the hardware class

    GoldContourPipeline vision = new GoldContourPipeline(); // Make a copy of the OpenCV pipeline (for vision processing)

    _GoldPlacement goldPlacement = _GoldPlacement.CENTER;   // Variable for tracking placement of the gold cube in the sample field


    // IMU object
    BNO055IMU imu;  // Object for accessing the gyroscope data from the IMU.

    Orientation angles; // Outdated object for storing orientation of robot


    // Autonomous constants

    public final double DROP_TIME       = 3.0; // Seconds

    public final double DRIVE_DIST      = 8.0;  // Inches
    public final double SAMPLE_OUTSIDE_DIST = 20.0;  // Inches
    public final double SAMPLE_CENTER_DIST  = 17.0;  // Inches
    public final double DEPOT_OUTSIDE_DIST  = 6.0;  // Inches
    public final double DEPOT_CENTER_DIST   = 25.0; // Inches

    public final double DRIVE_SPEED     = 0.8; // Power
    public final double SAMPLE_SPEED    = 0.8; // Power
    public final double CRATER_SPEED    = 0.8; // Power

    public final int SAMPLE_LEFT        = 40;  // Degrees
    public final int SAMPLE_RIGHT       = -37; // Degrees

    public final double DEPLOY_SPEED = 1.0;

    public final int REAR_DEPLOY_TARGET = -800; // Counts




    // HSV Threshold input variables
    private final double THRESHOLD_STEP = 1.0;

    private final double HSV_MAX = 255.0;
    private final double HSV_MIN = 0.0;

    private double[] hsvHue = new double[]{95.0, 120.0};
    private double[] hsvSat = new double[]{100.0, 255.0};
    private double[] hsvVal = new double[]{130.0, 255.0};


    // Gamepad button cooldowns
    GamepadCooldowns cooldowns = new GamepadCooldowns();

    ButtonCooldown gp2_a    = new ButtonCooldown();
    ButtonCooldown gp2_b    = new ButtonCooldown();



    // Camera view partitions
    private final int CTR_RIGHT  = (int) ((CTR_MAX_Y + hardware.CTR_MIN_Y) / 3.0);        // 1/3 of the width to bound the left third     [ |  ]
    private final int CTR_LEFT = (int) ((CTR_MAX_Y + hardware.CTR_MIN_Y) * 2.0 / 3.0);  // 2/3 of the width to bound the center third   [  | ]

    // OpenCV pipeline inputs (these will be tuned in during initialization)
    private double ctrXThreshold = 206.0;
    private double ctrMinArea   = 0.0;

    // Tallying
    int highest = 0;
    int highestArea = 0;

    // Variable for thresholding LT and RT inputs, e.g. if(gamepad1.left_trigger > TRIGGER_THRESHOLD)
    public final double TRIGGER_THRESHOLD = 0.7;


    public void runOpMode() {
        // Init

        hardware.init(hardwareMap);


        // Vision pipeline
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        vision.enable();


//        // IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        // Init_loop
        while(!opModeIsActive() && !isStopRequested()) {
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
            if(gamepad1.dpad_down && cooldowns.dpDown.ready(runtime)) {
                if (hsvHue[0] > HSV_MIN)   hsvHue[0] -= THRESHOLD_STEP;
                else                        hsvHue[0] = HSV_MIN;
                cooldowns.dpDown.updateSnapshot(runtime);
            }

            if(gamepad1.dpad_up && cooldowns.dpUp.ready(runtime)) {
                if(hsvHue[0] < hsvHue[1])  hsvHue[0] += THRESHOLD_STEP;
                else                        hsvHue[0] = hsvHue[1];
                cooldowns.dpUp.updateSnapshot(runtime);
            }


            // HUE MAXIMUM
            if(gamepad1.y && cooldowns.y.ready(runtime)) {
                if (hsvHue[1] < HSV_MAX)   hsvHue[1] += THRESHOLD_STEP;
                else                        hsvHue[1] = HSV_MAX;
                cooldowns.y.updateSnapshot(runtime);
            }

            if(gamepad1.a && cooldowns.a.ready(runtime)) {
                if(hsvHue[1] > hsvHue[0])  hsvHue[1] -= THRESHOLD_STEP;
                else                        hsvHue[1] = hsvHue[0];
                cooldowns.a.updateSnapshot(runtime);
            }




            // SAT MINIMUM
            if(gamepad1.dpad_left && cooldowns.dpLeft.ready(runtime)) {
                if (hsvSat[0] > HSV_MIN)   hsvSat[0] -= THRESHOLD_STEP;
                else                        hsvSat[0] = HSV_MIN;
                cooldowns.dpLeft.updateSnapshot(runtime);
            }

            if(gamepad1.dpad_right && cooldowns.dpRight.ready(runtime)) {
                if(hsvSat[0] < hsvSat[1])  hsvSat[0] += THRESHOLD_STEP;
                else                        hsvSat[0] = hsvSat[1];
                cooldowns.dpRight.updateSnapshot(runtime);
            }


            // SAT MAXIMUM
            if(gamepad1.b && cooldowns.b.ready(runtime)) {
                if (hsvSat[1] < HSV_MAX)   hsvSat[1] += THRESHOLD_STEP;
                else                        hsvSat[1] = HSV_MAX;
                cooldowns.b.updateSnapshot(runtime);
            }

            if(gamepad1.x && cooldowns.x.ready(runtime)) {
                if(hsvSat[1] > hsvSat[0])  hsvSat[1] -= THRESHOLD_STEP;
                else                        hsvSat[1] = hsvSat[0];
                cooldowns.x.updateSnapshot(runtime);
            }




            // VAL MINIMUM
            if(gamepad1.left_trigger > TRIGGER_THRESHOLD && cooldowns.lt.ready(runtime)) {
                if (hsvVal[0] > HSV_MIN)   hsvVal[0] -= THRESHOLD_STEP;
                else                        hsvVal[0] = HSV_MIN;
                cooldowns.lt.updateSnapshot(runtime);
            }

            if(gamepad1.left_bumper && cooldowns.lb.ready(runtime)) {
                if(hsvVal[0] < hsvVal[1])  hsvVal[0] += THRESHOLD_STEP;
                else                        hsvVal[0] = hsvVal[1];
                cooldowns.lb.updateSnapshot(runtime);
            }



            // VAL MAXIMUM
            if(gamepad1.right_trigger > TRIGGER_THRESHOLD && cooldowns.rt.ready(runtime)) {
                if (hsvVal[1] > hsvVal[0])  hsvVal[1] -= THRESHOLD_STEP;
                else                        hsvVal[1] = hsvVal[0];
                cooldowns.rt.updateSnapshot(runtime);
            }

            if(gamepad1.right_bumper && cooldowns.rb.ready(runtime)) {
                if(hsvVal[1] < HSV_MAX)     hsvVal[1] += THRESHOLD_STEP;
                else                        hsvVal[1] = HSV_MAX;
                cooldowns.rb.updateSnapshot(runtime);
            }

            //-----------------------------------------------------------------------------------
            // END HSV THRESHOLD CONTROLS
            //-----------------------------------------------------------------------------------




            // SET HSV THRESHOLDS
            vision.setHsvHue(hsvHue);
            vision.setHsvSat(hsvSat);
            vision.setHsvVal(hsvVal);





        /* CONTROLS (Increase, Decrease):
            X ctr threshold: gp1.lStickButton, gp2.rStickButton
         */

            // X COUNTOUR THRESHOLD
            if(gamepad1.left_stick_button && cooldowns.lStickB.ready(runtime)) {
                if (ctrXThreshold > CTR_MIN_X)  ctrXThreshold -= THRESHOLD_STEP * 2.0;
                else                            ctrXThreshold = CTR_MIN_X;
                cooldowns.lStickB.updateSnapshot(runtime);
            }

            if(gamepad1.right_stick_button && cooldowns.rStickB.ready(runtime)) {
                if(ctrXThreshold < CTR_MAX_X)   ctrXThreshold += THRESHOLD_STEP * 2.0;
                else                            ctrXThreshold = CTR_MAX_X;
                cooldowns.rStickB.updateSnapshot(runtime);
            }

            vision.setCtrXTreshold(ctrXThreshold);



            // CONTOUR MIN AREA
            if(gamepad2.a && gp2_a.ready(runtime)) {
                if (ctrMinArea < 1000.0)    ctrMinArea += THRESHOLD_STEP * 2.0;
                else                        ctrMinArea = 1000.0;
                gp2_a.updateSnapshot(runtime);
            }

            if(gamepad2.b && gp2_b.ready(runtime)) {
                if (ctrMinArea > 0.0)   ctrMinArea -= THRESHOLD_STEP * 2.0;
                else                    ctrMinArea = 0.0;
                gp2_b.updateSnapshot(runtime);
            }

            vision.setFilterContoursMinArea(ctrMinArea);



            // Contour array
            List<MatOfPoint> contours = vision.filterContoursOutput();

            int contourHeightMid;
            int contourX;

            // Tally of contourPlacements for all visible contours this cycle
            // (Set all to 0 so they start over each cycle)
            int[] ctrTallies = {0, 0, 0};

            int[] ctrAreaTallies = {0, 0, 0};


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

                            if( contourX > ctrXThreshold / 2){ // Make sure the contour is below the cutoff

                                if (contourHeightMid > CTR_LEFT) {
                                    ctrAreaTallies[0] += (contours.get(i).height() * contours.get(i).width());
                                    ctrTallies[0]++;
                                } else if (contourHeightMid > CTR_RIGHT) {
                                    ctrAreaTallies[1] += (contours.get(i).height() * contours.get(i).width());
                                    ctrTallies[1]++;
                                } else {
                                    ctrAreaTallies[2] += (contours.get(i).height() * contours.get(i).width());
                                    ctrTallies[2]++;
                                }
                            }
                        }
                    }
                }
            } catch(Exception e) {
                e.printStackTrace();
                telemetry.addData("Exception", e.getMessage());
            }


            highest = highestTally(ctrTallies);
            highestArea = highestTally(ctrAreaTallies);

            if(highestArea == -1);
            else if(highestArea == ctrAreaTallies[0]) {
                goldPlacement = _GoldPlacement.LEFT;
            } else if(highestArea == ctrAreaTallies[1]) {
                goldPlacement = _GoldPlacement.CENTER;
            } else {
                goldPlacement = _GoldPlacement.RIGHT;
            }

//            RoverHardware.getDoubleSetting("xval", 100);
//
//            if(gamepad2.dpad_up) {
//                RoverHardware.setDoubleSetting("xval", ctrXThreshold);
//                ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("Settings.json"), RoverHardware.settings.toJSONString());
//            }



            telemetry.addData("Contour X threshold", ctrXThreshold);
            telemetry.addLine();
            telemetry.addData("Contour Min Area", ctrMinArea);
            telemetry.addLine();
            telemetry.addData("Left area tally", ctrAreaTallies[0]);
            telemetry.addData("Center area tally", ctrAreaTallies[1]);
            telemetry.addData("Right area tally", ctrAreaTallies[2]);
            telemetry.addLine();
            telemetry.addData("Highest", highestArea);
            telemetry.addLine();
            telemetry.addData("Gold Placement", goldPlacement);
            telemetry.update();
        }









        // start

        telemetry.addData("Gold Placement", goldPlacement);
        telemetry.update();

        hardware.latch.setPosition(LATCH_CLOSED);

        // Open lock
        hardware.setLockPosition(LOCK_OPEN);

        // Wait to drop
        sleep(3000);

        // Lock lock
        hardware.setLockPosition(LOCK_CLOSED);

        sleep(1500);

        // Deploy the wheels
        hardware.frontPivot.setTargetPosition(PIV_OMNI_FRONT);
        hardware.rearPivot.setTargetPosition(REAR_DEPLOY_TARGET);

        hardware.frontPivot.setPower(DEPLOY_SPEED);
        hardware.rearPivot.setPower(DEPLOY_SPEED);

        sleep(1500);

        // Open latch
        hardware.latch.setPosition(LATCH_OPEN);

        sleep(750);

        // Drive away from lander
        driveInches(DRIVE_DIST, DRIVE_SPEED);


        hardware.rearPivot.setTargetPosition(-1000);

        // Turn towards gold sample
        switch(goldPlacement) {
            case LEFT:
//                hardware.frontPivot.setTargetPosition(PIV_OMNI_FRONT);
//                hardware.rearPivot.setTargetPosition(PIV_STD_REAR);

                try {
                    turnToHeadingPID(SAMPLE_LEFT);
                } catch(InterruptedException e) {
                    telemetry.addLine("PID turn interrupted");
                    telemetry.update();

                    stop();
                }

                hardware.ramp.setPosition(RAMP_DOWN);

//                hardware.frontPivot.setTargetPosition(PIV_MINE_FRONT);
//                hardware.rearPivot.setTargetPosition(PIV_STD_REAR);

//                hardware.snorfler.setPower(-1.0);

                driveInches(SAMPLE_OUTSIDE_DIST, SAMPLE_SPEED);

                hardware.ramp.setPosition(RAMP_UP);


                driveInches(DEPOT_OUTSIDE_DIST, CRATER_SPEED);

                try {
                    turnToHeadingPID(-40);
                } catch (InterruptedException e) {
                    telemetry.addLine("PID turn interrupted");
                    telemetry.update();

                    stop();
                }

                driveInches(20.0, CRATER_SPEED);

                hardware.george.setPosition(GEORGE_DEPLOY);

                driveInches(-5.0, 0.8);


                break;

            case RIGHT:
//                hardware.frontPivot.setTargetPosition(PIV_OMNI_FRONT);
//                hardware.rearPivot.setTargetPosition(PIV_STD_REAR);

                try {
                    turnToHeadingPID(SAMPLE_RIGHT);
                } catch(InterruptedException e) {
                    telemetry.addLine("PID turn interrupted");
                    telemetry.update();

                    stop();
                }
                hardware.ramp.setPosition(RAMP_DOWN);


//                hardware.frontPivot.setTargetPosition(PIV_MINE_FRONT);
//                hardware.rearPivot.setTargetPosition(PIV_STD_REAR);

//                hardware.snorfler.setPower(-1.0);

                driveInches(SAMPLE_OUTSIDE_DIST, SAMPLE_SPEED);

                hardware.ramp.setPosition(RAMP_UP);

                driveInches(DEPOT_OUTSIDE_DIST + 5.0, CRATER_SPEED);

                try {
                    turnToHeadingPID(45);
                } catch (InterruptedException e) {
                    telemetry.addLine("PID turn interrupted");
                    telemetry.update();

                    stop();
                }

                driveInches(20.0, CRATER_SPEED);

                hardware.george.setPosition(GEORGE_DEPLOY);

                driveInches(-5.0, 0.8);

                break;

            default:    // Center and unknown are considered default
//                hardware.frontPivot.setTargetPosition(PIV_MINE_FRONT);
//                hardware.rearPivot.setTargetPosition(PIV_STD_REAR);

                hardware.ramp.setPosition(RAMP_DOWN);

//                hardware.snorfler.setPower(-1.0);

                driveInches(SAMPLE_CENTER_DIST, SAMPLE_SPEED);

                hardware.ramp.setPosition(RAMP_UP);

                driveInches(DEPOT_CENTER_DIST, CRATER_SPEED);


                try {
                    turnToHeadingPID(45);
                } catch (InterruptedException e) {
                    telemetry.addLine("PID turn interrupted");
                    telemetry.update();

                    stop();
                }


                hardware.george.setPosition(GEORGE_DEPLOY);



                try {
                    turnToHeadingPID(0);
                } catch (InterruptedException e) {
                    telemetry.addLine("PID turn interrupted");
                    telemetry.update();

                    stop();
                }


                driveInches(-12.0, 1.0);

                break;
        }
//
//        // Drive to crater
//        //hardware.driveInches(CRATER_OUTSIDE_DIST);

        while(opModeIsActive()) {
            telemetry.addData("Gold position", goldPlacement.toString());
            telemetry.addLine();
            telemetry.addData("Heading", heading());
            telemetry.update();
        }

        // stop
        vision.disable();
    }


    // highestTally() decides which of three values in an array is the largest.
    private int highestTally(int[] tallies) {
        int highest;

        if(tallies[0] == tallies[1] && tallies[0] == tallies[2]) return -1; // Default to previous goldPlacement if all are equal

        if(tallies[0] > tallies[1] && tallies[0] > tallies[2]) highest = tallies[0];
        else if(tallies[1] > tallies[2]) highest = tallies[1];
        else highest = tallies[2];

        return highest;
    }


    // driveInches() converts interests into encoder counts, which are passed onto driveCounts()
    public void driveInches(double inches, double speed) {
        driveCounts(
                (int) (inches * COUNTS_PER_INCH_DRIVE_FRONT),
                (int) (inches * COUNTS_PER_INCH_DRIVE_REAR),
                speed
        );
    }

    // driveCounts() sets encoder targets relative to their current positions and moves to the new target positions
    public void driveCounts(int frontTarget, int rearTarget, double speed) {
        hardware.frontLeft.setTargetPosition    (hardware.frontLeft.getCurrentPosition()    + (int)(frontTarget * GEAR_REDUCTION_DRIVE_FRONT));
        hardware.rearLeft.setTargetPosition     (hardware.rearLeft.getCurrentPosition()     + (int)(rearTarget * GEAR_REDUCTION_DRIVE_REAR));
        hardware.frontRight.setTargetPosition   (hardware.frontRight.getCurrentPosition()   + (int)(frontTarget * GEAR_REDUCTION_DRIVE_FRONT));
        hardware.rearRight.setTargetPosition    (hardware.rearRight.getCurrentPosition()    + (int)(rearTarget * GEAR_REDUCTION_DRIVE_REAR));

        hardware.frontLeft.setMode  (DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearLeft.setMode   (DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontRight.setMode (DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearRight.setMode  (DcMotor.RunMode.RUN_TO_POSITION);

        hardware.frontLeft.setPower (speed);
        hardware.rearLeft.setPower  (speed);
        hardware.frontRight.setPower(speed);
        hardware.rearRight.setPower (speed);

        while(  opModeIsActive() &&
                hardware.frontLeft   .isBusy() &&
                hardware.frontRight  .isBusy()) {
            telemetry.addData("rl encoder", hardware.rearLeft.getCurrentPosition());
            telemetry.addData("rr encoder", hardware.rearRight.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("rl target", hardware.rearLeft.getTargetPosition());
            telemetry.addData("rr target", hardware.rearRight.getTargetPosition());
            telemetry.update();
        }

        hardware.frontLeft  .setPower(0);
        hardware.rearLeft   .setPower(0);
        hardware.frontRight .setPower(0);
        hardware.rearRight  .setPower(0);

        hardware.frontLeft.setMode  (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.rearLeft.setMode   (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.frontRight.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.rearRight.setMode  (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }




    //----------------------------------------------------------------------------------------------
    // PID controller methods
    //----------------------------------------------------------------------------------------------

    // turnToHeadingPID() uses the PID controller in RoverHardware.java to turn to a given heading
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

            hardware.setLeftPower(-power);
            hardware.setRightPower(power);

            if (Math.abs(error) < hardware.TOLERANCE || gamepad2.dpad_down) {
                break;
            }

            Thread.sleep(1);

            telemetry.update();
        }

        hardware.setLeftPower(0);
        hardware.setRightPower(0);
    }

    // normalize180() corrects heading values in turnToHeadingPID()
    public double normalize180(double angle) {
        while(angle > 180) {
            angle -= 360;
        }
        while(angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    // heading() condenses the method for getting the robot's heading into one method
    public float heading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

}
