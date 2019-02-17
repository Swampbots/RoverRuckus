package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import static org.firstinspires.ftc.teamcode.RoverHardware.CTR_MAX_X;
import static org.firstinspires.ftc.teamcode.RoverHardware.CTR_MAX_Y;
import static org.firstinspires.ftc.teamcode.RoverHardware.CTR_MIN_X;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_CLOSED;
import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_OPEN;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.PIV_KNEEL_REAR;


@Autonomous(name = "Silver_CV", group = "Autonomous")
public class AutoSilver extends LinearOpMode {

    RoverHardware hardware = new RoverHardware();

    GoldContourPipeline vision = new GoldContourPipeline();

    _GoldPlacement goldPlacement = _GoldPlacement.CENTER;


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


    GamepadCooldowns cooldowns = new GamepadCooldowns();



    ButtonCooldown gp2_a    = new ButtonCooldown();
    ButtonCooldown gp2_b    = new ButtonCooldown();




    private final int CTR_RIGHT  = (int) ((CTR_MAX_Y + hardware.CTR_MIN_Y) / 3.0);        // 1/3 of the width to bound the left third     [ |  ]
    private final int CTR_LEFT = (int) ((CTR_MAX_Y + hardware.CTR_MIN_Y) * 2.0 / 3.0);  // 2/3 of the width to bound the center third   [  | ]

    private double ctrXThreshold = 206.0;
    private double ctrMinArea   = 0.0;


    int highest = 0;

    // Variable for thresholding LT and RT inputs, e.g. if(gamepad1.left_trigger > TRIGGER_THRESHOLD)
    public final double TRIGGER_THRESHOLD = 0.7;


    public void runOpMode() {
        // Init

        hardware.init(hardwareMap);


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
                                    ctrTallies[0]++;
                                } else if (contourHeightMid > CTR_RIGHT) {
                                    ctrTallies[1]++;
                                } else {
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

            if(highest == -1);
            else if(highest == ctrTallies[0]) {
                goldPlacement = _GoldPlacement.LEFT;
            } else if(highest == ctrTallies[1]) {
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
            telemetry.addData("Left tally", ctrTallies[0]);
            telemetry.addData("Center tally", ctrTallies[1]);
            telemetry.addData("Right tally", ctrTallies[2]);
            telemetry.addLine();
            telemetry.addData("Highest", highest);
            telemetry.addLine();
            telemetry.addData("Gold Placement", goldPlacement);
            telemetry.update();
        }









        // start

        telemetry.addLine("I'm going " + goldPlacement.toString() + "!");
        telemetry.update();

//        hardware.latch.setPosition(LATCH_CLOSED);
//
//        // Open lock
//        hardware.setLockPosition(LOCK_OPEN);
//
//        // Wait to drop
//        sleep(3000);
//
//        // Lock lock
//        hardware.setLockPosition(LOCK_LOCKED);
//
//        // Deploy the wheels
//        hardware.frontPivot.setTargetPosition(PIV_KNEEL_FRONT);
//        hardware.rearPivot.setTargetPosition(PIV_KNEEL_REAR);
//
//        // Open latch
//        hardware.latch.setPosition(LATCH_OPEN);

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

        while(opModeIsActive()) {
            telemetry.addData("Runtime", getRuntime());
            telemetry.addLine();
            telemetry.addData("Gold position", goldPlacement.toString());
            telemetry.update();
        }

        // stop
        vision.disable();
    }



    private int highestTally(int[] tallies) {
        int highest;

        if(tallies[0] == tallies[1] && tallies[0] == tallies[2]) return -1; // Default to previous goldPlacement if all are equal

        if(tallies[0] > tallies[1] && tallies[0] > tallies[2]) highest = tallies[0];
        else if(tallies[1] > tallies[2]) highest = tallies[1];
        else highest = tallies[2];

        return highest;
    }

}
