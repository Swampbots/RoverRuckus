package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
//@Disabled
@Autonomous(name = "Gold Contour Pipeline Test", group = "Testing")
public class GoldContourPipelineTest extends LinearOpMode {

    GoldContourPipeline vision;

    RoverHardware hardware = new RoverHardware();


    // HSV Threshold input variables
    private final double THRESHOLD_STEP = 1.0;

    private final double HSV_MAX = 255.0;
    private final double HSV_MIN = 0.0;

    private double[] hsvHue = new double[]{106.0,118.0};
    private double[] hsvSat = new double[]{200.0,255.0};
    private double[] hsvVal = new double[]{113.0,255.0};


    // Cooldown variables

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


//    // Min and max for average X and Y values of all seen contours
//    int contourYMin = -1; // Y is the height
//    int contourYMax = -1;
//
//    int contourXMin = -1; // X is the width
//    int contourXMax = -1;



    private final double CTR_MAX_Y = 192.0;
    private final double CTR_MAX_X = 144.0;

    private final double CTR_MIN_Y = 0.0;
    private final double CTR_MIN_X = 0.0;

    private final int CTR_LEFT  = (int) ((CTR_MAX_Y + CTR_MIN_Y) / 3.0);        // 1/3 of the width to bound the left third     [ |  ]
    private final int CTR_RIGHT = (int) ((CTR_MAX_Y + CTR_MIN_Y) * 2.0 / 3.0);  // 2/3 of the width to bound the center third   [  | ]




    // Variable for thresholding LT and RT inputs, e.g. if(gamepad1.left_trigger > TRIGGER_THRESHOLD)
    public final double TRIGGER_THRESHOLD = 0.7;


    public void runOpMode() {

        // OCV INITIALIZATION
        vision = new GoldContourPipeline();
        telemetry.addLine("GoldPipeline instance created.");
        telemetry.update();

        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        telemetry.addLine("Vision initialized.");
        telemetry.update();

        vision.enable();
        telemetry.addLine("Vision enabled.");
        telemetry.addLine("Press play to start.");
        telemetry.update();


        // HARDWARE INITIALIZATION
        hardware.init(hardwareMap);



        waitForStart();

        while(opModeIsActive()) {


            //--------------------------------------------------------------------------------------
            // START HSV THRESHOLD CONTROLS
            //--------------------------------------------------------------------------------------

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

            //--------------------------------------------------------------------------------------
            // END HSV THRESHOLD CONTROLS
            //--------------------------------------------------------------------------------------




            // SET HSV THRESHOLDS
            vision.setHsvHue(hsvHue);
            vision.setHsvSat(hsvSat);
            vision.setHsvVal(hsvVal);


            // Show contours on left_stick_button press
            vision.setShowCountours(gamepad1.left_stick_button);



            // Contour array
            List<MatOfPoint> contours = vision.findContoursOutput();

            int contourHeightMid;
            int contourWidthMid;
            ContourPlacement contourPlacement;

            // Tally of contourPlacements for all visible contours this cycle
            // (Set all to 0 so they start over each cycle)
            int leftTally = 0;
            int centerTally = 0;
            int rightTally = 0;

            int[] ctrTallies = {leftTally, centerTally, rightTally};


            // Reset contours on right_stick_button press
//            if(gamepad1.right_stick_button) resetContourBounds();


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
//            telemetry.addLine();
//            telemetry.addData("Contour max Y", contourYMax);
//            telemetry.addData("Contour min Y", contourYMin);
//            telemetry.addLine();
//            telemetry.addData("Contour max X", contourXMax);
//            telemetry.addData("Contour min X", contourXMin);
            telemetry.addLine();
            try {
                if(contours != null) {
                    if(contours.size() > 0) {
                        for(int i = 0; i < contours.size(); i++) {
                            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
                            contourHeightMid = (boundingRect.y + boundingRect.height) / 2;
                            contourWidthMid = (boundingRect.x + boundingRect.width) / 2;

//                            // Only assign a new value to a minimum if none have yet been assigned
//                            // or if the current contour has a smaller height.
//                            if(contourYMin == -1 || contourYMin > contourHeightMid) contourYMin = contourHeightMid; // Y, height
//                            if(contourXMin == -1 || contourXMin > contourWidthMid) contourXMin = contourWidthMid;   // X, width
//
//                            // Only assign a new value to a maximum if none have yet been assigned
//                            // or if the current contour has a larger height.
//                            if(contourYMax == -1 || contourYMax < contourHeightMid) contourYMax = contourHeightMid; // Y, height
//                            if(contourXMax == -1 || contourXMax < contourWidthMid) contourXMax = contourWidthMid;   // X, width

                            if(contourHeightMid < CTR_LEFT) {
                                contourPlacement = ContourPlacement.LEFT;
                                leftTally ++;
                            }
                            else if(contourHeightMid < CTR_RIGHT) {
                                contourPlacement = ContourPlacement.CENTER;
                                centerTally ++;
                            }
                            else {
                                contourPlacement = ContourPlacement.RIGHT;
                                rightTally ++;
                            }

//                            telemetry.addData("Contour" + Integer.toString(i),
//                                    String.format(Locale.getDefault(), "(%d, %s)", contourHeightMid, contourPlacement.toString()));
                        }
                    }
                }
            } catch(Exception e) {
                e.printStackTrace();
                telemetry.addData("Exception", e.getMessage());
            }
            telemetry.addData("Left tally", leftTally);
            telemetry.addData("Center tally", centerTally);
            telemetry.addData("Right tally", rightTally);
            telemetry.addLine();
            telemetry.addData("Most common", highestTally(ctrTallies));

            telemetry.update();
        }

        vision.disable();
    }

    private String highestTally(int[] tallies) {
        int highest;

        highest = (tallies[0] > tallies[1] ? tallies[0] : tallies[1]);
        highest = (highest > tallies[2] ? highest : tallies[2]);

        if(highest == tallies[0])       return "Left";
        else if(highest == tallies[1])  return "Center";
        else                            return "Right";
    }

//    private void resetContourBounds() {
//        contourYMin = -1;
//        contourYMax = -1;
//        contourXMin = -1;
//        contourXMax = -1;
//    }
}
