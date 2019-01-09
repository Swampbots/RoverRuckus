package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Autonomous(name = "Sample", group = "Testing")
public class AutoSample extends OpMode {

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


    private final double CTR_MAX_Y = 192.0;
    private final double CTR_MIN_Y = 0.0;

    private final int CTR_LEFT  = (int) ((CTR_MAX_Y + CTR_MIN_Y) / 3.0);        // 1/3 of the width to bound the left third     [ |  ]
    private final int CTR_RIGHT = (int) ((CTR_MAX_Y + CTR_MIN_Y) * 2.0 / 3.0);  // 2/3 of the width to bound the center third   [  | ]

    // Variable for thresholding LT and RT inputs, e.g. if(gamepad1.left_trigger > TRIGGER_THRESHOLD)
    public final double TRIGGER_THRESHOLD = 0.7;

    // Enum storing decision on gold placement
    GoldPlacement goldPlacement;






    public void init(){
        // OpenCV pipeline
        vision = new GoldContourPipeline();
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        vision.enable();

        // Hardware
        hardware.init(hardwareMap);

        telemetry.addLine("Hardware and vision enabled.");
        telemetry.update();
    }

    public void init_loop() {

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

        // Contours from last frame
        List<MatOfPoint> contours = vision.findContoursOutput();

        // Average height of a given contour
        int contourHeightMid;

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
                        contourHeightMid = (boundingRect.y + boundingRect.height) / 2;

                        if(contourHeightMid < CTR_LEFT)         leftTally ++;
                        else if(contourHeightMid < CTR_RIGHT)   centerTally ++;
                        else                                    rightTally ++;
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
        telemetry.addData("Most common", highestTally(leftTally, centerTally, rightTally));

        telemetry.update();
    }

    public void start() {
        vision.disable();
        telemetry.addLine("Vision disabled.");
        telemetry.update();
    }


    public void loop() {

    }




    private String highestTally(int left, int center, int right) {
        int highest;
        highest = (left > center ? left : center);
        highest = (highest > right ? highest : right);

        if(highest == left)         goldPlacement = GoldPlacement.LEFT;
        else if(highest == center)  goldPlacement = GoldPlacement.CENTER;
        else                        goldPlacement = GoldPlacement.RIGHT;

        switch(goldPlacement) {
            case LEFT:
                return "Left";
            case CENTER:
                return "Center";
            case RIGHT:
                return "Right";

            default:
                return "Right";
        }
    }
}