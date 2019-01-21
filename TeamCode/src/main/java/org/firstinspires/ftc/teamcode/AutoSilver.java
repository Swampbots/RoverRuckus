package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import static org.firstinspires.ftc.teamcode.RoverHardware.LATCH_RIGHT;


@Autonomous(name = "Silver", group = "Autonomous")
public class AutoSilver extends OpMode {

    RoverHardware hardware = new RoverHardware();

    GoldContourPipeline vision = new GoldContourPipeline();


    public final double RAMP_STOWED = 0.0;
    public final double LOCK_LOCKED = 0.0;



    // HSV Threshold input variables
    private final double THRESHOLD_STEP = 1.0;

    private final double HSV_MAX = 255.0;
    private final double HSV_MIN = 0.0;

    private double[] hsvHue = new double[]{106.0,118.0};
    private double[] hsvSat = new double[]{200.0,255.0};
    private double[] hsvVal = new double[]{113.0,255.0};


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

    public void init() {

        // Hardware and servos
        hardware.init(hardwareMap);

        hardware.latch.setPosition(LATCH_RIGHT);
        hardware.ramp.setPosition(RAMP_STOWED);
//        hardware.george.setPosition(GEORGE_STOWED);
        hardware.setLockPosition(LOCK_LOCKED);


        // Vision pipeline
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        vision.enable();
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

    }

    public void start() {

    }

    public void loop() {

    }



}
