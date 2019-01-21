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

    }

    public void start() {

    }

    public void loop() {

    }



}
