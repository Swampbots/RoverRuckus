package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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







    public void init(){

    }

    public void init_loop() {

    }

    public void loop() {

    }
}
