package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_INCH_DRIVE_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.COUNTS_PER_INCH_DRIVE_REAR;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_DRIVE_FRONT;
import static org.firstinspires.ftc.teamcode.RoverHardware.GEAR_REDUCTION_DRIVE_REAR;

@TeleOp(name = "Test Drive Inches", group = "Testing")
public class TestDriveInches extends LinearOpMode {

    RoverHardware hardware = new RoverHardware();




    public void runOpMode() {
        hardware.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a)  driveInches(2.0, 0.8);
            if(gamepad1.b)  driveInches(4.0, 0.8);
            if(gamepad1.x)  driveInches(60.0, 0.8);
            if(gamepad1.y)  driveInches(-4.0, 0.8);

            telemetry.addLine("Encoders:");
            telemetry.addData("Front left", hardware.frontLeft.getCurrentPosition());
            telemetry.addData("Front right", hardware.frontRight.getCurrentPosition());
            telemetry.addData("Rear left", hardware.rearLeft.getCurrentPosition());
            telemetry.addData("Rear right", hardware.rearRight.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("CPI-Front", COUNTS_PER_INCH_DRIVE_FRONT);
            telemetry.addData("CPI-Rear", COUNTS_PER_INCH_DRIVE_REAR);
            telemetry.addLine();
            telemetry.addData("GRD-Front", GEAR_REDUCTION_DRIVE_FRONT);
            telemetry.addData("GRD-Rear", GEAR_REDUCTION_DRIVE_REAR);
            telemetry.addLine();

            telemetry.update();
        }
    }


    public void driveInches(double inches, double speed) {
        driveCounts(
                (int) (inches * COUNTS_PER_INCH_DRIVE_FRONT),
                (int) (inches * COUNTS_PER_INCH_DRIVE_REAR),
                speed
        );
    }

    }
}
