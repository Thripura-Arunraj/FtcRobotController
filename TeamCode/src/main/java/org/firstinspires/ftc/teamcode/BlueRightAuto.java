package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Bobert.LTURN;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class BlueRightAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private DcMotor RightForward, RightBack, LeftForward, Intake, LeftBack, Lifter;
    public final int Forward = 0, BACKWARD = 1, LEFT = 2, RIGHT = 3, UPRIGHT = 4, UPLEFT = 5, DOWNRIGHT = 6, DOWNLEFT = 7, LRIGHT = 8, RLEFT = 9, FBACKWARD = 10, FFORWARD = 11;
    private Servo Deposit;
    Caruso c = new Caruso();

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // 19, 0, 1
    int config1 = 19; // Tag ID 18 from the 36h11 family
    int config2 = 0;
    int config3 = 9;
    int configuration = 2;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {


        Bobert b = new Bobert(hardwareMap, telemetry);


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {


            /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
            if (opModeIsActive()) {
                b.imuTurn(b.LTURN, 90, 0.3);

                sleep(5000);
            }
        }


    }
}