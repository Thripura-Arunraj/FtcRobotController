package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Bobert {

    private DcMotor RightForward, RightBack, LeftForward, LeftBack, Lifter;
    public final int Forward = 0, BACKWARD = 1, LEFT = 2, RIGHT = 3, UPRIGHT= 4, UPLEFT = 5, DOWNRIGHT = 6, DOWNLEFT = 7, LRIGHT = 8, RLEFT = 9, FBACKWARD = 10, FFORWARD = 11;
    private Servo Deposit;
    public static boolean LTURN = true, RTURN = false;

    static final double FEET_PER_METER = 3.28084;

    // PID/IMU Variables
    public BNO055IMU imu;
    double globalAngle, correction, rotation;
    public Orientation angles;
    public Orientation lastAngles = new Orientation();

    PIDController drive;
    PIDController veloDrive, veloStrafe, veloRdiagonal, veloLdiagonal;
    PIDController strafe;
    PIDController diagonal, rdiagonal;

    public ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    Telemetry telemetry;

    public Bobert(HardwareMap hardwareMap, Telemetry telemetry){

        this.telemetry = telemetry;
        initialize(hardwareMap);
    }

    private void initialize(HardwareMap hwMap){

        RightForward = hwMap.dcMotor.get("RightFront");
        RightBack = hwMap.dcMotor.get("RightBack");
        LeftForward = hwMap.dcMotor.get("LeftFront");
        LeftBack = hwMap.dcMotor.get("LeftBack");
        Lifter = hwMap.dcMotor.get("Lifter");

        Deposit = hwMap.get(Servo.class, "Deposit");

        RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Deposit.setDirection(Servo.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        /*IMU INIT */
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingTag          = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
    }

    public void initAprilTagsCamera(OpenCvCamera camera, HardwareMap hardwareMap){

        AprilTagDetectionPipeline aprilTagDetectionPipeline;

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        OpenCvCamera finalCamera = camera;

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                finalCamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    public int aprilTagDetection(AprilTagDetectionPipeline atdp){

        // 19, 0, 1
        int config1 = 19; // Tag ID 18 from the 36h11 family
        int config2 = 0;
        int config3 = 9;
        int configuration = 2;

        AprilTagDetection tagOfInterest = null;

        ArrayList<AprilTagDetection> currentDetections = atdp.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == config1 || tag.id == config2 || tag.id == config3)
                {
                    if(tag.id == config1)
                        configuration = 1;
                    if(tag.id == config2)
                        configuration = 2;
                    if(tag.id == config3)
                        configuration = 3;

                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
                telemetry.addData("configuration", configuration);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        }
        else
        {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        telemetry.update();
        sleep(20);

        return configuration;
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void imuTurn(boolean direction, double angle, double power){
        globalAngle = 0;

        if (direction){
            LeftForward.setPower(-power);
            LeftBack.setPower(-power);
            RightForward.setPower(power);
            RightBack.setPower(power);

            while (!isStopRequested() && Math.abs(getAngle()) <= angle){
                telemetry.addData("currentAngle", getAngle());
                telemetry.addData("currentEncoderPosition", LeftForward.getCurrentPosition());
                telemetry.addLine("Turning :)");
                telemetry.update();
            }

            stopMotors();

        } else {
            LeftForward.setPower(power);
            LeftBack.setPower(power);
            RightForward.setPower(-power);
            RightBack.setPower(-power);

            while (!isStopRequested() && getAngle() >= angle){
                telemetry.addData("currentAngle", getAngle());
                telemetry.addData("currentEncoderPosition", LeftForward.getCurrentPosition());
                telemetry.addLine("Turning :)");
                telemetry.update();
            }

            stopMotors();
        }
    }

    public void stopMotors(){
        LeftForward.setPower(0);
        LeftBack.setPower(0);
        RightForward.setPower(0);
        RightBack.setPower(0);
    }

    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -90)
            deltaAngle += 360;
        else if (deltaAngle > 90)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public final boolean isStopRequested() {
        return Thread.currentThread().isInterrupted();
    }
}

