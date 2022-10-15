package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Caruso{

    public DcMotorEx LeftForward, RightForward, LeftBack, RightBack, Arm, Intake, Carousel;
    public DcMotorEx Turber;
    public DistanceSensor LeftDistance, RightDistance, FrontDistance, eled;
    public TouchSensor lLimit, rlimit, armTouch, armRLimit, armLLimit;
    private Servo dep;

    //UTILITY VARIABLES
    public final int Forward = 0, BACKWARD = 1, LEFT = 2, RIGHT = 3, UPRIGHT= 4, UPLEFT = 5, DOWNRIGHT = 6, DOWNLEFT = 7, LRIGHT = 8, RLEFT = 9, FBACKWARD = 10, FFORWARD = 11;
    public final boolean clock = true, cclock = false;
    public final double ARM_SLOPE = 0.85 / 1460;
    public static final double LEVEL3 = 0.98, LEVEL2 = 0.87, LEVEL1 = 0.8;

    public static boolean AT_HUB = false, DROPPED = false, AT_WALL = false, COLLECTED = false, READYFORARM = false, MOVEDONE = false, ARMDONE = false, NEXTELEMENT = false, DONE = false, TSE_DETECTED = false;

    public  static final int NEAR_HUB = 1;
    public  static final int NEAR_PARK = 2;
    public  static final int LEAVE_AS_IS = 3;
    public static int Position = LEAVE_AS_IS;

    //CAMERA VARIABLES
    public static final int CAMERA_HEIGHT = 480;
    public static final int CAMERA_WIDTH = 640;
    static public int RED_CAROUSAL = 1;
    static public int BLUE_CAROUSAL = 2;
    static public int BLUE_PICKUP = 3;
    static public int RED_PICKUP = 4;
    public static final int THRESH = 170;
    public static final int MAX = 200;

    private static int valLeft = -1;
    private static int valRight = -1;


    public static int thresh = 1;
    public static int bNw =  2;
    public static int reg = 3;

    public static boolean failSafe = true;

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

    public Caruso(){}

    public Caruso(HardwareMap hardwareMap, Telemetry telemetry){

        this.telemetry = telemetry;
        initialize(hardwareMap);
    }

    public void falsify(){
        AT_HUB = false;
        DROPPED = false;
        AT_WALL = false;
        COLLECTED = false;
        READYFORARM = false;
        MOVEDONE = false;
        ARMDONE = false;
        TSE_DETECTED = false;
        NEXTELEMENT = false;
        DONE = false;
    }

    private void initialize(HardwareMap hwMap){

        /* DRIVE MOTORS */
        this.LeftForward = hwMap.get(DcMotorEx.class, "LeftForward");
        this.RightForward = hwMap.get(DcMotorEx.class, "RightForward");
        this.LeftBack = hwMap.get(DcMotorEx.class, "LeftBack");
        this.RightBack = hwMap.get(DcMotorEx.class, "RightBack");

        //SERVO
        this.dep = hwMap.get(Servo.class, "Deposit");

        /* Attachments */
        this.Carousel = hwMap.get(DcMotorEx.class,"Carousel");
        this.Turber = hwMap.get(DcMotorEx.class,"Turret");
        this.Intake = hwMap.get(DcMotorEx.class,"Intake");
        this.Arm = hwMap.get(DcMotorEx.class, "Arm");

        /*Sensors*/
        this.LeftDistance = hwMap.get(DistanceSensor.class, "LeftDistance");
        this.RightDistance = hwMap.get(DistanceSensor.class, "RightDistance");
        this.FrontDistance = hwMap.get(DistanceSensor.class, "FrontDistance");
        this.lLimit = hwMap.get(TouchSensor.class, "llimit");
        this.rlimit = hwMap.get(TouchSensor.class, "rlimit");
        this.armTouch = hwMap.get(TouchSensor.class, "armTouch");
        this.eled = hwMap.get(DistanceSensor.class, "ElementDistance");
        this.armRLimit = hwMap.get(TouchSensor.class, "armRLimit");
        this.armLLimit = hwMap.get(TouchSensor.class, "armLLimit");


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

        drive = new PIDController(0.0016367 * 2, 0.00016367 * 5, 0.00016367 * 5);
        drive.setSetpoint(0);
        drive.setOutputRange(0, 0.5);
        drive.setInputRange(-90, 90);
        drive.enable();

        veloDrive = new PIDController(0.5, 0.05, 0.005);
        veloDrive.setSetpoint(0);
        veloDrive.setOutputRange(0, 1305);
        veloDrive.setInputRange(-90, 90);
        veloDrive.enable();

        veloStrafe = new PIDController(0.016367 * 0.8, 0.00016367 * 8, 0.00016367 * 6);
        veloStrafe.setSetpoint(0);
        veloStrafe.setOutputRange(0, 1305);
        veloStrafe.setInputRange(-90, 90);
        veloStrafe.enable();

        strafe = new PIDController(0.0016367 * 2, 0.00016367 * 5, 0.00016367 * 4);
        strafe.setSetpoint(0);
        strafe.setOutputRange(0, 0.5);
        strafe.setInputRange(-90, 90);
        strafe.enable();

        diagonal = new PIDController(0.016367, 0.0016367, 0.00016367);
        diagonal.setSetpoint(0);
        diagonal.setOutputRange(0, 0.5);
        diagonal.setInputRange(-90, 90);
        diagonal.enable();

        rdiagonal = new PIDController(0.0016367 * 2.5, 0.00016367 * 2, 0.00016367 * 2);
        rdiagonal.setSetpoint(0);
        rdiagonal.setOutputRange(0, 0.5);
        rdiagonal.setInputRange(-90, 90);
        rdiagonal.enable();

        veloRdiagonal = new PIDController(0.5, 0.05, 0.005);
        veloRdiagonal.setSetpoint(0);
        veloRdiagonal.setOutputRange(0, 1305);
        veloRdiagonal.setInputRange(-90, 90);
        veloRdiagonal.enable();


        LeftForward.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);

        Arm.setDirection(DcMotorSimple.Direction.REVERSE);

        RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Turber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Turber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // Carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Turber.setTargetPositionTolerance(22);
        Arm.setTargetPositionTolerance(20);

        telemetry.addLine("DONE");
        telemetry.update();
    }

    public void initPhoneCamera(OpenCvCamera camera, HardwareMap hardwareMap, OpenCvPipeline pipeline) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.setPipeline(pipeline);

        OpenCvCamera finalCamera = camera;
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

    }

    public void initCamera(OpenCvCamera camera, HardwareMap hardwareMap, OpenCvPipeline pipeline, int width, int height, int cameraNum)    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam " + cameraNum),
                cameraMonitorViewId);
        camera.setPipeline(pipeline);
        OpenCvCamera finalCamera = camera;
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
                if(cameraNum == 2)
                    finalCamera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                if(cameraNum == 1)
                    finalCamera.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }

    public void initCamera(OpenCvCamera camera1,OpenCvCamera camera2, HardwareMap hardwareMap, OpenCvPipeline pipeline1,OpenCvPipeline pipeline2, int width, int height)    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                viewportContainerIds[0]);
        camera2 = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 2"),
                viewportContainerIds[1]);

        OpenCvCamera finalCamera1 = camera1;
        OpenCvCamera finalCamera2 = camera2;
        camera1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
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
                finalCamera1.setPipeline(pipeline1);
                finalCamera1.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        camera2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
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
                finalCamera2.setPipeline(pipeline2);
                finalCamera2.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }

    public void rotateCarousel (int Direction, double power) {

        startMotors(0.24);
        runtime.reset();
        if (Direction == BLUE_CAROUSAL) {
            while (LeftBack.getVelocity() > 1000 || runtime.seconds() < 2) {
                telemetry.addData("leftback velocity", LeftBack.getVelocity());
            }
            stopMotors();
            Carousel.setPower(power);
            sleep(3000);
            Carousel.setPower(0);

        } else if (Direction == RED_CAROUSAL) {
            while (LeftBack.getVelocity() > 1000 || runtime.seconds() < 2) {
                telemetry.addData("leftback velocity", LeftBack.getVelocity());
            }
            stopMotors();
            Carousel.setPower(-power);
            sleep(3000);
            Carousel.setPower(0);
        }
    }


    public void initTelem(){
        telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("FrontDistance", FrontDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("ElementDistance", eled.getDistance(DistanceUnit.MM));
        telemetry.addData("getAngle", getAngle());
        telemetry.addData("correction", veloDrive.performPID(getAngle()));
        telemetry.addData("Limit", lLimit.isPressed());
        telemetry.addData("armLLimit", armLLimit.isPressed());
        telemetry.addData("armRLimit", armRLimit.isPressed());
    }

    public void initContourTelem(OpenCvCamera cam){
        telemetry.addLine("\nCAMERA");
        //telemetry.addData("xPos")
    }

    public void stopMotors(){
        LeftForward.setPower(0);
        LeftBack.setPower(0);
        RightForward.setPower(0);
        RightBack.setPower(0);
    }

    public void stopVelocity(){
        LeftForward.setVelocity(0);
        LeftBack.setVelocity(0);
        RightBack.setVelocity(0);
        RightForward.setVelocity(0);
    }

    public void startMotors(double Power){
        LeftForward.setPower(Power);
        LeftBack.setPower(Power);
        RightForward.setPower(Power);
        RightBack.setPower(Power);
    }

    public void startMotors(double Power, double Power1, boolean rand){
        LeftForward.setPower(Power);
        LeftBack.setPower(Power);
        RightForward.setPower(Power1);
        RightBack.setPower(Power1);
    }

    public void startMotors(double Power, double angle){
        correction = drive.performPID(getAngle() - angle);
        LeftForward.setPower(Power - correction);
        LeftBack.setPower(Power - correction);
        RightForward.setPower(Power + correction);
        RightBack.setPower(Power + correction);
    }

    public void startMotors(int direction, double Power, double angle, boolean condition){
        runtime.reset();

        if(direction == Forward) {
            while (!TSE_DETECTED) {
                correction = drive.performPID(getAngle() - angle);
                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);
            }
        }else if(direction == BACKWARD){
            while (!TSE_DETECTED) {
                correction = drive.performPID(getAngle() - angle);
                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(-Power + correction);
            }
        }
        stopMotors();
    }

    public void startMotors(int direction, double Power){


        if(direction == Forward){
            LeftForward.setPower(Power);
            LeftBack.setPower(Power);
            RightForward.setPower(Power);
            RightBack.setPower(Power);
        } else if(direction == BACKWARD){
            LeftForward.setPower(-Power);
            LeftBack.setPower(-Power);
            RightForward.setPower(-Power);
            RightBack.setPower(-Power);
        } else if(direction == RIGHT){
            LeftForward.setPower(Power);
            LeftBack.setPower(-Power);
            RightForward.setPower(-Power);
            RightBack.setPower(Power);
        }else if(direction == LEFT){
            LeftForward.setPower(-Power);
            LeftBack.setPower(Power);
            RightForward.setPower(Power);
            RightBack.setPower(-Power);
        }

    }

    public void startMotors(int direction, double Power, int time){

        if(direction == Forward){
            LeftForward.setPower(Power);
            LeftBack.setPower(Power);
            RightForward.setPower(Power);
            RightBack.setPower(Power);
        } else if(direction == BACKWARD){
            LeftForward.setPower(-Power);
            LeftBack.setPower(-Power);
            RightForward.setPower(-Power);
            RightBack.setPower(-Power);
        }
        else if(direction == LEFT){
            LeftForward.setPower(-Power);
            LeftBack.setPower(Power);
            RightForward.setPower(Power);
            RightBack.setPower(-Power);
        }
        else if(direction == RIGHT){
            LeftForward.setPower(Power);
            LeftBack.setPower(-Power);
            RightForward.setPower(-Power);
            RightBack.setPower(Power);
        }

        sleep(time);

        stopMotors();
    }

    public void moveToCarousal(double Velocity){

        runtime.reset();

        int collisionTime = 0;

        LeftForward.setVelocity(Velocity, AngleUnit.DEGREES);
        LeftBack.setVelocity(Velocity, AngleUnit.DEGREES);
        RightForward.setVelocity(Velocity, AngleUnit.DEGREES);
        RightBack.setVelocity(Velocity, AngleUnit.DEGREES);

        sleep(500);

        while(LeftForward.getVelocity(AngleUnit.DEGREES) > 75  && !isStopRequested()){

            correction = veloDrive.performPID(getAngle());

            LeftForward.setVelocity(Velocity + correction, AngleUnit.DEGREES);
            LeftBack.setVelocity(Velocity + correction, AngleUnit.DEGREES);
            RightForward.setVelocity(Velocity - correction, AngleUnit.DEGREES);
            RightBack.setVelocity(Velocity - correction, AngleUnit.DEGREES);

            if(LeftForward.getVelocity(AngleUnit.DEGREES) > 87){
                collisionTime += 1;
                sleep(10);
            }

            telemetry.addData("Vroom", RightForward.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("correction", correction);
            telemetry.addData("angle", getAngle());
            telemetry.addData("max output", veloDrive.getMaximumOutput());
            telemetry.addData("collisiionTime", collisionTime);
            telemetry.update();
        }

        stopVelocity();

        startVelocity(8);

        sleep(100);

        stopVelocity();

    }

    public boolean isPressed(){
        return lLimit.isPressed();
    }

    public double getDistance(){
        return LeftDistance.getDistance(DistanceUnit.INCH);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public final boolean isStopRequested() {
        return Thread.currentThread().isInterrupted();
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

    public double getYAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);



        double deltaAngle = angles.secondAngle - lastAngles.secondAngle;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void runIntake(){
        Intake.setPower(1);
    }

    public void stopIntake(){
        Intake.setPower(0);
    }

    public double ElementDistance(){ return eled.getDistance(DistanceUnit.MM); }

    private void encodTelem(){
        telemetry.addLine("going");
        telemetry.addData("correction", correction);
       // telemetry.addData("TargetPosition", LeftForward.getTargetPosition());
        //telemetry.addData("Current Position", LeftForward.getCurrentPosition());
        telemetry.addData("currentAngle", getAngle());
        telemetry.update();
    }

    public void moveEncoders(int Direction, double Power, int TargetPosition, double desiredAngle) {
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runtime.reset();



        if (Direction == Forward) {
            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = drive.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);
                encodTelem();
            }

        } else if (Direction == BACKWARD) {
            LeftBack.setTargetPosition(-TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = drive.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(-Power + correction);

                encodTelem();
            }

        } else if (Direction == LEFT) {
            LeftForward.setTargetPosition(TargetPosition);

            LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs( LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

                encodTelem();
            }

        } else if (Direction == RIGHT) {
            LeftForward.setTargetPosition(-TargetPosition);

            LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftForward.getCurrentPosition()) <= Math.abs(LeftForward.getTargetPosition())) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

                encodTelem();
            }

        } else if (Direction == UPRIGHT) {
            LeftForward.setTargetPosition(TargetPosition);

            LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {
                correction = diagonal.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                RightBack.setPower(Power + correction);

                encodTelem();
            }

        } else if(Direction == UPLEFT){
            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {
                correction = diagonal.performPID(getAngle() - desiredAngle);

                RightForward.setPower(Power - correction);
                LeftBack.setPower(Power + correction);

                encodTelem();
            }
        } else if(Direction == DOWNRIGHT){
            LeftBack.setTargetPosition(-TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {
                correction = diagonal.performPID(getAngle() - desiredAngle);

                RightForward.setPower(-Power + correction);
                LeftBack.setPower(-Power - correction);

                encodTelem();
            }
        } else if(Direction == DOWNLEFT){
            LeftBack.setTargetPosition(-TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {
                correction = diagonal.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power + correction);
                RightBack.setPower(-Power - correction);

                encodTelem();
            }
        }

        stopMotors();
    }

    public void veloMoveEncoders(int Direction, double Power, int TargetPosition, double desiredAngle) {
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runtime.reset();



        if (Direction == Forward) {
            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = drive.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);
                telemetry.addData("velocity", LeftBack.getVelocity());
                encodTelem();
            }

        } else if (Direction == BACKWARD) {
            LeftBack.setTargetPosition(-TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = veloDrive.performPID(getAngle() - desiredAngle);

                LeftForward.setVelocity(-Power + correction);
                LeftBack.setVelocity(-Power + correction);
                RightForward.setVelocity(-Power - correction);
                RightBack.setVelocity(-Power - correction);

                encodTelem();
            }

        } else if (Direction == LEFT) {
            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

                encodTelem();
            }

        } else if (Direction == RIGHT) {
            LeftBack.setTargetPosition(-TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

                encodTelem();
            }

        } else if (Direction == UPRIGHT) {
            LeftForward.setTargetPosition(TargetPosition);

            LeftForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {
                correction = diagonal.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                RightBack.setPower(Power + correction);

                encodTelem();
            }

        } else if(Direction == UPLEFT){
            LeftBack.setTargetPosition(TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {
                correction = diagonal.performPID(getAngle() - desiredAngle);

                RightForward.setPower(Power - correction);
                LeftBack.setPower(Power + correction);

                encodTelem();
            }
        } else if(Direction == DOWNRIGHT){
            LeftBack.setTargetPosition(-TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {
                correction = diagonal.performPID(getAngle() - desiredAngle);

                RightForward.setPower(-Power + correction);
                LeftBack.setPower(-Power - correction);

                encodTelem();
            }
        } else if(Direction == DOWNLEFT){
            LeftBack.setTargetPosition(-TargetPosition);

            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(LeftBack.getCurrentPosition()) <= Math.abs(LeftBack.getTargetPosition())) {
                correction = diagonal.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power + correction);
                RightBack.setPower(-Power - correction);

                encodTelem();
            }
        }

        stopMotors();
    }


    public void moveDistance(int Direction, double Power, double Distance, double desiredAngle, double failSafeTime) {

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runtime.reset();

        if (Direction == LEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (!isStopRequested() && LeftDistance.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        } else if (Direction == RIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && RightDistance.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        } else if (Direction == UPRIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && RightDistance.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power + correction);
                RightBack.setPower(Power - correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        }else if (Direction == DOWNRIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && LeftDistance.getDistance(DistanceUnit.INCH) < Distance &&  runtime.seconds() < failSafeTime) {

                correction = diagonal.performPID(getAngle() - desiredAngle);

                RightForward.setPower(-Power + correction);
                LeftBack.setPower(-Power - correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("time", runtime.seconds());
                telemetry.update();

            }

        }else if (Direction == UPLEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && LeftDistance.getDistance(DistanceUnit.INCH) > Distance &&  runtime.seconds() < failSafeTime) {

                correction = diagonal.performPID(getAngle() - desiredAngle);

                RightForward.setPower(Power + correction);
                LeftBack.setPower(Power - correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("time", runtime.seconds());
                telemetry.update();

            }

        }else if (Direction == DOWNLEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && RightDistance.getDistance(DistanceUnit.INCH) < Distance &&  runtime.seconds() < failSafeTime) {

                correction = rdiagonal.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                RightBack.setPower(-Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("time", runtime.seconds());
                telemetry.update();

            }

        }

        stopMotors();
    }

    public void startVelocity(int VelocityInDegreesPerSecond){
        LeftForward.setVelocity(VelocityInDegreesPerSecond, AngleUnit.DEGREES);
        LeftBack.setVelocity(VelocityInDegreesPerSecond, AngleUnit.DEGREES);
        RightBack.setVelocity(VelocityInDegreesPerSecond, AngleUnit.DEGREES);
        RightForward.setVelocity(VelocityInDegreesPerSecond, AngleUnit.DEGREES);
    }

    public void veloMoveDistance(int Direction, double Velocity, double Distance, double desiredAngle, double failSafeTime, DistanceSensor sensor) {

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runtime.reset();

        if (Direction == LEFT) {

            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = veloStrafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Velocity - correction);
                LeftBack.setPower(Velocity - correction);
                RightForward.setPower(Velocity + correction);
                RightBack.setPower(-Velocity + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", sensor.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        } else if (Direction == RIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = veloStrafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Velocity - correction);
                LeftBack.setPower(-Velocity - correction);
                RightForward.setPower(-Velocity + correction);
                RightBack.setPower(Velocity + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", sensor.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        } else if (Direction == UPRIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setVelocity(Velocity + correction);
                RightBack.setVelocity(Velocity - correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        }else if (Direction == DOWNRIGHT) {


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) < Distance &&  runtime.seconds() < failSafeTime) {

                correction = veloRdiagonal.performPID(getAngle() - desiredAngle);

                RightForward.setVelocity(-Velocity + correction, AngleUnit.DEGREES);
                LeftBack.setVelocity(-Velocity - correction, AngleUnit.DEGREES);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", sensor.getDistance(DistanceUnit.INCH));
                telemetry.addData("time", runtime.seconds());
                telemetry.update();

            }

        }else if (Direction == UPLEFT) {

            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) > Distance &&  runtime.seconds() < failSafeTime) {

                correction = veloRdiagonal.performPID(getAngle() - desiredAngle);

                RightForward.setVelocity(Velocity + correction, AngleUnit.DEGREES);
                LeftBack.setVelocity(Velocity - correction, AngleUnit.DEGREES);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", sensor.getDistance(DistanceUnit.INCH));
                telemetry.addData("time", runtime.seconds());
                telemetry.update();

            }

        }else if (Direction == DOWNLEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && RightDistance.getDistance(DistanceUnit.INCH) < Distance &&  runtime.seconds() < failSafeTime) {

                correction = rdiagonal.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Velocity - correction);
                RightBack.setPower(-Velocity + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("time", runtime.seconds());
                telemetry.update();

            }

        } else if (Direction == LRIGHT) {

            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) < Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = veloRdiagonal.performPID(getAngle() - desiredAngle);

                LeftForward.setVelocity(Velocity - correction, AngleUnit.DEGREES);
                LeftBack.setVelocity(-Velocity - correction, AngleUnit.DEGREES);
                RightForward.setVelocity(-Velocity + correction, AngleUnit.DEGREES);
                RightBack.setVelocity(Velocity + correction, AngleUnit.DEGREES);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        }
        else if (Direction == RLEFT) {

            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) < Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = veloStrafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Velocity - correction);
                LeftBack.setPower(Velocity - correction);
                RightForward.setPower(Velocity + correction);
                RightBack.setPower(-Velocity + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        }

        stopVelocity();
    }
    public void moveDistance(int Direction, double Power, double Distance, double desiredAngle, double failSafeTime, DistanceSensor sensor) {

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runtime.reset();

        if (Direction == LEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        } else if (Direction == RIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        } else if (Direction == LRIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) < Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        } else if (Direction == RLEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) < Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = diagonal.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        }
        else if (Direction == FBACKWARD) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) < Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = drive.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(-Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        }
        else if (Direction  == FFORWARD) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = drive.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("frontdistance", FrontDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        } else if (Direction == UPRIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = diagonal.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(Power + correction);
                RightBack.setPower(Power - correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        }else if (Direction == DOWNRIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) < Distance &&  runtime.seconds() < failSafeTime) {

                correction = diagonal.performPID(getAngle() - desiredAngle);

                RightForward.setPower(-Power + correction);
                LeftBack.setPower(-Power - correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("time", runtime.seconds());
                telemetry.update();

            }

        }else if (Direction == UPLEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) > Distance &&  runtime.seconds() < failSafeTime) {

                correction = diagonal.performPID(getAngle() - desiredAngle);

                RightForward.setPower(Power + correction);
                LeftBack.setPower(Power - correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("time", runtime.seconds());
                telemetry.update();

            }

        }else if (Direction == DOWNLEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && sensor.getDistance(DistanceUnit.INCH) > Distance &&  runtime.seconds() < failSafeTime) {

                correction = diagonal.performPID(getAngle() - desiredAngle);

                LeftForward.setPower(-Power - correction);
                RightBack.setPower(-Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("time", runtime.seconds());
                telemetry.update();

            }

        }

        stopMotors();
    }

    private boolean activate(boolean condition, int delay){
        if(runtime.milliseconds() > delay)
            return true;
        return false;
    }

    public void moveDistance(int Direction, double Power, double Distance, double desiredAngle, double failSafeTime, boolean condition, int activationTime) {

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runtime.reset();

        if (Direction == LEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (!isStopRequested() && LeftDistance.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                condition = activate(condition, activationTime);

                LeftForward.setPower(-Power - correction);
                LeftBack.setPower(Power - correction);
                RightForward.setPower(Power + correction);
                RightBack.setPower(-Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        } else if (Direction == RIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && RightDistance.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                condition = activate(condition, activationTime);

                LeftForward.setPower(Power - correction);
                LeftBack.setPower(-Power - correction);
                RightForward.setPower(-Power + correction);
                RightBack.setPower(Power + correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        } else if (Direction == UPRIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && RightDistance.getDistance(DistanceUnit.INCH) > Distance && (failSafeTime > 0 && runtime.seconds() < failSafeTime)) {

                correction = strafe.performPID(getAngle() - desiredAngle);

                condition = activate(condition, activationTime);

                LeftBack.setPower(Power + correction);
                RightForward.setPower(Power - correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("rightdistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

            }

        }else if (Direction == DOWNRIGHT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && LeftDistance.getDistance(DistanceUnit.INCH) < Distance &&  runtime.seconds() < failSafeTime) {

                correction = diagonal.performPID(getAngle() - desiredAngle);

                condition = activate(condition, activationTime);

                RightForward.setPower(-Power + correction);
                LeftBack.setPower(-Power - correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("time", runtime.seconds());
                telemetry.update();

            }

        }else if (Direction == UPLEFT) {

            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (!isStopRequested() && LeftDistance.getDistance(DistanceUnit.INCH) > Distance &&  runtime.seconds() < failSafeTime) {

                correction = diagonal.performPID(getAngle() - desiredAngle);

                condition = activate(condition, activationTime);

                RightForward.setPower(Power + correction);
                LeftBack.setPower(Power - correction);

                telemetry.addLine("going");
                telemetry.addData("correction", correction);
                telemetry.addData("leftback position", LeftBack.getCurrentPosition());
                telemetry.addData("leftdistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("time", runtime.seconds());
                telemetry.update();

            }

        }

        stopMotors();
    }

    /**
     * method only moves arm up
     * @param encoders
     * @param power
     */

    /** SERVO POSITIONS
     * 0.95 Collect
     * 0.55 Travel
     * 0.1 Drop
     */

    public void moveArm(int encoders, double Velocity, int servoTarget){
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();

        Arm.setVelocity(-Velocity, AngleUnit.DEGREES);

        //while(Math.abs(Arm.getVelocity(AngleUnit.DEGREES)) < 60){}

        while(armTouch.isPressed()){}

        Arm.setVelocity(0);

        Arm.setTargetPosition(-encoders);

        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Arm.setVelocity(-Velocity, AngleUnit.DEGREES);

        while(!isStopRequested() && Math.abs(Arm.getCurrentPosition()) <= Math.abs(Arm.getTargetPosition())){

            if(Math.abs(Arm.getCurrentPosition()) > servoTarget)
                depoHold();

            telemetry.addLine(">>>>GOING UP<<<<<<<");
            telemetry.addData("Current Position", Arm.getCurrentPosition());
            telemetry.addData("Target Position", Arm.getTargetPosition());
            telemetry.update();
        }

        Arm.setVelocity(0);

    }

    public void armDown(){
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setPower(1);

        while(!armTouch.isPressed()){
            telemetry.addLine(">>GOING DOWN<<");
            telemetry.addData("armTouch", armTouch.isPressed());
            telemetry.update();
        }

        Arm.setPower(0);
    }


    /**
     * negative encoders means counterclockwise
     * @param encoders
     * @param power
     */
    public void moveTurret(boolean direction, int encoders, double power){
        Turber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(direction)
            Turber.setTargetPosition(-encoders);
        else
            Turber.setTargetPosition(encoders);

        Turber.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Turber.setPower(power);

        while(!isStopRequested() && Math.abs(Turber.getCurrentPosition()) <= Math.abs(Turber.getTargetPosition()) && failSafe){
            telemetry.addLine(">>>>TURNING<<<<<<<");
            telemetry.addData("Current Position", Turber.getCurrentPosition());
            telemetry.addData("Target Position", Turber.getTargetPosition());
            telemetry.update();
        }

        Turber.setPower(0);

        Turber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turretPower(double Power){Turber.setPower(Power);}

    public void moveTurret(boolean direction, double power) {
        runtime.reset();

        int temp = 0;

        if (direction) {
            Turber.setVelocity(-100, AngleUnit.DEGREES);

            while(lLimit.isPressed()){}

            while (!lLimit.isPressed()){

            }

            Turber.setVelocity(0);

        }
        else {
            Turber.setVelocity(100, AngleUnit.DEGREES);

            while(lLimit.isPressed()) {

            }

            while(temp != 2){
                if(lLimit.isPressed()){
                    if(temp == 1){
                        while(lLimit.isPressed()){}
                    }

                    temp++;
                }
            }

            Turber.setVelocity(0);
        }

        telemetry.addData("Turning", direction);
        telemetry.addData("limit", isPressed());
        telemetry.addData("time", runtime.seconds());
        telemetry.update();


        Turber.setPower(0);
    }

    public void centerTurret(boolean direction){
        failSafe = true;

        if(direction) {
            Turber.setVelocity(-105, AngleUnit.DEGREES);

            while(!lLimit.isPressed() && failSafe){}

            Turber.setVelocity(-23, AngleUnit.DEGREES);
        }
        else {
            Turber.setVelocity(105, AngleUnit.DEGREES);

            while(!rlimit.isPressed() && failSafe){}

            Turber.setVelocity(23, AngleUnit.DEGREES);
        }

        while(!(rlimit.isPressed() && lLimit.isPressed()) && failSafe){
            telemetry.addData("rlimit", rlimit.isPressed());
            telemetry.addData("llimit", lLimit.isPressed());
            telemetry.update();
        }

        Turber.setVelocity(0);
    }

    public void magMoveArm(int level, int Velocity, int servoTarget, double depoHold, long threeTime){
        boolean done = false;
        boolean crossed = false;

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm.setVelocity(-Velocity, AngleUnit.DEGREES);

        while(!isStopRequested() && !done){

            if(level == 3){
                if(armLLimit.isPressed()){
                    while(armLLimit.isPressed()){}
                    sleep(threeTime);
                    done = true;
                }
            } else if(level == 2){
                if(armLLimit.isPressed()){
                    sleep(65);
                    done = true;
                }
            } else if(level == 1){
                if(armRLimit.isPressed()) {
                    done = true;
                }
            }

            if((level == 3 || level == 2) && armRLimit.isPressed()) {
                depoDrop(depoHold / 1.25);
                runtime.reset();
                crossed = true;
            }
            else if(crossed && runtime.milliseconds() < servoTarget) {
                depoDrop(depoHold);
            }

            telemetry.addLine(">>>>GOING UP<<<<<<<");
            telemetry.addData("done", done);
            telemetry.update();
        }

        depoDrop(depoHold);

        Arm.setVelocity(0);

        sleep(50);
    }

    public void depoCollect(){ dep.setPosition(0.076); }
    public void depoDrop(){
        dep.setPosition(1);
    }
    public void depoDrop(double position){ dep.setPosition(position);}
    public void depoHold(){ dep.setPosition(0.55); }
    public void depo2Drop(){
        dep.setPosition(0.87);
    }

    public void adjust(double Power){
        if(getAngle() < 0){

        }
    }

    public void imuTurn(boolean direction, double angle, double power){
        if (!direction){
            LeftForward.setPower(-power);
            LeftBack.setPower(-power);
            RightForward.setPower(power);
            RightBack.setPower(power);

            while (!isStopRequested() && getAngle() <= angle){
                telemetry.addData("currentAngle", getAngle());
                telemetry.addData("currentEncoderPosition", LeftForward.getCurrentPosition());
                telemetry.addLine("Turning :)");
                telemetry.update();
            }

            LeftForward.setPower(0);
            LeftBack.setPower(0);
            RightForward.setPower(0);
            RightBack.setPower(0);
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

    public void collectFreight(double Power, int strafeDirection, DistanceSensor sensor){
        runIntake();

        Arm.setPower(0.25);

        moveDistance(FFORWARD, 0.5, 37, 0, 5, FrontDistance);

        stopMotors();

       // moveDistance(strafeDirection, 0.5, 5, 0, 2, sensor);

        startMotors(Power, 0.0);

        while(ElementDistance() > 92 && !isStopRequested() && FrontDistance.getDistance(DistanceUnit.MM) > 12){
            correction = drive.performPID(getAngle());

            LeftForward.setPower(Power - correction);
            LeftBack.setPower(Power - correction);
            RightForward.setPower(Power + correction);
            RightBack.setPower(Power + correction);


            telemetry.addData("Eled", ElementDistance());
            telemetry.addData("Forward Distance", FrontDistance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }


        stopMotors();
        Arm.setPower(0);
    }

    public void reverseIntake(){
        Intake.setPower(-1);
    }

    public void toCarousel(double power){
        startMotors(0.4, 0.0);

        while(LeftBack.getVelocity() < 20){
            telemetry.addData("LeftBack Velo", LeftBack.getVelocity());
            telemetry.update();
        }

        stopMotors();
    }


    public static class ContourPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        int returnMat = Caruso.reg;
        int color = Caruso.BLUE_CAROUSAL;

        private static int pixRightx = 350, pixLeftx = 50;



        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        public ContourPipeline() {}
        public ContourPipeline(int returnMat){
            this.returnMat = returnMat;
        }

        public ContourPipeline(int returnMat, int color){
            this(returnMat);
            this.color = color;
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();

            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 1);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 155, 210, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            double[] pixLeft = thresholdMat.get(70, input.rows() / 2 + 1);
            valLeft = (int) pixLeft[0];

            //get values from frame
            double[] pixRight = thresholdMat.get(pixRightx,input.rows() / 2);//gets value at circle
            valRight = (int) pixRight[0];



            //create three points
            Point pointRight = new Point(pixRightx,input.rows() / 2);
            Point pointLeft = new Point(50, input.rows() / 2);

            //draw circles on those points
            Imgproc.circle(thresholdMat, pointRight, 10, new Scalar(255, 0, 0), 4);//draws circle
            Imgproc.circle(thresholdMat, pointLeft, 10, new Scalar(255,0,0), 4);
            Imgproc.circle(all, pointRight, 10, new Scalar(255, 0, 0), 4);//draws circle



            if(returnMat == thresh){
                return thresholdMat;
            } else if(returnMat == bNw){
                return all;
            } else{
                return input;
            }

        }

        public int right(){return valRight;}
        public int left(){return valLeft;}
        public int getPos() {

            if (color == Caruso.BLUE_CAROUSAL) {
                if (valRight == 0) {
                    return 3;
                } else if (valLeft == 0) {
                    return 2;
                } else
                    return 1;
            } else {
                if (valRight == 0) {
                    return 2;
                } else if (valLeft == 0) {
                    return 1;
                } else
                    return 3;
            }
        }
    }

    public static class RectPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        static final Scalar ORANGE = new Scalar(255, 110, 2);


        public int width;
        public int x;
        public int y;
        public int height;
        public double area;
        public int THRESH;
        public int MAX;
        public double targetXPos;
        public int color;

        public RectPipeline(){
            THRESH = 170;
            MAX = 200;
            targetXPos = 180;
        }

        public RectPipeline(int THRESH, int MAX){
            this();
            this.THRESH = THRESH;
            this.MAX = MAX;
        }

        public RectPipeline(int THRESH, int MAX, double targetXPos){
            this(THRESH, MAX);
            this.targetXPos = targetXPos;
        }

        public RectPipeline(int THRESH, int MAX, int color){
            this(THRESH, MAX);
            this.color = color;
        }

        public RectPipeline(int THRESH, int MAX, int color, double targetXPos){
            this(THRESH, MAX, color);
            this.targetXPos = targetXPos;
        }




        enum DuckConfiguration{
            ONE,
            TWO,
            THREE,
        }

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }



        private RectPipeline.Stage stageToRenderToViewport = RectPipeline.Stage.detection;
        private RectPipeline.Stage[] stages = RectPipeline.Stage.values();

        public volatile RectPipeline.DuckConfiguration configuration = RectPipeline.DuckConfiguration.ONE;

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }


        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();

            //color diff cr.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);;//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 1);//takes cr difference and stores (coi is channel of interest)
            //0 = Y, 1 = Cr, 2 = Cb

            //b&w (thresholding to make a map of the desired color
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, THRESH, MAX, Imgproc.THRESH_BINARY);

            Mat eroder= Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size( 3, 3));

            Mat dilator = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(8,8));

            Imgproc.erode(thresholdMat, thresholdMat, eroder);
            Imgproc.erode(thresholdMat, thresholdMat, eroder);

            Imgproc.dilate(thresholdMat, thresholdMat, dilator);
            Imgproc.dilate(thresholdMat, thresholdMat, dilator);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
            yCbCrChan2Mat.copyTo(all);
            Imgproc.drawContours(all, contoursList, -1, ORANGE, 4, 8);

            int maxHeight = 0;
            int tempX = 0;

            Rect maxRect = new Rect();

            for(MatOfPoint c : contoursList ){
                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                Rect rect = Imgproc.boundingRect(copy);

                int h = rect.height;

                if(h > maxHeight && rect.area() > 5000 && rect.y > 215){
                    maxHeight = h;
                    maxRect = rect;
                }

                c.release();
                copy.release();
            }

            Imgproc.rectangle(thresholdMat, maxRect, new Scalar(0, 0.0, 255), 10);
            Imgproc.line(all, new Point(0, Caruso.CAMERA_HEIGHT/2), new Point(Caruso.CAMERA_WIDTH, Caruso.CAMERA_HEIGHT/2), new Scalar(255, 255,255), 3);
            Imgproc.rectangle(input, maxRect, new Scalar(0, 0.0, 255), 10);
            Imgproc.rectangle(all, maxRect, new Scalar(0, 0.0, 255), 10);

            // Imgproc.line(input, new Point(50,0), new Point(50, 480), new Scalar(255, 110, 2), 2);
            // Imgproc.line(all, new Point(50,0), new Point(50, 480), new Scalar(255, 110, 2), 2);


            width = maxRect.width;
            x = maxRect.x + (maxRect.width / 2);
            y = maxRect.y;
            area = maxRect.area();
            height = maxRect.height;

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection:{
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }

        public int getWidth() {return width;}
        public int getXPos() {return x;}
        public int getYPos() {return y;}
        public int getHeight() {return height;}
        public double getArea() {return area;}

        public int getConfig() {

            if (color == Caruso.BLUE_CAROUSAL) {
                if (getXPos() > 250) {
                    return 2;
                } else if (1 < getXPos() && getXPos() < 250) {
                    return 1;
                } else
                    return 3;
            } else if (color == Caruso.BLUE_PICKUP) {
                if (getXPos() > 150) {
                    return 3;
                } else if (1 < getXPos() && getXPos() < 150) {
                    return 2;
                } else
                    return 1;
            } else if (color == Caruso.RED_PICKUP) {
                if (getXPos() > 150) {
                    return 1;
                } else if (1 < getXPos() && getXPos() < 150) {
                    return 2;
                } else
                    return 3;
            } else {
                if (getXPos() > 250) {
                    return 3;
                } else if (1 < getXPos() && getXPos() < 250) {
                    return 2;
                } else
                    return 1;
            }
        }

    }

    public static class SStageSwitchingPipeline extends OpenCvPipeline {
        Mat HSVChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();


        public static final int thresholdFactor = 10;

        Scalar lowerBlue = new Scalar(226 - thresholdFactor,21 - thresholdFactor,46 - thresholdFactor);
        Scalar upperBlue = new Scalar(35 + thresholdFactor, 16 + thresholdFactor, 89 + thresholdFactor);

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        private static float[] midPos = {4f / 8f, 4f / 8f};//0 = col, 1 = row
        private static int valMid = -1;

        public SStageSwitchingPipeline(){

        }

        @Override
        public Mat processFrame(Mat input) {

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, HSVChan2Mat, Imgproc.COLOR_RGB2HSV);//converts rgb to ycrcb
            Core.extractChannel(HSVChan2Mat, HSVChan2Mat, 2);//takes cb difference and stores

            //b&w
            //Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 75, 160, Imgproc.THRESH_BINARY);

            Core.inRange(HSVChan2Mat, lowerBlue, upperBlue, thresholdMat);

            //get values from frame
            double[] pixMid = thresholdMat.get(Caruso.CAMERA_WIDTH / 2, Caruso.CAMERA_HEIGHT / 2);//gets value at circle
            valMid = (int) pixMid[0];

            //create three points
            Point pointMid = new Point(input.rows() / 2, input.cols() / 2 + 50);

            //draw circles on those points
            Imgproc.circle(all, pointMid, 10, new Scalar(255, 0, 0), 4);//draws circle

            return thresholdMat;
        }

        public int getValMid(){
            return valMid;
        }

    }

    public static class navTarget extends OpenCvPipeline{

        Mat img_gray;
        Mat img_rgb;

        @Override
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, img_gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.cvtColor(input, img_rgb, Imgproc.COLOR_BGR2GRAY);
            return input;


        }
    }

}