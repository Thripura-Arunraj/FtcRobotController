/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

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
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private DcMotor RightForward, RightBack, LeftForward, LeftBack;
    public final int FORWARD = 0, BACKWARD = 1, LEFT = 2, RIGHT = 3;
//    private Servo Deposit;
//    Caruso c = new Caruso();

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
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        // Turner.setPositon(0);
        RightForward = hardwareMap.dcMotor.get("RightFront");
        RightBack = hardwareMap.dcMotor.get("RightBack");
        LeftForward = hardwareMap.dcMotor.get("LeftFront");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");

//        Lifter = hardwareMap.dcMotor.get("Lifter");

//        Deposit = hardwareMap.get(Servo.class, "Deposit");

        RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        Deposit.setDirection(Servo.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        // Turner.setDirection(Servo.Direction.REVERSE);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

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
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
            configuration = 2;
        }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        if (opModeIsActive()) {
            //moveEncoders(BACKWARD, 0.45, 250);
            moveWithTimeForward(1650);
            if (configuration == 3) {
                //moveWithTimeForward(300);
            }
            else if (configuration == 2) {
                moveWithTimeBackward(700);
            }
            else if (configuration == 1){
                moveWithTimeBackward(1400);
            }
            else {}
            strafeLeft(700);
            if(configuration == 2)
                moveWithTimeForward(100);

        }
    }

    public void moveWithTimeForward(int milliseconds) {
        double power = .2;
        LeftForward.setPower(-power);
        LeftBack.setPower(-power);
        RightForward.setPower(-power);
        RightBack.setPower(-power);
        sleep(milliseconds);
        LeftForward.setPower(0);
        LeftBack.setPower(0);
        RightForward.setPower(0);
        RightBack.setPower(0);

    }

    public void moveWithTimeBackward(int milliseconds) {
        double power = -0.32;
        LeftForward.setPower(-power);
        LeftBack.setPower(-power);
        RightForward.setPower(-power);
        RightBack.setPower(-power);
        sleep(milliseconds);
        LeftForward.setPower(0);
        LeftBack.setPower(0);
        RightForward.setPower(0);
        RightBack.setPower(0);

    }

    public void strafeLeft(int milliseconds) {
        double power = 0.65;
        LeftForward.setPower(power);
        LeftBack.setPower(-power);
        RightForward.setPower(-power);
        RightBack.setPower(power);
        sleep(milliseconds);
        LeftForward.setPower(0);
        LeftBack.setPower(0);
        RightForward.setPower(0);
        RightBack.setPower(0);
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
    public void moveEncoders(int Direction, double Power, int TargetPosition) {
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (Direction == Forward) {
            RightBack.setTargetPosition(TargetPosition);

            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(RightBack.getCurrentPosition()) <= Math.abs(RightBack.getTargetPosition())) {

                LeftForward.setPower(Power);
                LeftBack.setPower(Power);
                RightForward.setPower(Power);
                RightBack.setPower(Power);

            }

        } else if (Direction == BACKWARD) {
            RightBack.setTargetPosition(-TargetPosition);

            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(RightBack.getCurrentPosition()) <= Math.abs(RightBack.getTargetPosition())) {
                LeftForward.setPower(-Power);
                LeftBack.setPower(-Power);
                RightForward.setPower(-Power);
                RightBack.setPower(-Power);
            }

        } else if (Direction == LEFT) {
            RightForward.setTargetPosition(TargetPosition);

            RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs( RightForward.getCurrentPosition()) <= Math.abs(RightForward.getTargetPosition())) {

                LeftForward.setPower(-Power);
                LeftBack.setPower(Power);
                RightForward.setPower(Power);
                RightBack.setPower(-Power);
            }

        } else if (Direction == RIGHT) {
            RightForward.setTargetPosition(-TargetPosition);

            RightForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (!isStopRequested() && Math.abs(RightForward.getCurrentPosition()) <= Math.abs(RightForward.getTargetPosition())) {

                LeftForward.setPower(Power);
                LeftBack.setPower(-Power);
                RightForward.setPower(-Power);
                RightBack.setPower(Power);
            }

        }

        LeftForward.setPower(0);
        LeftBack.setPower(0);
        RightForward.setPower(0);
        RightBack.setPower(0);
    }

}