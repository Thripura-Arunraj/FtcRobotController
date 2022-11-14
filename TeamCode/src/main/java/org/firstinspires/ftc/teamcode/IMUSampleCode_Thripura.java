package org.firstinspires.ftc.teamcode.openCV;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Position;



@Autonomous
public class IMUsampleCode_Thripura extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor RightForward, RightBack, LeftForward, LeftBack;
        BNO055IMU imu;


        RightForward = hardwareMap.dcMotor.get("RightForward");
        RightBack = hardwareMap.dcMotor.get("RightBack");
        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");

        RightBack.setDirection(DcMotor.Direction.FORWARD);
        RightForward.setDirection(DcMotor.Direction.FORWARD);
        LeftForward.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        Position initialPosition = imu.getPosition();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        telemetry.addData("Position x", initialPosition.x);
        telemetry.addData("Position y", initialPosition.y);
        telemetry.addData("Position z", initialPosition.z);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // Write code to start all the motors
        LeftForward.setPower(1);
        RightForward.setPower(1);
        LeftBack.setPower(1);
        RightBack.setPower(1);


        double desiredX1 = 100; //Set value by manually moving Robot to Location 1
        double desiredY1 = 100;
        double desiredZ1 = 100;

        double desiredX2 = 200; //Set value by manually moving Robot to Location 2
        double desiredY2 = 200;
        double desiredZ2 = 200;

        double desiredX3 = 50; //Set value by manually moving Robot to Location 3
        double desiredY3 = 50;
        double desiredZ3 = 50;

        int intConfiguration = 1; // Set configuration value from April Tag detection


        //Check for position x,y,z continuously until we reach desired location - Configuration 1
        while (imu.getPosition().x == desiredX1 && imu.getPosition().y == desiredY1 && imu.getPosition().z == desiredZ1)
        {
            LeftForward.setPower(0);
            RightForward.setPower(0);
            LeftBack.setPower(0);
            RightBack.setPower(0);

            //If the april tag reads 2 then go right
            if (intConfiguration == 2) {

                //Write code to move right
                LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
                LeftForward.setDirection(DcMotorSimple.Direction.FORWARD);
                RightForward.setDirection(DcMotorSimple.Direction.REVERSE);
                RightBack.setDirection(DcMotorSimple.Direction.FORWARD);

                //set power
                LeftForward.setPower(1);
                RightForward.setPower(1);
                LeftBack.setPower(1);
                RightBack.setPower(1);


                //Check desiredPosition2 and stop the motors
                while (imu.getPosition().x == desiredX2 && imu.getPosition().y == desiredY2 && imu.getPosition().z == desiredZ2) {
                    LeftForward.setPower(0);
                    RightForward.setPower(0);
                    LeftBack.setPower(0);
                    RightBack.setPower(0);
                }
                }
            }

            //If the april tag reads 3 then go left
            if (intConfiguration == 3) {

                //Write code to move left
                LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
                LeftForward.setDirection(DcMotorSimple.Direction.REVERSE);
                RightForward.setDirection(DcMotorSimple.Direction.FORWARD);
                RightBack.setDirection(DcMotorSimple.Direction.REVERSE);

                //set power
                LeftForward.setPower(1);
                RightForward.setPower(1);
                LeftBack.setPower(1);
                RightBack.setPower(1);

                //Check desiredPosition3 and stop the motors
                while (imu.getPosition().x == desiredX3 && imu.getPosition().y == desiredY3 && imu.getPosition().z == desiredZ3) {
                    LeftForward.setPower(0);
                    RightForward.setPower(0);
                    LeftBack.setPower(0);
                    RightBack.setPower(0);
                }
            }
        }
    }



