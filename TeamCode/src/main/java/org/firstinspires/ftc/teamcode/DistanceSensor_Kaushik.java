package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous
public class DistanceSensor_Kaushik extends LinearOpMode {
    private DcMotor LeftForward, LeftBack, RightForward, RightBack;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        String name_of_distance_sensor;

        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        RightForward = hardwareMap.dcMotor.get("RightForward");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        distanceSensor = hardwareMap.get(DistanceSensor.class, name_of_distance_sensor);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Distance (in)", distanceSensor.getDistance(DistanceUnit.INCH));

            telemetry.update();
        }
    }

}