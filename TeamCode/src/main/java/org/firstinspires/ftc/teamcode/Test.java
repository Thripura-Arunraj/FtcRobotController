package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Caruso;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "Test", group = "test")
public class Test extends LinearOpMode {

    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException{

        Caruso caruso = new Caruso(hardwareMap, telemetry);
        Caruso.RectPipeline color = new Caruso.RectPipeline(165, 180, Caruso.BLUE_PICKUP);
        caruso.initCamera(camera, hardwareMap, color, Caruso.CAMERA_WIDTH, Caruso.CAMERA_HEIGHT, 1);

        while(!isStopRequested() && !isStarted()){
            caruso.initTelem();
            telemetry.addData("x", color.getXPos());
            telemetry.addData("y", color.getYPos());
            telemetry.addData("Randimization", color.getConfig());
            telemetry.update();
        }

        if (opModeIsActive()){

            caruso.initTelem();
            telemetry.addData("x", color.getXPos());
            telemetry.addData("y", color.getYPos());
            telemetry.addData("Randimization", color.getConfig());
            telemetry.update();

        }
    }
}
