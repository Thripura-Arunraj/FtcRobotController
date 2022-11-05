package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Caruso;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "simpleBlueCarousel", group = "auto")
public class simpleBlueCarousel extends LinearOpMode {

    OpenCvCamera camera2;
    public static int ARMLEVEL = 1525;
    public static double backDistance, strafeDistance, dropPosition;
    public static int CONFIG;
    public static int sleepTime = 0;

    Caruso c;

    @Override
    public void runOpMode() throws InterruptedException{
        c = new Caruso(hardwareMap, telemetry);
        Caruso.RectPipeline color = new Caruso.RectPipeline(165, 180, Caruso.BLUE_CAROUSAL);
        Turret turret = new Turret();

        c.initCamera(camera2, hardwareMap, color, Caruso.CAMERA_WIDTH, Caruso.CAMERA_HEIGHT, 1);
        c.depoCollect();

        c.falsify();
        while (!isStopRequested() && !isStarted()) {
            CONFIG = color.getConfig();

            if (CONFIG == 3) {
                strafeDistance = 35;
                backDistance = 32;
                dropPosition = 0.96;
            }
            else if (CONFIG == 2) {
                strafeDistance = 35;
                backDistance = 22;
                dropPosition = 0.84;
            }
            else if (CONFIG == 1) {
                strafeDistance = 35;
                backDistance = 22;
                dropPosition = 0.77;
            }

            if(gamepad2.dpad_left)
                sleepTime -= 500;
            else if(gamepad2.dpad_right)
                sleepTime += 500;


            c.initTelem();
            telemetry.addLine(">>>>>>>>CAMERA<<<<<<<<");
            telemetry.addData("position", CONFIG);
            telemetry.addData("CONFIG", color.getConfig());
            telemetry.addData("xpos", color.getXPos());
            telemetry.addData("area", color.getArea());
            telemetry.addData("strafeDistance", strafeDistance);
            telemetry.addData("backDistance", backDistance);
            telemetry.addData("dropPosition", dropPosition);
            telemetry.addData("------->sleepTime", sleepTime);
            // telemetry.addData("cam FPS", camera1.getFps());
            telemetry.update();
        }

        if (opModeIsActive()) {

            //SLEEP FOR A PRESET DURATION

            sleep(sleepTime);

            c.magMoveArm(CONFIG, 55, 920, .45, 35);

            sleep(200);

            //if(CONFIG == 3)
                //c.moveArm(30, 100, 200);
            //else if(CONFIG == 2)
              //  c.moveArm(155, 70, 200);

            if(CONFIG == 2)
                c.startMotors(c.Forward, 0.4, 100);
            if(CONFIG == 1)
                c.startMotors(c.Forward, 0.35, 100);

            c.moveDistance(c.RLEFT, 0.55, strafeDistance,0,3.5, c.RightDistance);

            if(CONFIG == 3)
                c.startMotors(c.BACKWARD, 0.3, 250);
           // if(CONFIG == 2)
                //c.startMotors(c.BACKWARD, 0.25, 180);

            // c.moveEncoders(c.RIGHT, 0.25, 1400, 0);

            //c.moveDistance(c.DOWNRIGHT, 0.65, backDistance, 0, 3);

            c.depoDrop(dropPosition);

            sleep(1400);

            c.depoDrop(0.05);

            sleep(500);

            c.moveDistance(c.FFORWARD, 0.35, 20, 0, 2, c.FrontDistance);

            c.armDown();

            sleep(200);

            c.imuTurn(c.clock, -87, 0.5);

            sleep(300);

            c.moveDistance(c.LEFT, 0.5, 4, -90, 2.5);

            sleep(200);

            c.moveDistance(c.FFORWARD, 0.3, 14.85, -90,4.2, c.FrontDistance);

            sleep(500);

            c.startMotors(c.Forward, 0.2, 600);

            c.Carousel.setPower(0.2);

            telemetry.addLine("Moving Carousal");
            telemetry.update();

            sleep(3500);

            c.Carousel.setPower(0);

            c.moveDistance(c.FBACKWARD, 0.38, 22, -90, 2.5, c.FrontDistance);


        }
    }



    private class Turret extends Thread{
        @Override
        public void run(){
            try {
                while(true)
                {
                    if(c.Position == c.NEAR_HUB) {
                        c.Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        c.depoCollect();
                        c.moveArm(ARMLEVEL, 0.67, 350);
                        sleep(50);
                        //TURN TURRET TOWARDS SHIPPING HUB
                        /*c.moveTurret(c.cclock, 0.35);
                        c.moveTurret(c.cclock, 85, 0.35);*/
                        c.Position = c.LEAVE_AS_IS;
                    }
                    else if (c.Position == c.NEAR_PARK) {
                        c.depoCollect();
                        sleep(100);
                        c.moveArm(ARMLEVEL, -0.67, 350);
                        sleep(50);
                        c.Position = c.LEAVE_AS_IS;
                    }
                }
            } catch (InterruptedException interruptedException) {
                interruptedException.printStackTrace();
            }


        }

    }
}