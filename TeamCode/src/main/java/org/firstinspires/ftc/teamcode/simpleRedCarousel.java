package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Caruso;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "simpleRedCarousel", group = "auto")
public class simpleRedCarousel extends LinearOpMode {

    OpenCvCamera camera2;
    public static int ARMLEVEL = 1525;
    public static double backDistance, strafeDistance, dropPosition;
    public static int encoders;
    public static int CONFIG;
    public static int sleepTime;

    Caruso c;
//roses are red, violets are blue, ur a chad caruso, chad is a fat robot
    @Override
    public void runOpMode() throws InterruptedException{
        c = new Caruso(hardwareMap, telemetry);
        Caruso.RectPipeline color = new Caruso.RectPipeline(165, 200, Caruso.RED_CAROUSAL);
        Turret turret = new Turret();

        c.initCamera(camera2, hardwareMap, color, Caruso.CAMERA_WIDTH, Caruso.CAMERA_HEIGHT, 2);
        c.depoCollect();

        c.falsify();
        while (!isStopRequested() && !isStarted()) {
            CONFIG = color.getConfig();

            if (CONFIG == 3) {
                strafeDistance = 38;
                backDistance = 41;
                dropPosition = 0.90;
            }
            else if (CONFIG == 2) {
                strafeDistance = 37;
                backDistance = 22;
                dropPosition = 0.84;
            }
            else if (CONFIG == 1) {
                strafeDistance = 37;
                backDistance = 22;
                dropPosition = 0.8;
            }

            if(gamepad2.dpad_left)
                sleepTime -= 500;
            else if(gamepad2.dpad_right)
                sleepTime += 500;

// tee hee im so cool   -aarav

            c.initTelem();
            telemetry.addLine(">>>>>>>>CAMERA<<<<<<<<");
            telemetry.addData("position", CONFIG);
            telemetry.addData("CONFIG", color.getConfig());
            telemetry.addData("xpos", color.getXPos());
            telemetry.addData("area", color.getArea());
            telemetry.addData("strafeDistance", strafeDistance);
            telemetry.addData("backDistance", backDistance);
            telemetry.addData("ARMLEVEL", ARMLEVEL);
            // telemetry.addData("cam FPS", camera1.getFps());
            telemetry.update();
        }
// krauggie was born on December 20, 2004 at 12:20pm.
        if (opModeIsActive()) {

            sleep(sleepTime);

            c.magMoveArm(CONFIG, 65, 800, 0.55, 35);

            //if(CONFIG == 3)
              //  c.moveArm(60, 65, 200);
            //else if(CONFIG == 2)
               // c.moveArm(90, 65, 200);

            c.moveDistance(c.LRIGHT, 0.55, strafeDistance,0,4.2, c.LeftDistance);

            if(CONFIG == 3)
                c.startMotors(c.BACKWARD, 0.3, 380);
            else if(CONFIG == 1)
                c.startMotors(c.BACKWARD, 0.4, 190);

           // c.moveEncoders(c.RIGHT, 0.25, 1400, 0);

            //c.moveDistance(c.DOWNRIGHT, 0.65, backDistance, 0, 3);

            c.depoDrop(dropPosition);

            sleep(1400);

            c.moveDistance(c.FFORWARD, 0.35, 21, 0, 2, c.FrontDistance);

            c.depoCollect();

            sleep(500);

            c.armDown();

            c.moveDistance(c.FFORWARD, 0.35, 5, 0, 2, c.FrontDistance);

            sleep(200);

            c.moveDistance(c.LEFT, 0.5, 16, 0,5, c.LeftDistance);

            sleep(500);

            c.startMotors(c.LEFT, 0.3, 860);

            c.Carousel.setPower(-0.2);

            telemetry.addLine("Moving Carousal");//ur a carousal
            telemetry.update();

            sleep(3500);

            c.Carousel.setPower(0);

           c.imuTurn(c.cclock, 0, 0.45);

           sleep(200);
// tu madre es muy gordo
            c.moveDistance(c.LRIGHT, 0.6, 26, -1, 2.5, c.LeftDistance);

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