package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Caruso;
import org.openftc.easyopencv.OpenCvCamera;


@Autonomous(name = "RedPickupAuto", group = "auto")
public class RedPickupAuto extends LinearOpMode {

    OpenCvCamera camera1;
    OpenCvCamera camera2;
    Caruso c;
    Caruso.RectPipeline color;
    public static int ARMLEVEL;
    public static double STRAFE_DISTANCE;
    public static double DIAG_DISTANCE;
    public static double depoPosition;
    public static int BACK_ENCODERS;
    public static int CONFIG;
    public static int sleepTime = 0;
    public ElapsedTime autoTime;

    @Override
    public void runOpMode() throws InterruptedException{

        c = new Caruso(hardwareMap, telemetry);
        color = new Caruso.RectPipeline(165, 180, Caruso.RED_PICKUP);
        Movement movement = new Movement();
        Turret turret = new Turret();
        autoTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        c.initCamera(camera1, hardwareMap, color, Caruso.CAMERA_WIDTH, Caruso.CAMERA_HEIGHT, 1);
        c.depoCollect();

        c.falsify();

        while(!isStopRequested() && !isStarted()){
            if(color.getConfig() == 3) {
                STRAFE_DISTANCE = 15.3;
                DIAG_DISTANCE = 13.6;
                depoPosition = 0.98;
                BACK_ENCODERS = 380;
            }
            else if(color.getConfig() == 2) {
                STRAFE_DISTANCE = 13.2;
                DIAG_DISTANCE = 14.3;
                depoPosition = 0.87;
                BACK_ENCODERS = 340;
            }
            else if(color.getConfig() == 1) {
                STRAFE_DISTANCE = 12.5;
                DIAG_DISTANCE = 12.9;
                depoPosition = 0.8;
                BACK_ENCODERS = 380;
            }

            if(gamepad2.dpad_right) {
                sleepTime += 500;
                sleep(200);
            }
            if(gamepad2.dpad_left){
                sleepTime -= 500;
                sleep(200);
            }if(sleepTime < 0)
                sleepTime = 0;

            c.initTelem();
            telemetry.addData("----> SLEEP TIME", sleepTime);
            telemetry.addData("ARMLEVEL", ARMLEVEL);
            telemetry.addData("At hub?", Caruso.AT_HUB);
            telemetry.addData("at Wall?", Caruso.AT_WALL);
            telemetry.addData("dropped?", Caruso.DROPPED);
            telemetry.addData("collected?", Caruso.COLLECTED);
            telemetry.addData("arm done?", Caruso.ARMDONE);
            telemetry.addData("DONE?", Caruso.DONE);
            telemetry.addLine(">>>>>>>>CAMERA<<<<<<<<");
            telemetry.addData("config", color.getConfig());
            telemetry.addData("xPos", color.getXPos());
            // telemetry.addData("cam FPS", camera1.getFps());
            telemetry.update();

            CONFIG = color.getConfig();

        }

        if (opModeIsActive()){

            sleep(sleepTime);

            movement.start();
            turret.start();

            try {
                while (opModeIsActive() && !Caruso.DONE) {

                }

            } catch (Exception e){
                telemetry.addLine("FATAL ERROR!!!!!!!!!!!!!!!!!!!!!!");
                telemetry.update();
            }

            movement.interrupt();
            turret.interrupt();


        }
    }
    private class Movement extends Thread{
        public Movement(){
            this.setName("Movement");
        }

        @Override
        public void run(){
            try{
                while(!isInterrupted()){
                    //TODO DROP PRELOADED ELEMENT

                    //TODO TIGHTEN ALL SET SCREWS
                    //TODO MAKE RED PICK UP

                    //MOVE IN FRONT OF SHIPPING HUB

                    //1 and 3 = 8.5

                    c.falsify();

                    sleep(350);

                    c.moveDistance(c.RLEFT, 0.5, STRAFE_DISTANCE, 0, 1.5, c.RightDistance);

                    sleep(10);

                    //3 = 18.5
                    //1 = 15.7

                    //c.moveDistance(c.DOWNRIGHT, 0.8, DIAG_DISTANCE, 0, 5);

                    c.moveEncoders(c.BACKWARD, 0.4, BACK_ENCODERS, 0);

                    Caruso.AT_HUB = true;

                    while(!Caruso.DROPPED){}

                    c.moveDistance(c.RIGHT, 0.6, 13, 0, 1.5, c.RightDistance);

                    //TODO DROP SET AMOUNT
                    c.moveDistance(c.UPRIGHT, 0.9, 4, 0, 7, c.RightDistance);

                    Caruso.AT_WALL = true;

                    sleep(200);

                    //c.collectFreight(0.3, c.RIGHT, c.RightDistance);

                    c.moveDistance(c.FFORWARD, 0.5, 32, 0, 5, c.FrontDistance);

                    Caruso.COLLECTED = true;

                    sleep(200);

                    //c.moveDistance(c.FBACKWARD, 0.6, 40, 0, 4, c.FrontDistance);

                    //c.moveEncoders(c.BACKWARD, 0.4, 460, 0);

                    //move encoders

                    c.moveDistance(c.RLEFT, 0.5, 20, 0, 4, c.RightDistance);

                    c.stopMotors();

                    sleep(10);

                    Caruso.DONE = true;

                    //TODO PARK IN

                    /*c.moveEncoders(c.BACKWARD, 0.6, 700, 0);

                    Caruso.COLLECTED = true;

                    c.moveDistance(c.RIGHT, 0.5, 3, 0,2);
*/


                }
            } catch (InterruptedException e){

            } catch(Exception e){

            }
        }
    }

    private class Turret extends Thread{
        public Turret(){
            this.setName("Turret");
        }

        @Override
        public void run(){
            try{
                while(!isInterrupted()){
                    //TODO DROP PRELOADED ELEMENT

                    Caruso.NEXTELEMENT = false;

                    c.magMoveArm(CONFIG, 80, 680, 0.55, 100);

                    sleep(50);

                    //TURN TURRET TOWARDS SHIPPING HUB

                    c.moveTurret(c.clock, 0.35);

                    //1 = 80 encoders

                    c.moveTurret(c.clock, 160, 0.35);

                    while(!Caruso.AT_HUB){}

                    c.depoDrop(depoPosition);

                    //DROP ELEMENT
                    sleep(1200);

                    c.depoCollect();

                    Caruso.DROPPED = true;

                    //TODO GO BACK TO WALL

                    if(CONFIG == 1)
                        c.moveArm(50, 100, 200);

                    c.centerTurret(c.clock);

                    sleep(80);

                    while (!Caruso.AT_WALL) {}

                    c.armDown();

                    Caruso.ARMDONE = true;

                    while(!Caruso.COLLECTED){}

                    sleep(100);

                    c.reverseIntake();

                    sleep(500);

                    c.stopIntake();

                    Caruso.AT_HUB = false;

                    //TODO DROP SET AMOUNT

                    //TODO PARK IN


                }
            }
            catch(InterruptedException e){} catch(Exception e){}
        }
    }



}
