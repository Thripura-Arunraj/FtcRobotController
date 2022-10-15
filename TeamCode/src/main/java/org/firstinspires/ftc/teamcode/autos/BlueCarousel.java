package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Caruso;
import org.openftc.easyopencv.OpenCvCamera;

public class BlueCarousel extends LinearOpMode {

    OpenCvCamera camera1;
    public static int ARMLEVEL;

    Caruso c;

    @Override
    public void runOpMode(){
        c = new Caruso(hardwareMap, telemetry);
        Caruso.ContourPipeline color = new Caruso.ContourPipeline();
        Movement movement = new Movement();
        Turret turret = new Turret();
        c.initCamera(camera1, hardwareMap, color, Caruso.CAMERA_WIDTH, Caruso.CAMERA_HEIGHT, 1);

        while(!isStopRequested() && !isStarted()){
            c.initTelem();
            telemetry.addData("ARMLEVEL", ARMLEVEL);
            telemetry.addData("At hub?", Caruso.AT_HUB);
            telemetry.addData("at Wall?", Caruso.AT_WALL);
            telemetry.addData("dropped?", Caruso.DROPPED);
            telemetry.addData("collected?", Caruso.COLLECTED);
            telemetry.addData("arm done?", Caruso.ARMDONE);
            telemetry.addLine(">>>>>>>>CAMERA<<<<<<<<");
            telemetry.addData("right", color.right());
            telemetry.addData("left", color.left());
            telemetry.addData("position", color.getPos());
            // telemetry.addData("cam FPS", camera1.getFps());
            telemetry.update();

            if(color.getPos() == 3)
                ARMLEVEL = 1525;
            else if(color.getPos() == 2)
                ARMLEVEL = 1405;
            else if(color.getPos() == 1)
                ARMLEVEL = 1100;
        }

        if (opModeIsActive()){

            movement.start();
            turret.start();

            try {
                while (opModeIsActive() && !Caruso.COLLECTED) {
                    telemetry.addData("At hub?", Caruso.AT_HUB);
                    telemetry.update();

                    if(Caruso.COLLECTED){
                        sleep(5);
                        Caruso.AT_HUB = false;
                        Caruso.AT_WALL = false;
                        Caruso.COLLECTED = false;
                        Caruso.DROPPED = false;
                        Caruso.ARMDONE = false;
                    }
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

                    c.moveEncoders(c.BACKWARD, 0.5, 300, 0);

                    Caruso.READYFORARM = true;

                    c.moveDistance(c.RIGHT, 0.5, 20, 0, 2, c.RightDistance);

                    Caruso.AT_HUB = true;

                    while(!Caruso.DROPPED){}

                    c.moveDistance(c.LEFT, 0.5, 3, 0, 2, c.RightDistance);

                    sleep(1);

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

                    while(!Caruso.READYFORARM){}

                    c.moveArm(ARMLEVEL, 0.3, 300);

                    c.moveTurret(c.clock, 0.35);

                    while(!Caruso.AT_HUB){}

                    c.depoDrop();

                    sleep(1000);

                    c.depoCollect();

                    Caruso.DROPPED = true;

                    c.centerTurret(c.cclock);

                    c.sleep(50);

                    c.armDown();

                    interrupt();
                }
            }
            catch(InterruptedException e){} catch(Exception e){}
        }
    }
}
