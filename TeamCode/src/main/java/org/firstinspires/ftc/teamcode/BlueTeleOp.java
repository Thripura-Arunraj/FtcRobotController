package org.firstinspires.ftc.teamcode;
// import Rahuls's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Caruso;

@TeleOp(name = "BlueTeleOp", group = "")
public class BlueTeleOp extends LinearOpMode {

    private double Multiplier = 0.6;
    public double contPower;
    public ElapsedTime runtime = new ElapsedTime();

    public VoltageSensor battery;

    public static boolean idle = true;

    public static boolean tiltLogicSwitch = false;

    private double Scale(double Input) {
        double Output = Input * Math.abs(Input);
        return Output;
    }

    int side = 1;

    static double depoPosition = 0.98;

    Caruso c;

    @Override
    public void runOpMode() throws InterruptedException {

        c = new Caruso(hardwareMap, telemetry);

        c.depoCollect();

        Turret turret = new Turret();
        Arm arm = new Arm();

        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addData(">", "INIT DONE");
        telemetry.update();

        /*while(!isStopRequested() && !isStarted()){
            if(gamepad1.dpad_left)
                side++;
            if(side % 1 == 0)
                telemetry.addData("Side", "blue");
            else
                telemetry.addData("Side", "red");
            telemetry.update();
        }
*/

        waitForStart();

        turret.start();
        arm.start();

        if (opModeIsActive()) {

            try {
                runtime.reset();

                while (opModeIsActive() && !isStopRequested()) {

                    //MAKES FAILSAFE FALSE WHEN GAMEPAD IS PRESSED
                    if(gamepad2.dpad_down)
                        Caruso.failSafe = false;

                    //GETS Y ANGULAR VALUE
                    Orientation angles = c.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    double YValue = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle));

                    //TILT LOGIC
                    if(YValue > -79.5 && tiltLogicSwitch){
                        c.startMotors(c.Forward, 0.85);

                        sleep(450);

                        c.stopMotors();
                    }

                    //TURN TILT LOGIC ON AND OFF
                    if(gamepad1.x)
                        tiltLogicSwitch = !tiltLogicSwitch;

                    //MOVEMENT
                    if (gamepad1.right_trigger > 0.01) {
                        // Strafing to the Right
                        c.startMotors(c.RIGHT, 0.8 * Scale(gamepad1.right_trigger));
                    } else if (gamepad1.left_trigger > 0.01) {
                        // Strafing to the Left
                        c.startMotors(c.LEFT, 0.8 * Scale(gamepad1.left_trigger));
                    } else if (gamepad1.y) {
                        c.startMotors(0.9);
                    } else if (gamepad1.a) {
                        c.startMotors(-0.9);
                    } else if(gamepad1.left_bumper){
                        c.RightBack.setPower(0.9);
                        c.RightForward.setPower(0.9);
                        c.LeftForward.setPower(-0.9);
                        c.LeftBack.setPower(-0.9);
                    } else if (gamepad1.right_bumper){
                        c.RightBack.setPower(-0.9);
                        c.RightForward.setPower(-0.9);
                        c.LeftForward.setPower(0.9);
                        c.LeftBack.setPower(0.9);
                    } else if (gamepad1.dpad_up) {
                        c.startMotors(.3);
                    } else if (gamepad1.dpad_down) {
                        c.startMotors(-.3);
                    } else {
                        c.startMotors(-0.75 * Scale(gamepad1.left_stick_y), -0.75 * Scale(gamepad1.right_stick_y), true);
                    }

                    //RUMBLE 20 SEC BEFORE END GAME
                    if(runtime.seconds() > 110)
                        gamepad1.rumble(1000);


                    telemetry.addData("time", runtime.seconds());
                    telemetry.addData("Tilt Logic Active", tiltLogicSwitch);
                    telemetry.addData("Raw Y angle",YValue);
                    telemetry.addData("RAW X Angle", angles.firstAngle);
                    telemetry.addData("RAW Z Angle", angles.thirdAngle);
                    telemetry.addData("Delta Angle", Math.abs(c.getYAngle()));
                    telemetry.addData("Position", c.imu.getPosition());
                    telemetry.addData("Carousal Velocity", c.Carousel.getVelocity(AngleUnit.DEGREES));
                    telemetry.addData("Caroual Encoders", c.Carousel.getCurrentPosition());
                    telemetry.addData("Carousal Power", c.Carousel.getPower());
                    //telemetry.addData("X-axis", c.angles.firstAngle);
                    //telemetry.addData("Y-axis", c.angles.secondAngle);
                    //telemetry.addData("Z-axis", c.angles.thirdAngle);
                    telemetry.addData("Turret Encoders", c.Turber.getCurrentPosition());
                    telemetry.addData("LeftBack Velo", c.LeftBack.getVelocity());
                    telemetry.addData("LeftForward Velo", c.LeftForward.getVelocity());
                    telemetry.addData("RightBack Velo", c.RightBack.getVelocity());
                    telemetry.addData("RightForward Velo", c.RightForward.getVelocity());
                    telemetry.addData("LeftBack Position", c.LeftBack.getCurrentPosition());
                    telemetry.addData("LeftForward Position", c.LeftForward.getCurrentPosition());
                    telemetry.addData("RightForward Position", c.RightForward.getCurrentPosition());
                    telemetry.addData("RightBack Position", c.RightBack.getCurrentPosition());
                    telemetry.addData("Turret Thread", turret.isInterrupted());
                    telemetry.addData("Arm", c.Arm.getVelocity(AngleUnit.DEGREES));
                    telemetry.addData("RightForward", c.RightForward.getPower());
                    telemetry.addData("LeftForward", c.LeftForward.getPower());
                    telemetry.addData("RightBack", c.RightBack.getPower());
                    telemetry.addData("LeftBack", c.LeftBack.getPower());
                    telemetry.addData("Arm Current Position", c.Arm.getCurrentPosition());
                    telemetry.update();

                }
            } catch(Exception e){
                telemetry.addLine("There was an exception");
                telemetry.addLine(e.toString());
                telemetry.update();
            }

            turret.interrupt();
            arm.interrupt();
        }
    }

    private class Turret extends Thread{

        @Override
        public void run() {

            try{

                while(!isInterrupted()){

                    int turretEncoders = c.Turber.getCurrentPosition();

                    if (gamepad2.b)
                        c.depoDrop(depoPosition);
                    if (gamepad2.x)
                        c.depoCollect();
                    if (gamepad2.a)
                        c.depoHold();
                    if(gamepad2.start)
                        c.depoDrop(0.4);

                    if(gamepad2.y){
                        if(c.rlimit.isPressed() && c.lLimit.isPressed()) {
                            c.depoCollect();

                            c.Arm.setPower(0.95);

                            while (!c.armTouch.isPressed() && !gamepad2.dpad_down) {
                                telemetry.addLine(">>GOING DOWN<<");
                                telemetry.addData("armTouch", c.armTouch.isPressed());
                                telemetry.update();
                            }
                            c.Arm.setPower(0);

                            c.Turber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            c.Turber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        }

                        idle = true;
                    }

                    if (gamepad2.left_bumper) {

                        depoPosition = 0.98;

                        c.magMoveArm(3, 120, 680, 0.55, 60);

                        Caruso.failSafe = true;

                        c.moveTurret(c.cclock, 270, 0.35);

                        //Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        c.Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    }

                    if (gamepad2.right_bumper) {

                        depoPosition = 0.8;

                        c.magMoveArm(1, 100, 200, 0.35, 60);

                        Caruso.failSafe = true;

                        c.moveTurret(c.clock, 205, 0.35);

                        //Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        c.Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    }

                    if (gamepad2.dpad_up) {

                        Caruso.failSafe = true;

                        if(turretEncoders < 0)
                            c.centerTurret(c.cclock);
                        else
                            c.centerTurret(c.clock);

                        if(c.rlimit.isPressed() && c.lLimit.isPressed()) {
                            c.depoCollect();

                            c.Arm.setPower(0.95);

                            while (!c.armTouch.isPressed() && !gamepad2.dpad_down) {
                                telemetry.addLine(">>GOING DOWN<<");
                                telemetry.addData("armTouch", c.armTouch.isPressed());
                                telemetry.update();
                            }
                            c.Arm.setPower(0);

                            c.Turber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            c.Turber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        }

                        idle = true;

                    }
                    else {
                        c.Turber.setPower(0.25 * Scale(-gamepad2.left_stick_x));
                    }

                    if (gamepad2.right_trigger > 0.1) {
                        c.Intake.setPower(Scale(gamepad2.right_trigger)*0.85);

                        if(c.rlimit.isPressed() && c.lLimit.isPressed())
                            c.Arm.setPower(0.25);
                    }
                    else if (gamepad2.left_trigger > 0.1) {
                        c.Intake.setPower(-Scale(gamepad2.left_trigger)*0.85);
                        if(c.rlimit.isPressed() && c.lLimit.isPressed())
                            c.Arm.setPower(0.25);
                    }
                    else {
                        c.Intake.setPower(0);
                        c.Arm.setPower(Multiplier * Scale(gamepad2.right_stick_y));
                    }

/*                    if(c.ElementDistance() < 92) {
                        c.stopIntake();
                        c.moveArm(500, 0.35, 350);
                    }
*/
                    if(gamepad2.dpad_left)
                        c.Carousel.setPower(0.19);
                    else
                        c.Carousel.setPower(0);

                }
            } catch (Exception e){
                telemetry.addLine("there is an exception in turret");
                telemetry.addLine(e.toString());
                telemetry.update();

            }

        }

    }

    private class Arm extends Thread{
        @Override
        public void run(){
            try{
                while(!isInterrupted()){
                    if(!true){
                        //c.Arm.setPower(0.13);
                    }else if(true){}
                    else {
                    }
                }
            }catch(Exception e){
                telemetry.addLine("Arm has a exception");
                telemetry.addLine(e.toString());
                telemetry.update();
            }

        }
    }

}