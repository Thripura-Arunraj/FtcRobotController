package org.firstinspires.ftc.teamcode;

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

@TeleOp(name = "PowerPlayBlueTeleOp", group = "")

public class PowerPlayBlueTeleOp extends LinearOpMode{

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

        telemetry.addData(">", "INIT DONE");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            try {
                runtime.reset();

                while (opModeIsActive() && !isStopRequested()) {

                    /*
                    //MAKES FAILSAFE FALSE WHEN GAMEPAD IS PRESSED
                    Caruso.failSafe = !gamepad2.dpad_down;

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

                    */

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

                    /*
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
                    telemetry.addData("Arm", c.Arm.getVelocity(AngleUnit.DEGREES));
                    telemetry.addData("RightForward", c.RightForward.getPower());
                    telemetry.addData("LeftForward", c.LeftForward.getPower());
                    telemetry.addData("RightBack", c.RightBack.getPower());
                    telemetry.addData("LeftBack", c.LeftBack.getPower());
                    telemetry.addData("Arm Current Position", c.Arm.getCurrentPosition());
                    telemetry.update();
                       */
                }
            } catch(Exception e){
                telemetry.addLine("There was an exception");
                telemetry.addLine(e.toString());
                telemetry.update();
            }
        }
    }
}
