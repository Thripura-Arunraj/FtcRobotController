/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Caruso;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MainTeleOp", group="Iterative Opmode")
@Disabled
public class MainTeleOp extends OpMode
{
    private double Multiplier = 0.7;
    public double contPower;
    public ElapsedTime runtime = new ElapsedTime();

    private double Scale(double Input) {
        double Output = Input * Math.abs(Input);
        return Output;
    }

    Caruso c;

    static double depoPosition = 0.98;

    @Override
    public void init() {
        c = new Caruso(hardwareMap, telemetry);

        c.depoCollect();

        Turret turret = new Turret();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

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

                    if (gamepad2.y) {

                        if(c.rlimit.isPressed() && c.lLimit.isPressed()) {
                            c.depoCollect();

                            sleep(400);

                            c.Arm.setPower(0.75);

                            while (!c.armTouch.isPressed() && !gamepad2.x) {
                                telemetry.addLine(">>GOING DOWN<<");
                                telemetry.addData("armTouch", c.armTouch.isPressed());
                                telemetry.update();
                            }
                            c.Arm.setPower(0);
                        }
                    }
                    if (gamepad2.left_bumper) {

                        depoPosition = 0.98;

                        c.Arm.setPower(-0.4);

                        while(c.Arm.getVelocity(AngleUnit.DEGREES) < 80){}

                        c.Arm.setPower(0);

                        c.Turber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        c.Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        c.Arm.setTargetPosition(-1450);
                        c.Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        c.Arm.setPower(0.3);

                        Caruso.failSafe = true;

                        while (!Thread.currentThread().isInterrupted() && Math.abs(c.Arm.getCurrentPosition()) <= Math.abs(c.Arm.getTargetPosition()) && Caruso.failSafe) {

                            if (Math.abs(c.Arm.getCurrentPosition()) > 450)
                                c.depoHold();

                            telemetry.addLine(">>>>GOING UP<<<<<<<");
                            telemetry.addData("Current Position", c.Arm.getCurrentPosition());
                            telemetry.addData("Target Position", c.Arm.getTargetPosition());
                            telemetry.update();
                        }

                        c.Arm.setPower(0);

                        Caruso.failSafe = true;

                        c.moveTurret(c.cclock, 270, 0.35);

                        //Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        c.Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    }

                    if (gamepad2.right_bumper) {

                        depoPosition = 0.76;

                        c.Arm.setPower(-0.4);

                        while(c.Arm.getVelocity(AngleUnit.DEGREES) < 80){}

                        c.Arm.setPower(0);

                        c.Turber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        c.Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        c.Arm.setTargetPosition(-800);
                        c.Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        c.Arm.setPower(0.3);

                        while (!Thread.currentThread().isInterrupted() && Math.abs(c.Arm.getCurrentPosition()) <= Math.abs(c.Arm.getTargetPosition()) && !gamepad2.dpad_down) {

                            if (Math.abs(c.Arm.getCurrentPosition()) > 450)
                                c.depoHold();

                            telemetry.addLine(">>>>GOING UP<<<<<<<");
                            telemetry.addData("Current Position", c.Arm.getCurrentPosition());
                            telemetry.addData("Target Position", c.Arm.getTargetPosition());
                            telemetry.update();
                        }

                        c.Arm.setPower(0);

                        Caruso.failSafe = true;

                        c.moveTurret(c.clock, 150, 0.35);

                        //Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        c.Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }

                    if (gamepad2.dpad_up) {

                        if(turretEncoders < 0)
                            c.centerTurret(c.cclock);
                        else
                            c.centerTurret(c.clock);

                    }
                    else {
                        c.Turber.setPower(0.25 * Scale(-gamepad2.left_stick_x));
                    }

                    if (gamepad2.right_trigger > 0.1)
                        c.Intake.setPower(Scale(gamepad2.right_trigger));

                    else if (gamepad2.left_trigger > 0.1)
                        c.Intake.setPower(-Scale(gamepad2.left_trigger));
                    else
                        c.Intake.setPower(0);

                    if(gamepad2.dpad_left)
                        c.Carousel.setPower(0.25);
                    else
                        c.Carousel.setPower(0);

                    c.Arm.setPower(Multiplier * Scale(gamepad2.right_stick_y));

                }
            } catch (Exception e){
                telemetry.addLine("there is an exception");
                telemetry.addLine(e.toString());
                telemetry.update();
            }

        }
    }

}
