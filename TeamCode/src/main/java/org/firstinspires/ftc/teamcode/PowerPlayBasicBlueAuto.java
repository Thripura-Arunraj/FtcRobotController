package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "PowerPlayBasicBlueAuto", group = "auto")
public class PowerPlayBasicBlueAuto extends LinearOpMode {

    Caruso c;

    @Override
    public void runOpMode() throws InterruptedException{

        c = new Caruso(hardwareMap, telemetry);

        c.moveEncoders(c.Forward, .5, 200, 0);
        c.moveConeLift(.5, 200);
        c.moveEncoders(c.BACKWARD, .5, 200, 0);
        c.moveEncoders(c.RIGHT, .5, 200, 0);
        // lift arm
        // turn in place
        // pick up cone
        // go forward and stop with distance sensor
        // turn in place, drop cone
        // turn back to be straight to strafe
        // move to right zone
    }
}