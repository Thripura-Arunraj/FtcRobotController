package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "encoderTest", group = "test")
public class encoderTest extends LinearOpMode{

    public DcMotor LeftForward, LeftBack, RightForward, RightBack;

    @Override
    public void runOpMode() throws InterruptedException {

        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        RightForward = hardwareMap.dcMotor.get("RightForward");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightForward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftForward.setDirection(DcMotorSimple.Direction.FORWARD);
        RightForward.setDirection(DcMotorSimple.Direction.FORWARD);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("LB Position", LeftBack.getCurrentPosition());
            telemetry.addData("RB Position", RightBack.getCurrentPosition());
            telemetry.addData("LF Position", LeftForward.getCurrentPosition());
            telemetry.addData("RF Position", RightForward.getCurrentPosition());
            telemetry.update();

        }
    }
}
