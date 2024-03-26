package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Test", group = "test")
public class IntakeTest extends LinearOpMode{
    DcMotorEx
            m1,m2,m3,m4,
            arm,
            lift,pitch,intake;

    int armLim = 0;
    int liftLim = 0;
    int pitchlim = 0;



    //    Servo leftClaw,rightClaw,wrist;
//    TouchSensor ButtonOne, ButtonTwo;
    public void runOpMode() {
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right");

        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lift = (DcMotorEx) hardwareMap.dcMotor.get("lift");
        pitch = (DcMotorEx) hardwareMap.dcMotor.get("pitch");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pitch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        leftClaw = hardwareMap.servo.get("leftClaw");
//        rightClaw = hardwareMap.servo.get("rightClaw");
//        wrist = hardwareMap.servo.get("wrist");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        while (opModeIsActive()) {
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = -gamepad1.right_stick_x;

            double p1 = px + py - pa;
            double p2 = -px + py - pa;
            double p3 = -px + py + pa;
            double p4 = px + py + pa;
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);

//            leftClaw.setPosition(gamepad1.left_bumper ? 0.3 : 0.0);
//            rightClaw.setPosition(gamepad1.right_bumper ? 0.3 : 0.0);
//            wrist.setPosition(gamepad2.y ? 1 : gamepad2.x ? 0.3 : 0.0);

            lift.setPower(-gamepad2.left_stick_y);
            pitch.setPower(-gamepad2.right_stick_y/3);
            arm.setPower(-gamepad1.left_stick_y);
            intake.setPower(gamepad2.right_trigger > 0 ? gamepad2.right_trigger : gamepad2.left_trigger > 0 ? -gamepad2.left_trigger : 0);

            telemetry.addData("Pitch", pitch.getCurrentPosition());
            telemetry.addData("Lift", lift.getCurrentPosition());
            telemetry.update();
        }
    }
}