package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class KestrelDemoOp extends LinearOpMode{
    DcMotorEx m1,m2,m3,m4, slide,pitch;
    Servo claw;
    public void runOpMode() {
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right");

        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);


        slide = (DcMotorEx) hardwareMap.dcMotor.get("slide");
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pitch = (DcMotorEx) hardwareMap.dcMotor.get("pitch");
        pitch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.servo.get("claw");
 
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




            if(gamepad1.left_bumper) {
                //close
                claw.setPosition(0);
                telemetry.addData("Claw", "closed");
            }

            if(gamepad2.right_bumper) {
                //open
                claw.setPosition(0.4);
                telemetry.addData("Claw", "opened");
            }
            
            pitch.setPower(-gamepad2.left_stick_y);
            slide.setPower(-gamepad2.right_stick_y);
        }
    }
}