package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (group = "Tele")
public class Tele extends LinearOpMode{
    DcMotorEx m1,m2,m3,m4,arm;
    Servo claw;
    int end = 3150;
    double speed = 1.0;
    BNO055IMU imu;
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right");

        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("clawServo");

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
            m1.setPower(p1 * speed);
            m2.setPower(p2 * speed);
            m3.setPower(p3 * speed);
            m4.setPower(p4 * speed);

            if (gamepad1.left_bumper) {
                if (speed == 1.0) {
                    while (gamepad1.left_bumper) {
                        speed = 0.7;
                    }
                } else {
                    while (gamepad1.left_bumper) {
                        speed = 1.0;
                    }
                }
            }


            arm.setVelocity(arm.getCurrentPosition() < end ? -10000*gamepad2.left_stick_y : 0);

//            if (gamepad2.b) {
//                arm.setTargetPosition(0);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setVelocity(10000);
//            }else if (gamepad2.a) {
//                arm.setTargetPosition(600);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setVelocity(10000);
//            }else if (gamepad2.x) {
//                arm.setTargetPosition(2600);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setVelocity(10000);
//            }else if (gamepad2.y) {
//                arm.setTargetPosition(3150);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setVelocity(10000);
//            }else {
//                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }



//            if (gamepad1.a) {
//                arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            }
//            if (gamepad1.b) end = arm.getCurrentPosition();


            if(gamepad2.left_bumper) {
                //close
                claw.setPosition(0);
                telemetry.addData("Claw", "closed");
            }

            if(gamepad2.right_bumper) {
                //open
                claw.setPosition(0.5);
                telemetry.addData("Claw", "opened");
            }


            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.addData("end", end);
            telemetry.addData("claw", claw.getPosition());
            telemetry.update();
        }
    }
}
