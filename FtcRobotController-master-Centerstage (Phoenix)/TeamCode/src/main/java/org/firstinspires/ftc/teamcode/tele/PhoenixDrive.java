package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Disabled
@TeleOp
public class PhoenixDrive extends LinearOpMode{
    IMU imu;
    DcMotorEx
            m1,m2,m3,m4,
            arm,
            leftLift,pitch,rightLift;

    // Limits:
    // Pitch: 5850
    // Lift: 3550
    int pitchLim = 5400;
    int armLim = 0;
    int liftLim = 3500;

        Servo leftClaw,rightClaw,wrist;
        CRServo claw;
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

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);


        m1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftLift = (DcMotorEx) hardwareMap.dcMotor.get("leftLift");
        rightLift = (DcMotorEx) hardwareMap.dcMotor.get("rightLift");
        pitch = (DcMotorEx) hardwareMap.dcMotor.get("pitch");


        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pitch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pitch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//        pitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        wrist = hardwareMap.servo.get("wrist");

        claw = hardwareMap.crservo.get("asldfjasdkl");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));

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

            // lift.setPower(lift.getCurrentPosition() < liftLim && -gamepad2.left_stick_y >= 0 ? -gamepad2.left_stick_y : lift.getCurrentPosition() > 0 && -gamepad2.left_stick_y <= 0 ? -gamepad2.left_stick_y : 0);
            // pitch.setPower(pitch.getCurrentPosition() < pitchLim && -gamepad2.right_stick_y >= 0 ? -gamepad2.right_stick_y : pitch.getCurrentPosition() > 0 && -gamepad2.right_stick_y <= 0 ? -gamepad2.right_stick_y : 0);

            leftLift.setPower(-gamepad2.left_stick_y);
            rightLift.setPower(-gamepad2.left_stick_y);
            pitch.setPower(-gamepad2.right_stick_y);


            arm.setPower((gamepad2.right_trigger-gamepad2.left_trigger));


            // if(gamepad1.a && gamepad1.b) {
            //     pitch.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            //     pitch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            //     lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            //     lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            // }



            telemetry.addData("Pitch", pitch.getCurrentPosition());
            telemetry.addData("Lift", leftLift.getCurrentPosition());
            telemetry.addData("Perpendicular", m2.getCurrentPosition());
            telemetry.addData("Parallel", m3.getCurrentPosition());
            telemetry.addData("",imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS));
            telemetry.update();
        }
    }
}