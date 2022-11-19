package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name = "Tele", group = "Tele")
public class Tele extends LinearOpMode {
    DcMotorEx m1,m2,m3,m4
//            , m5
            ;
//    Servo claw;

    public void runOpMode(){
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left_motor");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("front_right_motor");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("back_left_motor");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right_motor");
//        m5 = (DcMotorEx) hardwareMap.dcMotor.get("arm_motor");

        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        m5.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        m5.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        m5.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        claw = hardwareMap.servo.get("clawServo");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);

        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = -gamepad1.right_stick_x;

            if (Math.abs(pa) < 0.05) pa = 0;
            double p1 = -px + py - pa;
            double p2 = px + py + -pa;
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


//            if (gamepad2.dpad_up) {
//                m5.setTargetPostion(300);
//                m5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                m5.setVelocity(400);
//            }
//            if (gamepad2.dpad_down) {
//                m5.setTargetPostion(0);
//                m5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                m5.setVelocity(400);
//            }
//            if (gamepad2.dpad_left) {
//                m5.setTargetPostion(150);
//                m5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                m5.setVelocity(400);
//            }
//            if (gamepad2.dpad_right) {
//                m5.setTargetPostion(100);
//                m5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                m5.setVelocity(400);
//            }

//            if(gamepad2.x) {
//                claw.setPosition(1);
//            }
//            if(gamepad2.b) {
//                claw.setPosition(0);
//            }

//            if(gamepad2.left_stick_button && gamepad2.right_stick_button) {
//                m5.setPower(-0.1);
//                m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }


            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);

            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.update();
        }
//        m1.setPower(0);
//        m2.setPower(0);
//        m3.setPower(0);
//        m4.setPower(0);
    }
}
