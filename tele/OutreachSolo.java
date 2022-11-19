package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@TeleOp(name = "OutreachSolo", group = "Practice")
public class OutreachSolo extends LinearOpMode {
    //drive train-----------------------------
    DcMotorEx frontLeft, backLeft, frontRight, backRight;

    //shooter-------------------------------
    DcMotorEx spin, piston, yaw, pitch;

    //sensors----------------------------------
    TouchSensor tleft, tright, tup, tdown;

    public void runOpMode() {


        //sensors----------------------------------
        tleft = hardwareMap.get(TouchSensor.class, "tleft");
        tright = hardwareMap.get(TouchSensor.class, "tright");
        tup = hardwareMap.get(TouchSensor.class, "tup");
        tdown = hardwareMap.get(TouchSensor.class, "tdown");


        //drive train--------------------------------
        frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        backLeft = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        frontRight = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        backRight = (DcMotorEx) hardwareMap.dcMotor.get("back_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        //shooter------------------------------------
        spin = (DcMotorEx) hardwareMap.dcMotor.get("spin");

        piston = (DcMotorEx) hardwareMap.dcMotor.get("piston");

        yaw = (DcMotorEx) hardwareMap.dcMotor.get("yaw");
        yaw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pitch = (DcMotorEx) hardwareMap.dcMotor.get("pitch");
        pitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pitch.setDirection(DcMotorSimple.Direction.REVERSE);

        double spinSpeed = 0;
        double pistonSpeed = 500;
        double yawSpeed = 0;
        double pitchSpeed = 0;

        waitForStart();

        while(opModeIsActive()) {

            //drive train-------------------------
            double leftDrivePower = -gamepad1.left_stick_y;
            double rightDrivePower= -gamepad1.right_stick_y;
            double slowLeft = leftDrivePower/2;
            double slowRight = rightDrivePower/2;


            if (gamepad1.left_bumper) {
                frontLeft.setPower(slowLeft);
                frontRight.setPower(slowRight);
                backLeft.setPower(slowLeft);
                backRight.setPower(slowRight);
            } else {
                frontLeft.setPower(leftDrivePower);
                frontRight.setPower(rightDrivePower);
                backLeft.setPower(leftDrivePower);
                backRight.setPower(rightDrivePower);
            }


            //shooter--------------------------------
            if((gamepad1.dpad_left && (!tleft.isPressed()))||(gamepad2.dpad_left && (!tleft.isPressed()))) {
                yaw.setVelocity(500);
            } else
            if ((gamepad1.dpad_right && (!tright.isPressed()))||(gamepad2.dpad_right && (!tright.isPressed()))) {
                yaw.setVelocity(-500);
            } else {
                yaw.setVelocity(0);
            }



            if ((gamepad1.dpad_up && (!tup.isPressed()))||(gamepad2.dpad_up && (!tup.isPressed()))) {
                pitch.setVelocity(4000);
            } else
            if ((gamepad1.dpad_down && (!tdown.isPressed()))||(gamepad2.dpad_down && (!tdown.isPressed()))) {
                pitch.setVelocity(-4000);
            } else {
                pitch.setVelocity(0);
            }


            if((gamepad1.x && (spinSpeed < 100))||(gamepad2.x && (spinSpeed < 100))) {
                spinSpeed = spinSpeed + 1;
            }
            if ((gamepad1.b && (spinSpeed > 0))||(gamepad2.b && (spinSpeed > 0))) {
                spinSpeed = spinSpeed - 1;
            }



            if ((gamepad1.right_bumper)||(gamepad1.right_bumper)) {
                spin.setVelocity(spinSpeed);
            } else {
                spin.setVelocity(0);
            }


            if ((gamepad1.a)||(gamepad1.a)) {
                piston.setVelocity(pistonSpeed);
            } else {
                piston.setVelocity(0);
            }







            //Telemetry---------------------
            if (tleft.isPressed()) {
                telemetry.addData("Left limit reached","");
            }
            if (tright.isPressed()) {
                telemetry.addData("Right limit reached","");
            }
            if (tup.isPressed()) {
                telemetry.addData("Up limit reached","");
            }
            if (tdown.isPressed()) {
                telemetry.addData("Dowm limit reached","");
            }


            if (gamepad1.left_bumper||gamepad2.left_bumper) {
                telemetry.addData("Slow mode enabled", "");
            }
            telemetry.addData("speed of flywheel: " + spinSpeed,"");
            telemetry.update();

        }

    }

}
