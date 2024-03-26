package org.firstinspires.ftc.teamcode.tele;

//"In order to access static members, it is necessary to qualify references with the class they came from.
//For example, one must say:
//double r = Math.cos(Math.PI * theta);
//In order to get around this, people sometimes put static members into an interface and inherit from that interface.
//This is a bad idea. In fact, it's such a bad idea that there's a name for it: the Constant Interface Antipattern"
// - https://docs.oracle.com/javase/7/docs/technotes/guides/language/static-import.html
//Wow! That's a great point oracle. There's just one problem: I really don't care
//But like seriously though, you know it's bad when even Oracle says not to use something
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@TeleOp(name = "outputmode", group = "whatever")
public class outputmode extends LinearOpMode {
    DcMotor FLeft, FRight, BLeft, BRight, UpperArm, Turret, LowerArm, Wrist;
    Servo Claw;
    TouchSensor ButtonOne, ButtonTwo;

    @Override
    public void runOpMode() {
        FLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BRight = hardwareMap.get(DcMotor.class, "BackRight");
        LowerArm = hardwareMap.get(DcMotor.class, "LowerArm");
        Turret = hardwareMap.get(DcMotor.class, "Turret");
        UpperArm = hardwareMap.get(DcMotor.class, "UpperArm");
        Wrist = hardwareMap.get(DcMotor.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");
        ButtonOne = hardwareMap.touchSensor.get("ButtonOne");
        ButtonTwo = hardwareMap.touchSensor.get("ButtonTwo");

        final int UpperPos = UpperArm.getCurrentPosition();
        final int LowerPos = LowerArm.getCurrentPosition();
        final int TurretPos = Turret.getCurrentPosition();
        final int WristPos = Wrist.getCurrentPosition();
        int mode = 0;
        //target positions for arm presets
        final int UpperArmDown = UpperPos - 2091;
        final int LowerArmDown = LowerPos + 8522;
        final int UpperArmUp = UpperPos - 4396;
        final int LowerArmUp = LowerPos + 4154;

        waitForStart();
        while (opModeIsActive() && (!(gamepad1.x || gamepad2.x))) {
            //mechanum math stuff
            double max;
            double py = -gamepad1.right_stick_x;
            double px = Math.round(gamepad1.left_stick_x);
            double pa = -gamepad1.left_stick_y;
            if (Math.abs(pa) < 0.05) pa = 0;
            double pFLeft = px + py - pa;
            double pFRight = -px + py + pa;
            double pBLeft = -px + py - pa;
            double pBRight = px + py + pa;
            max = Math.max(1.0, Math.max(Math.abs(pFLeft), Math.max(Math.abs(pFRight), Math.max(Math.abs(pBLeft), Math.abs(pBRight)))));
            pFLeft /= max;
            pFRight /= max;
            pBLeft /= max;
            pBRight /= max;
            FLeft.setPower(pFLeft);
            FRight.setPower(pFRight);
            BLeft.setPower(pBLeft);
            BRight.setPower(pBRight);

            //turret and arm controls
            double speed = (gamepad2.b ? 1.6 : 1);
            Turret.setPower((gamepad2.left_trigger > 0 && !ButtonTwo.isPressed()) ? 0.5 * speed : (gamepad2.right_trigger > 0 && !ButtonOne.isPressed()) ? -0.5 * speed : 0);
            Claw.setPosition(gamepad2.right_bumper ? 0.8 : 0.0);
            Wrist.setPower(gamepad1.left_bumper ? 1 : gamepad1.right_bumper ? -1 : 0);

            //currently you cannot manually control motors while preset is being enacted. this limit is for testing and will be removed later
            mode = (mode == 0 && gamepad2.dpad_down ? 1 : mode == 0 && gamepad2.dpad_up ? 2 : (mode == 1 && UpperArm.getCurrentPosition() == UpperArmDown && LowerArm.getCurrentPosition() == LowerArmDown) || (mode == 2 && UpperArm.getCurrentPosition() == UpperArmUp && LowerArm.getCurrentPosition() == LowerArmUp) ? 0 : mode);
            //set motor mode to normal when in mode 0, and run to position otherwise
            LowerArm.setMode(mode == 0 ? RUN_USING_ENCODER : RUN_TO_POSITION);
            UpperArm.setMode(mode == 0 ? RUN_USING_ENCODER : RUN_TO_POSITION);
            //set target position based on what mode we are in, 0 is default since I'm pretty sure it doesn't matter in encoder mode
            LowerArm.setTargetPosition(mode == 1 ? LowerArmDown : mode == 2 ? LowerArmDown : 0);
            UpperArm.setTargetPosition(mode == 1 ? UpperArmDown : mode == 2 ? UpperArmDown : 0);
            //these are positive because apparently the motor will adjust the direction automatically
            UpperArm.setPower(mode == 1 ? 1 : mode == 2 ? 1 : gamepad2.right_stick_y);
            LowerArm.setPower(mode == 1 ? 1 : mode == 2 ? 1 : gamepad2.left_stick_y);

            //print encoder info
            if (gamepad1.a || gamepad2.a) {
                telemetry.addData("Mode", mode);
                telemetry.addData("Upper Arm start", UpperPos);
                telemetry.addData("Upper Arm current", UpperArm.getCurrentPosition());
                telemetry.addData("Upper Arm mode", UpperArm.getMode());
                telemetry.addData("Lower Arm start", LowerPos);
                telemetry.addData("Lower Arm current", LowerArm.getCurrentPosition());
                telemetry.addData("Lower Arm mode", LowerArm.getMode());
                telemetry.addData("Turret", Turret.getCurrentPosition());
                telemetry.addData("ButtonOne", ButtonOne.isPressed());
                telemetry.addData("ButtonTwo", ButtonTwo.isPressed());
                telemetry.addData("Claw", Claw.getPosition());
                telemetry.addData("Wrist", Wrist.getCurrentPosition());
                telemetry.addData("WristPos", WristPos);
                telemetry.update();
            }

            //arm reset
            if (gamepad1.y || gamepad2.y) {
                //moves the turret, then upper arm, then lower arm, then wrist
                while (Turret.getCurrentPosition() != TurretPos && UpperArm.getCurrentPosition() != UpperPos) {
                    Turret.setMode(RUN_TO_POSITION);
                    UpperArm.setMode(RUN_TO_POSITION);
                    LowerArm.setMode(RUN_TO_POSITION);
                    Turret.setTargetPosition(TurretPos);
                    UpperArm.setTargetPosition(UpperPos);
                    LowerArm.setTargetPosition(LowerPos);
                }
                //the while is here so we can set them back to encoder once the motors are in place
                Turret.setMode(RUN_USING_ENCODER);
                UpperArm.setMode(RUN_USING_ENCODER);
                LowerArm.setMode(RUN_USING_ENCODER);
            }
        }
    }
}