package org.firstinspires.ftc.teamcode.teleop.recorder;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.recording.AutonomousRecorder;
import org.firstinspires.ftc.teamcode.autonomous.recording.RecordingDcMotor;
import org.firstinspires.ftc.teamcode.autonomous.recording.RecordingServo;

public abstract class Recorder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousRecorder recorder = new AutonomousRecorder(hardwareMap);

        DcMotor m1 = new RecordingDcMotor(recorder, hardwareMap.dcMotor.get("front_left"));
        DcMotor m2 = new RecordingDcMotor(recorder, hardwareMap.dcMotor.get("back_left"));
        DcMotor m3 = new RecordingDcMotor(recorder, hardwareMap.dcMotor.get("front_right"));
        DcMotor m4 = new RecordingDcMotor(recorder, hardwareMap.dcMotor.get("back_right"));
        DcMotor slide = new RecordingDcMotor(recorder, hardwareMap.dcMotor.get("slide"));
        DcMotor pitch = new RecordingDcMotor(recorder, hardwareMap.dcMotor.get("joint"));
        Servo claw = new RecordingServo(recorder, hardwareMap.servo.get("claw"));
        Servo drone = new RecordingServo(recorder, hardwareMap.servo.get("drone"));

        String cameraPosition = "L";

        while(!isStarted()) {
            if (isStopRequested()) {
                recorder.stopRecording();
            }
        }

        if (isRecordingTeleop()) {
            while (!a());

            recorder.startRecording();
        }

        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);
        m4.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        pitch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pitch.setDirection(DcMotorSimple.Direction.REVERSE);
        pitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            double px = -gamepad1.left_stick_x;
            double py = gamepad1.right_stick_x;
            double pa = gamepad1.left_stick_y;
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
            m1.setPower(p1 / 1.5);
            m2.setPower(p2 / 1.5);
            m3.setPower(p3 / 1.5);
            m4.setPower(p4 / 1.5);
            if(gamepad2.right_bumper) {
                //close
                claw.setPosition(0);
                telemetry.addData("Claw", "closed");
            }
            if(gamepad2.left_bumper) {
                //open
                claw.setPosition(0.3);
                telemetry.addData("Claw", "opened");
            }
            if(gamepad2.y) {
                //launch drone
                drone.setPosition(0.5);
                telemetry.addData("Drone", "opened");
            }
            pitch.setPower(gamepad2.right_stick_y);
            slide.setPower(-gamepad2.left_stick_y);
            telemetry.update();

            // Save recording
            if (isRecordingTeleop()) {
                if (x()) {
                    recorder.stopRecording();
                    recorder.exportRecording(getRecordingName() + cameraPosition);
                    stop();
                    break;
                } else if (b()) {
                    recorder.stopRecording();
                    stop();
                    break;
                }
            }

            // Change camera position
            if (dpad_left()) {
                cameraPosition = "L";
            } else if (dpad_up()) {
                cameraPosition = "C";
            } else if (dpad_right()) {
                cameraPosition = "R";
            }
        }
    }

    protected abstract String getRecordingName();

    protected abstract boolean isRecordingTeleop();

    private boolean a() {
        return gamepad1.a || gamepad2.a;
    }

    private boolean b() {
        return gamepad1.b || gamepad2.b;
    }

    private boolean x() {
        return gamepad1.x || gamepad2.x;
    }

    private boolean dpad_right() {
        return gamepad1.dpad_right || gamepad2.dpad_right;
    }

    private boolean dpad_left() {
        return gamepad1.dpad_left || gamepad2.dpad_left;
    }

    private boolean dpad_up() {
        return gamepad1.dpad_up || gamepad2.dpad_up;
    }

    private boolean dpad_down() {
        return gamepad1.dpad_down || gamepad2.dpad_down;
    }
}
