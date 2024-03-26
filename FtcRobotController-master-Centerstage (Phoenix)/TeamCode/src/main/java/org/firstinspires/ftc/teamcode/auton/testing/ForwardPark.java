package org.firstinspires.ftc.teamcode.auton.testing;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.Pipelines.SpikeDetectionBlue;
import org.firstinspires.ftc.teamcode.auton.Pipelines.SpikeDetectionRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.roadrunnertuning.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.*;

@Autonomous(group = "bad auto")
public class ForwardPark extends LinearOpMode {
    DcMotorEx arm,leftLift, rightLift,pitch;
    Servo claw;
    private SpikeDetectionBlue sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    boolean two, three;
    @Override
    public void runOpMode() throws InterruptedException {
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift = (DcMotorEx) hardwareMap.dcMotor.get("leftLift");
        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift = (DcMotorEx) hardwareMap.dcMotor.get("rightLift");
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pitch = (DcMotorEx) hardwareMap.dcMotor.get("pitch");
        pitch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        pitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("claw");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12,62,Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .back(46)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {openClaw();})
                .forward(2)
                .build();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SpikeDetectionBlue();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });


        while(!isStarted()) {
            closeClaw();
            two = sleeveDetection.getPosition() == SpikeDetectionBlue.SpikePosition.CENTER;
            three = sleeveDetection.getPosition() == SpikeDetectionBlue.SpikePosition.RIGHT;
            telemetry.addData("Position", sleeveDetection.getPosition());
            telemetry.update();
        }
        waitForStart();

        if(!isStopRequested()) {
            drive.followTrajectorySequence(left);
//            drive.followTrajectorySequence(seq1);
            telemetry.addData("Park",sleeveDetection.getPosition());
            telemetry.update();
        }
    }

    public void raiseArm(int pos, double velocity) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(velocity);

        telemetry.addData("Moving Arm to",pos);
        telemetry.update();
    }

    public void actuate(int pos, double velocity) {
        leftLift.setTargetPosition(pos);
        rightLift.setTargetPosition(pos);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setVelocity(velocity);
        rightLift.setVelocity(velocity);



        telemetry.addData("Lifting to",pos);
        telemetry.update();
    }

    public void pitch(int pos, double velocity) {
        pitch.setTargetPosition(pos);
        pitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pitch.setVelocity(velocity);

        telemetry.addData("Pitching to",pos);
        telemetry.update();
    }

    public void openClaw() {
        claw.setPosition(0.4);
    }

    public void closeClaw() {
        claw.setPosition(1.0);
    }
}