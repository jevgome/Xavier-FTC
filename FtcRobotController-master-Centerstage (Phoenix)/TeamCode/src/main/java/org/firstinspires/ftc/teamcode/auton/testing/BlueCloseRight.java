package org.firstinspires.ftc.teamcode.auton.testing;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.Pipelines.SpikeDetectionBlue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.roadrunnertuning.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.*;

@Autonomous(group = "auto")
public class BlueCloseRight extends LinearOpMode {
    DcMotorEx arm,leftLift,rightLift,pitch;
    Servo claw,wrist,stopper;
    private SpikeDetectionBlue sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    boolean one,two,wait;
    @Override
    public void runOpMode() throws InterruptedException {
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

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
        wrist = hardwareMap.servo.get("wrist");
        stopper = hardwareMap.servo.get("stopper");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12,62,Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence firstLeft = drive.trajectorySequenceBuilder(startPose)
                .back(27)
                .turn(Math.toRadians(-90))
                .forward(19)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    openClaw();
                })
                .waitSeconds(0.4)
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()-> {
                    closeClaw();
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(46,35))
                .UNSTABLE_addTemporalMarkerOffset(-.4, () -> {
                    arm(2200,3000);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    wristDown();
                })
                .waitSeconds(1)
                .strafeLeft(8)
//                .forward(14)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    openClaw();
                })
                .waitSeconds(0.4)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    arm(0,3000);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    wristUp();
                })
//                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(38,10))
                .forward(24)
                .build();

        TrajectorySequence firstMid = drive.trajectorySequenceBuilder(startPose)
                .back(30)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    openClaw();
                })
                .waitSeconds(0.4)
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()-> {
                    closeClaw();
                })
                .waitSeconds(0.4)
                .forward(2)
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(46,35))
                .UNSTABLE_addTemporalMarkerOffset(-.4, () -> {
                    arm(2200,3000);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    wristDown();
                })
                .waitSeconds(1)
//                .strafeLeft(7)
//                .forward(14)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    openClaw();
                })
                .waitSeconds(0.4)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    arm(0,3000);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    wristUp();
                })
//                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(38,10))
                .forward(24)
                .build();

        TrajectorySequence firstRight = drive.trajectorySequenceBuilder(startPose)
                .back(27)
                .turn(Math.toRadians(-90))
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    openClaw();
                })
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    closeClaw();
                })
                .waitSeconds(0.4)
                .lineToConstantHeading(new Vector2d(46,35))
                .UNSTABLE_addTemporalMarkerOffset(-.4, () -> {
                    arm(2200,3000);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    wristDown();
                })
                .waitSeconds(1)
                .strafeRight(8)
//                .forward(14)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    openClaw();
                })
                .waitSeconds(0.4)
                .back(3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    arm(0,3000);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    wristUp();
                })
//                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(38,10))
                .forward(24)
                .build();
        TrajectorySequence waitSeq = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(10)
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
            stopper.setPosition(0.35);
            closeClaw();
            wristUp();
            one = sleeveDetection.getPosition() == SpikeDetectionBlue.SpikePosition.LEFT;
            two = sleeveDetection.getPosition() == SpikeDetectionBlue.SpikePosition.CENTER;
            if(gamepad1.x)wait = true;
            if(gamepad1.b)wait = false;
            telemetry.addData("Wait",wait);
            telemetry.addData("Position", sleeveDetection.getPosition());
            telemetry.update();
        }
        waitForStart();

        if(!isStopRequested()) {
            if(wait)drive.followTrajectorySequence(waitSeq);
            drive.followTrajectorySequence(one ? firstLeft : two ? firstMid : firstRight);
            telemetry.addData("Park",sleeveDetection.getPosition());
            telemetry.addData("Wait",wait);
            telemetry.update();
        }
    }

    public void arm(int pos, double velocity) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(velocity);

        telemetry.addData("Moving Arm to",pos);
        telemetry.update();
    }

    public void lift(int pos, double velocity) {
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

    public void wristUp() {
        wrist.setPosition(0.6);
    }

    public void wristDown() {
        wrist.setPosition(0.0);
    }
}