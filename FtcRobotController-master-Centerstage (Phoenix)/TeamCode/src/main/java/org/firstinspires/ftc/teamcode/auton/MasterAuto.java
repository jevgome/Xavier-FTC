package org.firstinspires.ftc.teamcode.auton;

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

@Autonomous(group = "!auto")
public class MasterAuto extends LinearOpMode {
    DcMotorEx arm,leftLift,rightLift,pitch;
    Servo claw,wrist,stopper;

    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    boolean one,two,wait,blue,close,left;
    boolean parkBool = true;

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


        Pose2d blueClose = new Pose2d(12,62,Math.toRadians(90));
        Pose2d blueFar = new Pose2d(-36,62,Math.toRadians(90));
        Pose2d redClose = new Pose2d(12,-62,Math.toRadians(-90));
        Pose2d redFar = new Pose2d(-36,-62,Math.toRadians(-90));
        Pose2d startPose;



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        SpikeDetectionRed redDetection = new SpikeDetectionRed();
        SpikeDetectionBlue blueDetection = new SpikeDetectionBlue();
        camera.setPipeline(blueDetection);

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
            if(gamepad1.dpad_left) {
                blue = true;
                camera.setPipeline(blueDetection);
            }
            if(gamepad1.dpad_right) {
                blue = false;
                camera.setPipeline(redDetection);
            }
            camera.setPipeline(blue ? blueDetection : redDetection);
            if(gamepad1.dpad_up) close = true;
            if(gamepad1.dpad_down) close = false;
            if(gamepad1.left_bumper) left = true;
            if(gamepad1.right_bumper) left = false;
            closeClaw();
            wristUp();
            if(gamepad1.x)wait = true;
            if(gamepad1.b)wait = false;
            if(gamepad1.a)parkBool = false;
            if(gamepad1.y)parkBool = true;
            stopper.setPosition(0.35);


            one = blue ? blueDetection.getPosition()==SpikeDetectionBlue.SpikePosition.LEFT : redDetection.getPosition() == SpikeDetectionRed.SpikePosition.LEFT;
            two = blue ? blueDetection.getPosition()==SpikeDetectionBlue.SpikePosition.CENTER : redDetection.getPosition() == SpikeDetectionRed.SpikePosition.CENTER;

            telemetry.addData("Color (blue: leftpad, red: rightpad)",blue ? "Blue" : "Red");
            telemetry.addData("Starting Position (close: uppad, far: downpad",close ? "Close" : "Far");
            telemetry.addData("Park (left: left bumper, right: right bumper",left ? "Left" : "Right");
            telemetry.addData("Wait (no wait: b, wait: x)",wait);
            telemetry.addData("Park? (no park: a, park: y)",parkBool);

            telemetry.addData("Zone",blue ? blueDetection.getPosition() : redDetection.getPosition());

            telemetry.update();
        }
        waitForStart();

        if(!isStopRequested()) {

            startPose = blue&&close ? blueClose : blue ? blueFar : close ? redClose : redFar;
            drive.setPoseEstimate(startPose);

            TrajectorySequence waitSeq = drive.trajectorySequenceBuilder(startPose)
                    .waitSeconds(close ? 7 : 2)
                    .build();

            TrajectorySequence spikeClose = drive.trajectorySequenceBuilder(startPose)
                    .back(27)
                    .turn(Math.toRadians(90 * (blue ? -1 : 1)))
                    .forward(19)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, this::openClaw)
                    .waitSeconds(0.4)
                    .forward(3)
                    .UNSTABLE_addTemporalMarkerOffset(0, this::closeClaw)
                    .waitSeconds(0.4)
                    .forward(4)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::openClaw)
                    .waitSeconds(0.4)
                    .back(4)
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, this::closeClaw)
                    .build();

            TrajectorySequence spikeMid = drive.trajectorySequenceBuilder(startPose)
                    .back(30)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, this::openClaw)
                    .waitSeconds(0.4)
                    .forward(3)
                    .UNSTABLE_addTemporalMarkerOffset(0, this::closeClaw)
                    .waitSeconds(0.4)
                    .forward(4)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::openClaw)
                    .waitSeconds(0.4)
                    .back(4)
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, this::closeClaw)
                    .forward(4)
                    .turn(Math.toRadians(90 * (blue?-1:1)))
                    .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(),blue ? 37 : -37))
                    .build();

            TrajectorySequence spikeFar = drive.trajectorySequenceBuilder(startPose)
                    .back(27)
                    .turn(Math.toRadians(90 * (blue?-1:1)))
                    .back(3)
                    .UNSTABLE_addTemporalMarkerOffset(0, this::openClaw)
                    .forward(3)
                    .UNSTABLE_addTemporalMarkerOffset(0, this::closeClaw)
                    .waitSeconds(0.4)
                    .forward(4)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::openClaw)
                    .waitSeconds(0.4)
                    .back(4)
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, this::closeClaw)
                    .build();

            TrajectorySequence spikeClose2 = drive.trajectorySequenceBuilder(startPose)
                    .back(27)
                    .turn(Math.toRadians(90 * (blue?1:-1)))
                    .back(3)
                    .UNSTABLE_addTemporalMarkerOffset(0, this::openClaw)
                    .forward(3)
                    .UNSTABLE_addTemporalMarkerOffset(0, this::closeClaw)
                    .waitSeconds(0.4)
                    .forward(4)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::openClaw)
                    .waitSeconds(0.4)
                    .back(4)
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, this::closeClaw)
                    .forward(3)
                    .build();

            TrajectorySequence spikeMid2 = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(drive.getPoseEstimate().getX(),(blue ? 10 : -10)))
                    .turn(Math.toRadians(180))
                    .back(7)
                    .UNSTABLE_addTemporalMarkerOffset(0, this::openClaw)
                    .forward(3)
                    .UNSTABLE_addTemporalMarkerOffset(0, this::closeClaw)
                    .waitSeconds(0.4)
                    .forward(4)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::openClaw)
                    .waitSeconds(0.4)
                    .back(4)
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, this::closeClaw)
                    .build();

            TrajectorySequence spikeFar2 = drive.trajectorySequenceBuilder(startPose)
                    .back(27)
                    .turn(Math.toRadians(90 * (blue?-1:1)))
                    .back(3)
                    .UNSTABLE_addTemporalMarkerOffset(0, this::openClaw)
                    .forward(3)
                    .UNSTABLE_addTemporalMarkerOffset(0, this::closeClaw)
                    .waitSeconds(0.4)
                    .forward(4)
                    .UNSTABLE_addTemporalMarkerOffset(0,this::openClaw)
                    .waitSeconds(0.4)
                    .back(4)
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, this::closeClaw)
                    .forward(3)
                    .build();

            if(wait)drive.followTrajectorySequence(waitSeq);

            drive.followTrajectorySequence(blue&&one || !(blue||(one||two)) ? (close ? spikeClose : spikeClose2) : two ? (close ? spikeMid : spikeMid2) : (close ? spikeFar : spikeFar2));

            TrajectorySequence door = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(drive.getPoseEstimate().getX(),7))
                    .lineToConstantHeading(new Vector2d(35,7))
                    .build();

            if(!close) drive.followTrajectorySequence(door);
            TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(38,36 * (blue ? 1 : -1), Math.toRadians(0)))
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> arm(2225,3000))
                    .UNSTABLE_addTemporalMarkerOffset(-0.8, this::wristDown)
                    .waitSeconds(1.7)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        openClaw();
                    })
                    .waitSeconds(0.6)
                    .lineToConstantHeading(new Vector2d(two&&!close ? 47 : 45,36 * (blue ? 1 : -1) + (one ? 7 : two ? 0 : -7)))
                    .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> arm(2300,3000))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        stopper.setPosition(0);
                    })
                    .waitSeconds(0.6)
                    .back(3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0,() -> arm(0,3000))
                    .UNSTABLE_addTemporalMarkerOffset(0.2, this::wristUp)
                    .build();


            drive.followTrajectorySequence(toBackdrop);

            TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(38,blue&&left ? 54 : blue ? 16 : left ? -16 : -54))
                    .splineToConstantHeading(new Vector2d(38+24,blue&&left ? 60 : blue ? 10 : left ? -10 : -60),0)
                    .build();
            if(parkBool)drive.followTrajectorySequence(park);

            telemetry.addData("Color",blue ? "Blue" : "Red");
            telemetry.addData("Starting Position",close ? "Close" : "Far");
            telemetry.addData("Park",left ? "Left" : "Right");
            telemetry.addData("Wait",wait);

            telemetry.addData("Zone",blue ? blueDetection.getPosition() : redDetection.getPosition());
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