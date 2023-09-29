package org.firstinspires.ftc.teamcode.auton.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.Pipelines.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.roadrunnertuning.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.*;

@Autonomous(group = "auto")
public class LowTest extends LinearOpMode {
    DcMotorEx arm;
    Servo claw;
    private SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("clawServo");

        boolean two = false;
        boolean three = false;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35,-62,Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35,-12))
                .UNSTABLE_addTemporalMarkerOffset(-3.5,() -> raiseArm(1700,10000))
                .turn(Math.toRadians(-130))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .turn(Math.toRadians(40))

                .forward(25)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> raiseArm(650,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(23)
                .turn(Math.toRadians(-38.5))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3)
                .turn(Math.toRadians(38.5))

                .forward(29)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> raiseArm(500,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(24)
                .turn(Math.toRadians(-40))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3)
                .turn(Math.toRadians(40))

                .forward(29)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> raiseArm(400,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(24)
                .turn(Math.toRadians(-40))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3)
                .turn(Math.toRadians(40))
                .back(20)
                .build();

        TrajectorySequence mid = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35,-12))
                .UNSTABLE_addTemporalMarkerOffset(-3.5,() -> raiseArm(1700,10000))
                .turn(Math.toRadians(-130))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .turn(Math.toRadians(40))

                .forward(25)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> raiseArm(650,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(23)
                .turn(Math.toRadians(-38.5))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3)
                .turn(Math.toRadians(38.5))

                .forward(29)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> raiseArm(500,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(24)
                .turn(Math.toRadians(-40))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3)
                .turn(Math.toRadians(40))

                .forward(29)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> raiseArm(400,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(24)
                .turn(Math.toRadians(-40))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3)
                .turn(Math.toRadians(40))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35,-12))
                .UNSTABLE_addTemporalMarkerOffset(-3.5,() -> raiseArm(1700,10000))
                .turn(Math.toRadians(-130))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .turn(Math.toRadians(40))

                .forward(25)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> raiseArm(650,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(23)
                .turn(Math.toRadians(-38.5))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3)
                .turn(Math.toRadians(38.5))

                .forward(29)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> raiseArm(500,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(24)
                .turn(Math.toRadians(-40))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3)
                .turn(Math.toRadians(40))

                .forward(29)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> raiseArm(400,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> closeClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> raiseArm(1700,10000))
                .back(24)
                .turn(Math.toRadians(-40))
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> raiseArm(10,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(3)
                .turn(Math.toRadians(40))
                .forward(22)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
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

        closeClaw();
        while(!isStarted()) {
            two = sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.CENTER;
            three = sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.RIGHT;
            telemetry.addData("Position", sleeveDetection.getPosition());
            telemetry.update();
        }
        waitForStart();

        if(!isStopRequested()) {
            drive.followTrajectorySequence(two ? mid : three ? right : left);
            telemetry.addData("Park",sleeveDetection.getPosition());
            telemetry.update();
        }
    }

    public void closeClaw() {
        claw.setPosition(0);

        telemetry.addData("Claw","closing");
        telemetry.update();
    }
    public void openClaw() {
        claw.setPosition(0.5);

        telemetry.addData("Claw","Opening");
        telemetry.update();
    }
    public void raiseArm(int pos, double velocity) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(velocity);

        telemetry.addData("Moving Arm to",pos);
        telemetry.update();
    }
}
