package org.firstinspires.ftc.teamcode.auton.OCV;

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
public class VRR_Left_OCV extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(-35,-62,Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence main = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    closeClaw();
                })
                .lineToConstantHeading(new Vector2d(-12,-62))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> {
                    //arm up
                    raiseArm(3150,10000);
                })
                .forward(30)
                .turn(Math.toRadians(-55))
                .forward(5)
                .addTemporalMarker(() -> {
                    //let go
                    openClaw();
                })
                .waitSeconds(1)
                .back(6)
                .turn(Math.toRadians(55))
                .forward(15)
                .splineTo(new Vector2d(-25,-12),Math.toRadians(180))
                .strafeTo(new Vector2d(-55,-12))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    //arm up, get cone from stack
                    raiseArm(200,4000);
                })
                .forward(6)
                .addTemporalMarker(() -> {
                    //grab, lift
                    closeClaw();
                })
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    raiseArm(3150,10000);
                })
                .lineTo(new Vector2d(-12,-12))
                .turn(Math.toRadians(-45))
                .forward(9)
                .addTemporalMarker(() -> {
                    //let go
                    openClaw();
                })
                .waitSeconds(1.2)
                .back(9)
                .turn(Math.toRadians(45))

                .strafeTo(new Vector2d(-55,-12))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    //arm up, get cone from stack
                    raiseArm(200,4000);
                })
                .forward(6)
                .addTemporalMarker(() -> {
                    //grab, lift
                    closeClaw();
                })
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    raiseArm(3150,10000);
                })
                .lineTo(new Vector2d(-12,-12))
                .turn(Math.toRadians(-45))
                .forward(9)
                .addTemporalMarker(() -> {
                    //let go
                    openClaw();
                })
                .waitSeconds(1.2)
                .back(9)
                .turn(Math.toRadians(45))
                .build();

        Trajectory middle = drive.trajectoryBuilder(main.end())
                .forward(24)
                .build();

        Trajectory right = drive.trajectoryBuilder(main.end())
                .forward(44)
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


        waitForStart();

        if(!isStopRequested()) {

            closeClaw();
            raiseArm(3150, 10000);

            two = sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.CENTER;
            three = sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.RIGHT;

            drive.followTrajectorySequence(main);

            if (two) {
                drive.followTrajectory(middle);
            } else if (three) {
                drive.followTrajectory(right);
            }
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