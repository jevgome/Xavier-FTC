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
public class VRR_Test_OCV extends LinearOpMode {
    DcMotorEx arm;
    Servo claw;


    SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("claw");

        boolean two = false;
        boolean three = false;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35,-60,0);

        TrajectorySequence main = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    closeClaw();
                })
                .lineToConstantHeading(new Vector2d(17,-60))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> {
                    //arm up
                    raiseArm(300,400);
                })
                .splineToSplineHeading(new Pose2d(17,-32,Math.toRadians(55)),Math.toRadians(70))
                .addTemporalMarker(() -> {
                    //let go
                    openClaw();
                })
                .waitSeconds(0.5)
                .back(6)
                .addTemporalMarker(() -> {
                    //arm down
                    raiseArm(0,400);
                })
                .turn(Math.toRadians(35))
                .forward(20)
                .splineToLinearHeading(new Pose2d(25,-12,0),0)
                .strafeTo(new Vector2d(55,-12))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    //arm up, get cone from stack
                    raiseArm(200,400);
                })
                .forward(6)
                .addTemporalMarker(() -> {
                    //grab, lift
                    closeClaw();
                    raiseArm(300,400);
                })
                .waitSeconds(0.5)
                .back(7)
                .turn(Math.toRadians(-120))
                .forward(4)
                .addTemporalMarker(() -> {
                    //let go
                    openClaw();
                })
                .waitSeconds(0.5)
                .back(4)
                .turn(Math.toRadians(120))
                .forward(7)
                .addTemporalMarker(() -> {
                    //grab from stack, lift
                    raiseArm(150,400);
                    closeClaw();
                })
                .waitSeconds(0.5)
                .back(50)
                .turn(Math.toRadians(-42))
                .forward(7)
                .addTemporalMarker(() -> {
                    //let go
                    openClaw();
                })
                .waitSeconds(0.5)
                .back(7)
                .addTemporalMarker(() -> {
                    raiseArm(0,400);
                })
                .turn(Math.toRadians(42))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .forward(24)
                .build();

        Trajectory right = drive.trajectoryBuilder(startPose)
                .forward(48)
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

        while (!isStarted()) {
            if(sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.LEFT) {
                two = false;
                three = false;
            } else if(sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.CENTER) {
                two = true;
                three = false;
            } else if(sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.RIGHT) {
                three = true;
                two = false;
            }
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();

        if(opModeIsActive()) {
            while(opModeIsActive()) {

                drive.followTrajectorySequence(main);
                if(two) {
                    drive.followTrajectory(middle);
                } else if(three) {
                    drive.followTrajectory(right);
                }

            }
        }

    }



    public void closeClaw() {
        claw.setPosition(0);

        telemetry.addData("Claw","closing");
        telemetry.update();
    }
    public void openClaw() {
        claw.setPosition(1);

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