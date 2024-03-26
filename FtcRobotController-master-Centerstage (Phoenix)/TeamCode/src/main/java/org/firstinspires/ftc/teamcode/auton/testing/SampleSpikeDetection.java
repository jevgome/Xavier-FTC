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

@Autonomous(group = "auto")
public class SampleSpikeDetection extends LinearOpMode {

    private SpikeDetectionBlue sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    boolean one,two;
    @Override
    public void runOpMode() throws InterruptedException {

        // OpenCV initialization stuff
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
            // This is where we detect... while the robot is initialized
            one = sleeveDetection.getPosition() == SpikeDetectionBlue.SpikePosition.LEFT;
            two = sleeveDetection.getPosition() == SpikeDetectionBlue.SpikePosition.CENTER;
            telemetry.addData("Position", sleeveDetection.getPosition());
            telemetry.update();
        }
        waitForStart();

        if(!isStopRequested()) {
            // Then go once start is pressed
            telemetry.addData("Park",sleeveDetection.getPosition());
            telemetry.update();
        }
    }
}