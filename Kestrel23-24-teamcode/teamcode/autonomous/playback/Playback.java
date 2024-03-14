package org.firstinspires.ftc.teamcode.autonomous.playback;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.recording.AutonomousPlayback;
import org.firstinspires.ftc.teamcode.camera.SpikeDetection;
import org.firstinspires.ftc.teamcode.camera.SpikeDetectionBlue;
import org.firstinspires.ftc.teamcode.camera.SpikeDetectionRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class Playback extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam 1"), cameraMonitorViewId);

        OpenCvPipeline detection = getRecordingName().startsWith("Blue") ? new SpikeDetectionBlue() : new SpikeDetectionRed();
        camera.setPipeline(detection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        String name = "";
        while(!isStarted()) {
            name = getRecordingName() + ((SpikeDetection) detection).getPosition().getPositionName();
            telemetry.addData("recording:", name);
            telemetry.update();
        }

        AutonomousPlayback playback = new AutonomousPlayback(hardwareMap, telemetry, name);
        playback.startPlayback();
        while(opModeIsActive());
        playback.stopPlayback();
    }

    protected abstract String getRecordingName();
}
