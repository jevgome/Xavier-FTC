package org.firstinspires.ftc.teamcode.auton.TFOD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.roadrunnertuning.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.hardware.*;

import java.util.List;
@Disabled
@Autonomous(name = "VRR Test", group = "Test")
public class VRR_Test_TFOD extends LinearOpMode {

    DcMotorEx arm;
    Servo claw;

    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/PowerPlay.tflite";
    private static final String[] LABELS = {
            "1",
            "2",
            "3"
    };


    private static final String VUFORIA_KEY =
            "AUhWSWT/////AAABmUC6ftZWTkXIkAhi8kSayqRoOtqGil/AxZ02gFIzhKArupgHxJXcyUTS9XI+Ct83WL7lhJ1NGyHB2kBAsFCOzJVfxDBLDQgpFwWn6uX1kpjD3TgEKu2bz294ayXe1hzEdNUvWR4ROEl+dLwaM63JG/WxHuLMU4WJLrpzhdP/L+rwFPxl3XGdtAKQlvkmmtM+DU4xtT12CetmZgekxHNQser/dbEaGZAbhP99iixm/wERGjmblnWvOZ353dkDCp8bz495edNKRvE+81iYiYe0IqOP/rp8IgUCgr0yK9ZxIfrhY9hJza+V98g5fblDGd9PJ39wNafgvdzyOJ4ocSbrBaRnLOez8wUngul+XOMgJGCr";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;




    @Override
    public void runOpMode() {

        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("claw");

        boolean two = false;
        boolean three = false;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }


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


        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                    double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                    telemetry.addData(""," ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                    telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                    if (recognition.getLabel().equals("1")) {
                        telemetry.addData("Image", "1");


                    } else if (recognition.getLabel().equals("2")) {
                        telemetry.addData("Image", "2");
                        two = true;


                    } else if (recognition.getLabel().equals("3")) {
                        telemetry.addData("Image", "3");
                        three = true;


                    } else {
                        telemetry.addData("no item detected", "");
                        drive.followTrajectorySequence(main);

                    }
                }
                telemetry.update();
            }
        }
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                drive.followTrajectorySequence(main);
                if (two) {
                    drive.followTrajectory(middle);
                }
                if(three) {
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


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}