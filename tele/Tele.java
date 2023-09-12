// Package: Where the code is in the folders
// Will be automatically created for each file
package org.firstinspires.ftc.teamcode.tele;

// Import statements: Imports methods to use from other libraries
// Will also be automatically created, but not all the time
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// @TeleOp tells the program that this is a TeleOp OpMode

// The group name is "Tele"
// This is not necessary, it's just for organization
@TeleOp (group = "Tele")

// Starts the class, or "OpMode"
public class Tele extends LinearOpMode{

    // Initiates the motors, servos, and local variables
    DcMotorEx m1,m2,m3,m4,arm;
    Servo claw;
    int end = 3150;
    double speed = 1.0;
    BNO055IMU imu;

    // runOpMode(): everything in this method is the code that is run once you press "start" on the driver station
    public void runOpMode() {
        // Initiallizes IMU
        // Note: You could just copy/paste this into your code
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);

        // Initializes motors
        // The name in the string corresponds with the name in the config screen in the driver station
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right");

        // Sets what the motors will do when they aren't powered
        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reverses the direction of these motors
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        // Same thing, but for the arm
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initializes servo
        claw = hardwareMap.servo.get("clawServo");

        // waitForStart(): Waits until you press "start" on the driver station
        waitForStart();

        // Everything in this while loop plays once you press the play button on the driver station
        while (opModeIsActive()) {
            // Mecanum drive code
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = -gamepad1.right_stick_x;
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
            m1.setPower(p1 * speed);
            m2.setPower(p2 * speed);
            m3.setPower(p3 * speed);
            m4.setPower(p4 * speed);

            // If player one presses the left bumper, slow the robot down
            if (gamepad1.left_bumper) {
                if (speed == 1.0) {
                    while (gamepad1.left_bumper) {
                        speed = 0.7;
                    }
                } else {
                    while (gamepad1.left_bumper) {
                        speed = 1.0;
                    }
                }
            }


            // Control the arm with the left joystick of player 2
            arm.setVelocity(arm.getCurrentPosition() < end ? -10000*gamepad2.left_stick_y : 0);

            // Close the claw servo with the left bumper of player 2
            if(gamepad2.left_bumper) {
                //close
                claw.setPosition(0);
                telemetry.addData("Claw", "closed");
            }

            // Open the claw servo with the left bumper of player 2
            if(gamepad2.right_bumper) {
                //open
                claw.setPosition(0.5);
                telemetry.addData("Claw", "opened");
            }


            //Telemetry: Information that is printed onto the driver station
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.addData("end", end);
            telemetry.addData("claw", claw.getPosition());
            telemetry.update();
        }
    }
}
