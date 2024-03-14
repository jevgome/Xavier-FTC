package org.firstinspires.ftc.teamcode.teleop.recorder;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RecordRedLeft1", group="Recorder")
public class RecordRedLeft1 extends Recorder {
    @Override
    protected String getRecordingName() {
        return "Red Left 1";
    }

    @Override
    protected boolean isRecordingTeleop() {
        return true;
    }
}