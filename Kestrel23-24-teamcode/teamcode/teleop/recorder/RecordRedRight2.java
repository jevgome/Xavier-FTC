package org.firstinspires.ftc.teamcode.teleop.recorder;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RecordRedRight2", group="Recorder")
public class RecordRedRight2 extends Recorder {
    @Override
    protected String getRecordingName() {
        return "Red Right 2";
    }

    @Override
    protected boolean isRecordingTeleop() {
        return true;
    }
}