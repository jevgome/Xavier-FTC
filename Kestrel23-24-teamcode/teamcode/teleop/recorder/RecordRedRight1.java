package org.firstinspires.ftc.teamcode.teleop.recorder;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RecordRedRight1", group="Recorder")
public class RecordRedRight1 extends Recorder {
    @Override
    protected String getRecordingName() {
        return "Red Right 1";
    }

    @Override
    protected boolean isRecordingTeleop() {
        return true;
    }
}