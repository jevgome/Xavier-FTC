package org.firstinspires.ftc.teamcode.teleop.recorder;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RecordBlueLeft1", group="Recorder")
public class RecordBlueLeft1 extends Recorder {
    @Override
    protected String getRecordingName() {
        return "Blue Left 1";
    }

    @Override
    protected boolean isRecordingTeleop() {
        return true;
    }
}