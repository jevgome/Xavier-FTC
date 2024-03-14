package org.firstinspires.ftc.teamcode.teleop.recorder;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RecordBlueLeft2", group="Recorder")
public class RecordBlueLeft2 extends Recorder {
    @Override
    protected String getRecordingName() {
        return "Blue Left 2";
    }

    @Override
    protected boolean isRecordingTeleop() {
        return true;
    }
}