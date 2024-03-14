package org.firstinspires.ftc.teamcode.teleop.recorder;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RecordBlueRight2", group="Recorder")
public class RecordBlueRight2 extends Recorder {
    @Override
    protected String getRecordingName() {
        return "Blue Right 2";
    }

    @Override
    protected boolean isRecordingTeleop() {
        return true;
    }
}
