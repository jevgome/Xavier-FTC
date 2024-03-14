package org.firstinspires.ftc.teamcode.teleop.recorder;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RecordBlueRight1", group="Recorder")
public class RecordBlueRight1 extends Recorder {
    @Override
    protected String getRecordingName() {
        return "Blue Right 1";
    }

    @Override
    protected boolean isRecordingTeleop() {
        return true;
    }
}