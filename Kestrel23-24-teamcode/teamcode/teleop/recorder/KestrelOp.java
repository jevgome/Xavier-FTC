package org.firstinspires.ftc.teamcode.teleop.recorder;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="KestrelOp", group="TeleOp")
public class KestrelOp extends Recorder {
    @Override
    protected String getRecordingName() {
        return "KestrelOp";
    }

    @Override
    protected boolean isRecordingTeleop() {
        return false;
    }
}
