package org.firstinspires.ftc.teamcode.autonomous.recording;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * A recording CRServo acts like a regular CRServo, but records its changes to the provided autonomous recorder
 * @see CRServo
 */
public class RecordingCRServo implements CRServo, RecordingDevice {
    private JSONArray instructions;
    private final CRServo crServo;
    private final AutonomousRecorder recorder;

    private double lastKnownPower = 0;
    private Direction lastKnownDirection = null;

    /**
     * Creates a new recording CRServo and adds it to the provided autonomous recorder
     * @param recorder The autonomous recorder
     * @param crServo The CRServo base
     * @see CRServo
     */
    public RecordingCRServo(AutonomousRecorder recorder, CRServo crServo) {
        this.recorder = recorder;
        this.crServo = crServo;
        instructions = new JSONArray();
        recorder.addRecordingDevice(this);
    }

    @Override
    public JSONObject toJSON() throws JSONException {
        JSONObject obj = new JSONObject();
        obj.put("device_name", AutonomousRecorder.getDeviceHardwareMapName(recorder.getHardwareMap(), crServo));
        obj.put("instructions", instructions);
        return obj;
    }

    @Override
    public void resetRecordingData() {
        instructions = new JSONArray();
    }

    @Override
    public ServoController getController() {
        return crServo.getController();
    }

    @Override
    public int getPortNumber() {
        return crServo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        crServo.setDirection(direction);
        if (lastKnownDirection != direction && recorder.isRecording()) {
            lastKnownDirection = direction;
            JSONArray array = new JSONArray();
            array.put(getCurrentTime());
            array.put("direction");
            array.put(direction.name());
            instructions.put(array);
        }
    }

    @Override
    public Direction getDirection() {
        return crServo.getDirection();
    }

    @Override
    public void setPower(double power) {
        crServo.setPower(power);
        if (lastKnownPower != power && recorder.isRecording()) {
            lastKnownPower = power;
            JSONArray array = new JSONArray();
            array.put(getCurrentTime());
            array.put("power");
            try {
                array.put(power);
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            instructions.put(array);
        }
    }

    @Override
    public double getPower() {
        return crServo.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return crServo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return crServo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return crServo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return crServo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        crServo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        crServo.close();
    }

    private long getCurrentTime() {
        return System.currentTimeMillis() - recorder.getStartingTime();
    }
}
