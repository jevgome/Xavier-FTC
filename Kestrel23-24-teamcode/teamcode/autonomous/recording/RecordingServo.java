package org.firstinspires.ftc.teamcode.autonomous.recording;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * A recording servo acts like a regular servo, but records its changes to the provided autonomous recorder
 * @see Servo
 */
public class RecordingServo implements Servo, RecordingDevice {
    private JSONArray instructions;
    private final Servo servo;
    private final AutonomousRecorder recorder;

    private double lastKnownPosition = Double.NaN;
    private double lastKnownScaleMin = 0, lastKnownScaleMax = 1;
    private Direction lastKnownDirection = null;

    /**
     * Creates a new recording servo and adds it to the provided autonomous recorder
     * @param recorder The autonomous recorder
     * @param servo The servo base
     * @see com.qualcomm.robotcore.hardware.CRServo
     */
    public RecordingServo(AutonomousRecorder recorder, Servo servo) {
        this.recorder = recorder;
        this.servo = servo;
        instructions = new JSONArray();
        recorder.addRecordingDevice(this);
    }

    @Override
    public JSONObject toJSON() throws JSONException {
        JSONObject obj = new JSONObject();
        obj.put("device_name", AutonomousRecorder.getDeviceHardwareMapName(recorder.getHardwareMap(), servo));
        obj.put("instructions", instructions);
        return obj;
    }

    @Override
    public void resetRecordingData() {
        instructions = new JSONArray();
    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(direction);
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
        return servo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        servo.setPosition(position);
        if (lastKnownPosition != position && recorder.isRecording()) {
            lastKnownPosition = position;
            JSONArray array = new JSONArray();
            array.put(getCurrentTime());
            array.put("position");
            try {
                array.put(position);
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            instructions.put(array);
        }
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
        if ((lastKnownScaleMin != min || lastKnownScaleMax != max) && recorder.isRecording()) {
            lastKnownScaleMin = min;
            lastKnownScaleMax = max;
            JSONArray array = new JSONArray();
            array.put(getCurrentTime());
            array.put("scale_range");
            try {
                JSONArray scaleArray = new JSONArray();
                scaleArray.put(min);
                scaleArray.put(max);
                array.put(scaleArray);
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
            instructions.put(array);
        }
    }

    @Override
    public Manufacturer getManufacturer() {
        return servo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return servo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return servo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        servo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        servo.close();
    }

    private long getCurrentTime() {
        return System.currentTimeMillis() - recorder.getStartingTime();
    }
}
