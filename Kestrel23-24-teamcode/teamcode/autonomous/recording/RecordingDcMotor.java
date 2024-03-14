package org.firstinspires.ftc.teamcode.autonomous.recording;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * A recording DcMotor acts like a regular DcMotor, but records its changes to the provided autonomous recorder
 * @see DcMotor
 */
public class RecordingDcMotor implements DcMotor, RecordingDevice {
    private JSONArray instructions;
    private final DcMotor dcMotor;
    private final AutonomousRecorder recorder;

    private double lastKnownPower = 0;
    private int lastKnownTargetPosition = 0;
    private ZeroPowerBehavior lastKnownZeroPowerBehavior = null;
    private Direction lastKnownDirection = null;
    private RunMode lastKnownRunMode = null;

    /**
     * Creates a new recording DcMotor and adds it to the provided autonomous recorder
     * @param recorder The autonomous recorder
     * @param dcMotor The DcMotor base
     * @see DcMotor
     */
    public RecordingDcMotor(AutonomousRecorder recorder, DcMotor dcMotor) {
        this.recorder = recorder;
        this.dcMotor = dcMotor;
        instructions = new JSONArray();
        recorder.addRecordingDevice(this);
    }
    @Override
    public JSONObject toJSON() throws JSONException {
        JSONObject obj = new JSONObject();
        obj.put("device_name", AutonomousRecorder.getDeviceHardwareMapName(recorder.getHardwareMap(), dcMotor));
        obj.put("instructions", instructions);
        return obj;
    }

    @Override
    public void resetRecordingData() {
        instructions = new JSONArray();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return dcMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        dcMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return dcMotor.getController();
    }

    @Override
    public int getPortNumber() {
        return dcMotor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
        if (lastKnownZeroPowerBehavior != zeroPowerBehavior && recorder.isRecording()) {
            lastKnownZeroPowerBehavior = zeroPowerBehavior;
            JSONArray array = new JSONArray();
            array.put(getCurrentTime());
            array.put("zero_power_behavior");
            array.put(zeroPowerBehavior.name());
            instructions.put(array);
        }
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return dcMotor.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        dcMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return dcMotor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        dcMotor.setTargetPosition(position);
        if (lastKnownTargetPosition != position && recorder.isRecording()) {
            lastKnownTargetPosition = position;
            JSONArray array = new JSONArray();
            array.put(getCurrentTime());
            array.put("target_position");
            array.put(position);
            instructions.put(array);
        }
    }

    @Override
    public int getTargetPosition() {
        return dcMotor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return dcMotor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return dcMotor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        dcMotor.setMode(mode);
        if (lastKnownRunMode != mode && recorder.isRecording()) {
            lastKnownRunMode = mode;
            JSONArray array = new JSONArray();
            array.put(getCurrentTime());
            array.put("run_mode");
            array.put(mode.name());
            instructions.put(array);
        }
    }

    @Override
    public RunMode getMode() {
        return dcMotor.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        dcMotor.setDirection(direction);
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
        return dcMotor.getDirection();
    }

    @Override
    public void setPower(double power) {
        dcMotor.setPower(power);
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
        return dcMotor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return dcMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return dcMotor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return dcMotor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return dcMotor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        dcMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        dcMotor.close();
    }

    private long getCurrentTime() {
        return System.currentTimeMillis() - recorder.getStartingTime();
    }
}
