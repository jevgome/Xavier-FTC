package org.firstinspires.ftc.teamcode.autonomous.recording;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * An autonomous recorder uses a hardware map to record changes for each individual motor<br>
 * When the recording is saved, it is placed in the default data folder.
 * @see AutonomousPlayback
 */
public class AutonomousRecorder {
    private final HardwareMap hardwareMap;
    private List<RecordingDevice> recordingDevices = new ArrayList<>();
    private boolean isRecording = false;
    private long startingTime;
    static final File DEFAULT_DATA_FOLDER = new File(AppUtil.ROOT_FOLDER + "/Recordings/");
    private File dataFolder = DEFAULT_DATA_FOLDER;

    /**
     * Creates a new autonomous recorder
     * @param hardwareMap The hardware map that is used to record and playback the movement
     */
    public AutonomousRecorder(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * Starts a new recording for the hardware map
     * Note: starting a new recording will clear the existing recording data
     */
    public void startRecording() {
        for (RecordingDevice recordingDevice : recordingDevices) {
            recordingDevice.resetRecordingData();
        }
        isRecording = true;
        startingTime = System.currentTimeMillis();
    }

    /**
     * Stops the recording, saving the recording data for playing or exporting
     */
    public void stopRecording() {
        isRecording = false;
    }

    /**
     * Gets the milisecond time the recording started at
     * @return A long
     */
    public long getStartingTime() {
        return startingTime;
    }

    /**
     * @return if this recorder is currently recording or not
     */
    public boolean isRecording() {
        return isRecording;
    }

    /**
     * Gets the hardware map associated with this recorder
     * @return A HardwareMap
     */
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    /**
     * Adds a recording device to this recorder
     * @throws IllegalStateException if this is called while recording
     * @param device The recording device to add
     */
    void addRecordingDevice(RecordingDevice device) {
        if (isRecording)
            throw new IllegalStateException("Cannot add device while recording is in progress.");

        recordingDevices.add(device);
    }

    /**
     * Writes the recording to the output file
     * @param recordingName The name of the recording to be exported<br>
     *                      This will create a new file in the default data storage folder with this name<br>
     *                      If a file with this name already exists, the data is overridden.
     * @throws IllegalStateException if this is called while recording
     */
    public void exportRecording(String recordingName) {
        if (isRecording)
            throw new IllegalStateException("Can not export while recording is in progress.");

        try {
            dataFolder.mkdirs();
            File outputFile = new File(dataFolder, recordingName + ".autorec");
            outputFile.createNewFile();
            FileWriter writer = new FileWriter(outputFile);
            writer.write(getJSON().toString());
            writer.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Gets the location of the data folder that autonomous recordings saved to
     * @return The data folder
     */
    public File getDataFolder() {
        return dataFolder;
    }

    /**
     * Sets the location of the data folder that autonomous recordings saved to
     * @param folder The folder to save to
     */
    public void setDataFolder(File folder) {
        dataFolder = folder;
    }

    /**
     * Gets the JSON representation of this recorder
     * @return A JSONObject
     * @see JSONObject
     */
    private JSONObject getJSON() {
        try {
            JSONObject obj = new JSONObject();
            JSONArray array = new JSONArray();
            for (RecordingDevice device : recordingDevices) {
                array.put(device.toJSON());
            }
            obj.put("devices", array);
            return obj;
        } catch(JSONException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Gets the name of the device associated with it in the provided hardware map
     * @return The device hardware name, or null if device isn't present in the hardware map
     */
    public static String getDeviceHardwareMapName(HardwareMap hardwareMap, HardwareDevice device) {
        // Iterate over each device mapping
        for (HardwareMap.DeviceMapping<? extends HardwareDevice> mapping : hardwareMap.allDeviceMappings) {
            for(Map.Entry<String, ? extends HardwareDevice> entry : mapping.entrySet()) {
                // Test is entry is for the specific device
                if (entry.getValue() == device) {
                    return entry.getKey();
                }
            }
        }
        return null;
    }
}
