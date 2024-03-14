package org.firstinspires.ftc.teamcode.autonomous.recording;

import static org.firstinspires.ftc.teamcode.autonomous.recording.AutonomousRecorder.DEFAULT_DATA_FOLDER;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

/**
 * An autonomous playback loads an autonomous recording, and combines it with a hardware map to playback the saved
 * instructions onto existing motors
 * @see AutonomousRecorder
 */
public class AutonomousPlayback {
    private final HardwareMap hardwareMap;
    private final Telemetry telemtry;
    private final JSONObject json;
    private volatile boolean isPlaying = false;
    // TODO default data folder
    private File dataFolder = DEFAULT_DATA_FOLDER;

    /**
     * Creates a new autonomous playback from the given recording file
     * @param recordingName The name of the autonomous recording
     */
    public AutonomousPlayback(HardwareMap hardwareMap, Telemetry telemtry, String recordingName) {
        this.hardwareMap = hardwareMap;
        this.telemtry = telemtry;

        // Get json object
        try {
            File inputFile = new File(dataFolder, recordingName + ".autorec");
            if (!inputFile.exists()) {
                throw new IllegalArgumentException("Autonomous recording \"" + recordingName + "\" does not exist");
            }

            BufferedReader reader = new BufferedReader(new FileReader(inputFile));
            StringBuilder builder = new StringBuilder();
            while(reader.ready()) {
                builder.append(reader.readLine());
            }
            reader.close();
            json = new JSONObject(builder.toString());
        } catch (IOException | JSONException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Starts the playback for the autonomous recording
     */
    public void startPlayback() {
        isPlaying = true;

        // Get instructions
        Map<HardwareDevice, JSONArray> instructions = new HashMap<>();
        try {
            JSONArray devices = json.getJSONArray("devices");
            for (int i = 0; i < devices.length(); i++) {
                JSONObject obj = devices.getJSONObject(i);
                instructions.put(hardwareMap.get(obj.getString("device_name")), obj.getJSONArray("instructions"));
            }
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }

        // Start playback
        long startTime = System.currentTimeMillis();
        new Timer().schedule(new TimerTask() {public void run()  {
            if (!isPlaying) {
                // Stop motors
                for (HardwareDevice device : instructions.keySet()) {
                    if (device instanceof DcMotorSimple) {
                        DcMotorSimple motorSimple = (DcMotorSimple) device;
                        motorSimple.setPower(0);
                    }
                }

                cancel();
                return;
            }

            if (instructions.isEmpty()) {
                isPlaying = false;
                cancel();
                return;
            }

            // Iterate through each hardware device and run through the next instruction if it is time to
            Iterator<Map.Entry<HardwareDevice, JSONArray>> iterator = instructions.entrySet().iterator();
            while(iterator.hasNext()) {
                Map.Entry<HardwareDevice, JSONArray> entry = iterator.next();
                JSONArray array = entry.getValue();

                // Remove device from map if all instructions are complete
                if (array.length() == 0) {
                    iterator.remove();
                    continue;
                }

                // Test if the next instruction start time matches the current time, if it does then run that instruction
                try {
                    JSONArray instructionArray = array.getJSONArray(0);
                    if (instructionArray.getLong(0) <= System.currentTimeMillis() - startTime) {
                        runInstruction(entry.getKey(), instructionArray.getString(1), instructionArray.get(2));
                        array.remove(0);
                    }
                } catch (JSONException e) {
                    throw new RuntimeException(e);
                }

            }

            telemtry.update();
        }}, 1, 1);
    }

    /**
     * Stops the playback for the autonomous recording
     */
    public void stopPlayback() {
        isPlaying = false;
    }

    /**
     * Sets the location of the data folder that autonomous recordings are loaded from
     * The default folder when a new autonomous playback is created is TODO default folder
     * @param folder The folder to load from
     */
    public void setDataFolder(File folder) {
        dataFolder = folder;
    }

    /**
     * Gets the location of the data folder that autonomous recordings are loaded from
     * The default folder when a new autonomous playback is created is TODO default folder
     * @return The data folder
     */
    public File getDataFolder() {
        return dataFolder;
    }

    /**
     * Runs a single instruction for a motor
     * @param device The device to run the instruction on
     * @param instructionType The instruction type
     * @param instructionValue The value of the instruction
     * @throws JSONException
     */
    private void runInstruction(HardwareDevice device, String instructionType, Object instructionValue) throws JSONException {
        if (device instanceof DcMotor) {
            DcMotor motor = (DcMotor) device;
            switch(instructionType) {
                case "zero_power_behavior":
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.valueOf((String) instructionValue));
                    break;
                case "target_position":
                    motor.setTargetPosition((int) instructionValue);
                    break;
                case "run_mode":
                    motor.setMode(DcMotor.RunMode.valueOf((String) instructionValue));
                    break;
                case "direction":
                    String direction = (String) instructionValue;
                    motor.setDirection(DcMotorSimple.Direction.valueOf(direction));
                    telemtry.addData(AutonomousRecorder.getDeviceHardwareMapName(hardwareMap, device), direction);
                    break;
                case "power":
                    double power = ((Number) instructionValue).doubleValue();
                    motor.setPower(power);
                    telemtry.addData(AutonomousRecorder.getDeviceHardwareMapName(hardwareMap, device), power);
                    break;
                default:
                    throw new IllegalArgumentException("Invalid instruction type: " + instructionType);
            }
        } else if (device instanceof CRServo) {
            CRServo crServo = (CRServo) device;
            switch(instructionType) {
                case "direction":
                    crServo.setDirection(DcMotorSimple.Direction.valueOf((String) instructionValue));
                    break;
                case "power":
                    crServo.setPower(((Number) instructionValue).doubleValue());
                    break;
                default:
                    throw new IllegalArgumentException("Invalid instruction type: " + instructionType);
            }
        } else if (device instanceof Servo) {
            Servo servo = (Servo) device;
            switch(instructionType) {
                case "direction":
                    servo.setDirection(Servo.Direction.valueOf((String) instructionValue));
                    break;
                case "position":
                    servo.setPosition(((Number) instructionValue).doubleValue());
                    break;
                case "scale_range":
                    JSONArray scaleArray = (JSONArray) instructionValue;
                    servo.scaleRange(scaleArray.getDouble(0), scaleArray.getDouble(1));
                    break;
                default:
                    throw new IllegalArgumentException("Invalid instruction type: " + instructionType);
            }
        } else {
            throw new IllegalArgumentException("Invalid instruction motor: " + device.getDeviceName());
        }
    }
}
