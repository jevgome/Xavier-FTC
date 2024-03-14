package org.firstinspires.ftc.teamcode.autonomous.recording;

import org.json.JSONException;
import org.json.JSONObject;

public interface RecordingDevice {
    /**
     * Gets the JSON representation of this recording device
     * @return A JSONObject
     * @throws JSONException
     * @see JSONObject
     */
    JSONObject toJSON() throws JSONException;

    /**
     * Resets the recording data for this recording device
     */
    void resetRecordingData();
}
