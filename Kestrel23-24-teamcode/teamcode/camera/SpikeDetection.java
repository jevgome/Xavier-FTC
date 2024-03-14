package org.firstinspires.ftc.teamcode.camera;

public interface SpikeDetection {
    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    SpikePosition getPosition();


    int BOX_SIZE = 40;

    /*
     * An enum to define the skystone position
     */
    enum SpikePosition
    {
        LEFT("L"),
        CENTER("C"),
        RIGHT("R");

        private final String name;

        SpikePosition(String name) {
            this.name = name;
        }

        public String getPositionName() {
            return name;
        }
    }
}
