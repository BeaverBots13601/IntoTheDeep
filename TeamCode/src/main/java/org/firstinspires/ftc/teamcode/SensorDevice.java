package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor;

import java.util.function.BiConsumer;

/**
 * A SensorDevice is a general class to handle sensors which are of interest to multiple classes. Unlike HardwareMechanisms, SensorDevices must be manually instantiated.
 * <p>
 * Take note that sensors which are only of interest to a single mechanism should instead be handled within the corresponding HardwareMechanism.
 * @param <T> The type which the sensor will return when polled.
 */
public abstract class SensorDevice<T> {
    public boolean available;
    protected BiConsumer<String, Object> telemetry;

    public SensorDevice(BiConsumer<String, Object> telemetryFunc){
        telemetry = telemetryFunc;
    }

    /**
     * Call after doing waitForStart(). Allows the class to do setup that can only legally be done after starting.
     */
    abstract public void start();

    abstract public T poll();

    public static class SensorInitData {
        public TeamColor teamColor;
        public boolean dashboardEnabled;
    }
}
