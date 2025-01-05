package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.BiConsumer;

public abstract class HardwareMechanism {
    @Nullable
    protected static HardwareMechanism instance = null;

    protected HardwareMechanism(){}

    /**
     * Must be called prior to calling getInstance.
     *
     * @param data          Starting data for the hardware.
     * @param telemetryFunc The telemetry function, with the parameters Caption, Data.
     */
    abstract public void init(HardwareMap hardwareMap, InitData data, BiConsumer<String, Object> telemetryFunc);
    /**
     * Call after doing waitForStart(). Allows the class to do setup that can only legally be done after starting.
     * @param telemetryFunc The telemetry function, with the parameters Caption, Data.
     */
    abstract public void start(BiConsumer<String, Object> telemetryFunc);

    /**
     * The main loop function, meant to be called every TeleOp loop.
     * @param data Relevant data for the class.
     * @param telemetryFunc The telemetry function, with the parameters Caption, Data.
     */
    abstract public void run(RunData data, BiConsumer<String, Object> telemetryFunc);

    /**
     * Gets the available instance.
     * @return Null if the hardware is unavailable or has not yet been initialized; otherwise returns an instance of the class.
     */
    @Nullable
    public static HardwareMechanism getInstance(){
        // note: we can't just run the init function and give back the result because we don't know
        // the init data
        return instance;
    }

    // static utility members
    protected static Servo setUpServo(HardwareMap hardwareMap, String servoName) {
        Servo servo = hardwareMap.get(Servo.class, servoName);
        return servo;
    }

    public static class RunData {
        public Gamepad currentGamepadOne;
        public Gamepad currentGamepadTwo;
        public Gamepad previousGamepadOne;
        public Gamepad previousGamepadTwo;
    }

    public static class InitData {
        public TeamColor teamColor;
        public boolean allowBaskets;
    }
}
