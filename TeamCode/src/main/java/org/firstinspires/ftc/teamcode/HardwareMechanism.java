package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

import java.util.function.BiConsumer;

public abstract class HardwareMechanism {
    public boolean available;
    /**
     * WARNING: After using the constructor, you MUST check the field 'available'. Treat a false value as null.
     *
     * @param data          Starting data for the hardware.
     * @param telemetryFunc The telemetry function, with the parameters Caption, Data.
     */
    public HardwareMechanism(HardwareMap hardwareMap, InitData data, BiConsumer<String, Object> telemetryFunc){}

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

    // static utility members
    protected static Servo setUpServo(HardwareMap hardwareMap, String servoName) {
        Servo servo = hardwareMap.get(Servo.class, servoName);
        return servo;
    }

    /**
     * Creates a default motor with the settings 'RUN_USING_ENCODER' and 'FLOAT on zero power'.
     * Reverses if name includes left.
     */
    protected static DcMotorEx createDefaultMotor(HardwareMap hardwareMap, String motorName) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (motorName.toLowerCase().contains("left")) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        return motor;
    }

    public static class RunData {
        public Gamepad currentGamepadOne;
        public Gamepad currentGamepadTwo;
        public Gamepad previousGamepadOne;
        public Gamepad previousGamepadTwo;
        public double imuAngleRad;
    }

    public static class InitData {
        public TeamColor teamColor;
        public boolean allowBaskets;
        public DriveMode driveMode;
        public double imuAngleRad;
        public boolean dashboardEnabled;
    }

    public enum DriveMode {
        FIELD,
        ROBOT
    }
}
