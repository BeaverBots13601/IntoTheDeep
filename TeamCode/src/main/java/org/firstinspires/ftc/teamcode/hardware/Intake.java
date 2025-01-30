package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;

public class Intake extends HardwareMechanism {
    // magic numbers
    // we use multiple results to reduce false negatives. 3 works well
    private static final int numResultsToUse = 3;
    private static final int CALIBRATED_HORIZONTAL_SLIDE_LENGTH_TICKS = 1950;

    // hardware
    private CRServo leftRotationServo;
    private CRServo rightRotationServo;
    private RevColorSensorV3 colorSensor;
    private Servo wristServo;
    private DcMotorEx horizontalArmMotor;

    // states
    private boolean pickingUp = false;
    private boolean intakeRunning = false;
    private boolean expellingBad = false;
    private List<ColorResult> lastResultsArr;

    // init data
    private TeamColor teamColor;
    private boolean allowBaskets;

    public Intake(HardwareMap hardwareMap, InitData data, BiConsumer<String, Object> telemetryFunc){
        super(hardwareMap, data, telemetryFunc);
        try {
            leftRotationServo = hardwareMap.get(CRServo.class, "leftRotationServo");
            rightRotationServo = hardwareMap.get(CRServo.class, "rightRotationServo");
            colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
            wristServo = setUpServo(hardwareMap, "wristServo");
            horizontalArmMotor = createDefaultMotor(hardwareMap, "horizontalArmMotor");
        } catch(IllegalArgumentException e) {
            available = false; // tag as broken
            return;
        }
        leftRotationServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRotationServo.setPower(0);
        rightRotationServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRotationServo.setPower(0);
        horizontalArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        teamColor = data.teamColor;
        allowBaskets = data.allowBaskets;

        lastResultsArr = new ArrayList<>(numResultsToUse);
        // preload array; values will be expunged shortly
        for (int i = 0; i < numResultsToUse; i++){
            lastResultsArr.add(null);
        }

        // must be here to be legal during init (supposedly)
        setWristPosition(WristPosition.INIT);

        available = true; // tag as working
    }

    public void start() {
        // intake code expects starting high
        setWristPosition(WristPosition.HIGH);
    }

    public void run(RunData data) {
        if (intakeRunning){
            if (pickingUp){
                if (getColorSensorProximity(DistanceUnit.MM) < 25) { // have one
                    ColorSensorColor consensusColor = ColorSensorColor.UNKNOWN;
                    boolean consensusGoodReading = true;
                    for (ColorResult res : lastResultsArr) {
                        if (res == null) { consensusGoodReading = false; break; } // safety case
                        if (consensusColor == ColorSensorColor.UNKNOWN) consensusColor = res.highestColor;
                        // .002 is a bit higher than the field consistently reads.
                        if (res.highestColorValue < .002) { consensusGoodReading = false; break; }
                        // Want all colors to match
                        if (res.highestColor != consensusColor) { consensusGoodReading = false; break; }
                    }

                    if (consensusGoodReading){ // otherwise defer judgement
                        if (
                            (consensusColor == ColorSensorColor.BLUE && teamColor == TeamColor.BLUE) ||
                            (consensusColor == ColorSensorColor.RED && teamColor == TeamColor.RED)   ||
                            (consensusColor == ColorSensorColor.GREEN && allowBaskets)
                        ){
                            // sample we want
                            stopIntake();
                            setWristPosition(WristPosition.HIGH);
                            intakeRunning = false;
                        } else {
                            // wrong color; reject
                            expellingBad = true;
                            pickingUp = false;
                            reverseIntake();
                        }
                    }
                }
            } else {
                if (getColorSensorProximity(DistanceUnit.MM) > 40) {
                    if (!expellingBad){
                        stopIntake();
                        setWristPosition(WristPosition.HIGH);
                        intakeRunning = false;
                    } else {
                        expellingBad = false;
                        pickingUp = true;
                        forwardIntake();
                    }
                }
            }
        }

        // intake automation (gp1)
        if (data.currentGamepadOne.triangle && !data.previousGamepadOne.triangle){
            if (intakeRunning){
                setWristPosition(WristPosition.HIGH);
                // abort case
                stopIntake();
                intakeRunning = false;
                expellingBad = false; // clear state
            } else {
                setWristPosition(WristPosition.LOW);
                if(getColorSensorProximity(DistanceUnit.MM) > 25){ // typically rests ~20mm? away; floor ~44mm
                    // don't have one, so...
                    forwardIntake();
                    intakeRunning = true;
                    pickingUp = true; // save state for later
                } else {
                    // have one that needs to LEAVE
                    reverseIntake();
                    intakeRunning = true;
                    pickingUp = false;
                }
            }
        }

        ColorResult lastResult = processColorSensorResult(getColorSensorColor());
        lastResultsArr.remove(0);
        lastResultsArr.add(lastResult);
        telemetry.accept("Color Sensor Color", lastResult.highestColor);
        telemetry.accept("Color Sensor Strength", lastResult.highestColorValue);

        telemetry.accept("Color Sensor Prox", getColorSensorProximity(DistanceUnit.MM));
        telemetry.accept("Intake On", intakeRunning);
        telemetry.accept("Intake Expelling", expellingBad);
        telemetry.accept("Intake Intaking", pickingUp);

        // horizontal arm (gp 1)
        double val = data.currentGamepadOne.right_trigger - data.currentGamepadOne.left_trigger;
        telemetry.accept("Horizontal Arm Power", val);
        setHorizontalArmPower(val);
    }

    public List<GamepadButtons> getUsedButtons() {
        return Arrays.asList(
                GamepadButtons.GP1_TRIANGLE,
                GamepadButtons.GP1_RIGHT_TRIGGER,
                GamepadButtons.GP1_LEFT_TRIGGER
        );
    }

    public void reverseIntake(){
        leftRotationServo.setPower(1);
        rightRotationServo.setPower(1);
    }
    public void forwardIntake(){
        leftRotationServo.setPower(-1);
        rightRotationServo.setPower(-1);
    }
    public void stopIntake(){
        leftRotationServo.setPower(0);
        rightRotationServo.setPower(0);
    }

    public double getColorSensorProximity(DistanceUnit unit){
        return colorSensor.getDistance(unit);
    }
    public NormalizedRGBA getColorSensorColor(){
        return colorSensor.getNormalizedColors();
    }
    private static ColorResult processColorSensorResult(NormalizedRGBA result){
        ColorResult out = new ColorResult();

        // todo can this logic be bettered?
        if (result.blue > Math.max(result.red, result.green)){
            out.highestColor = ColorSensorColor.BLUE;
            out.highestColorValue = result.blue;
        } else if (result.red > Math.max(result.blue, result.green)){
            out.highestColor = ColorSensorColor.RED;
            out.highestColorValue = result.red;
        } else if (result.green > Math.max(result.blue, result.red)) {
            out.highestColor = ColorSensorColor.GREEN;
            out.highestColorValue = result.green;
        }

        return out;
    }
    private enum ColorSensorColor {
        RED,
        GREEN,
        BLUE,
        UNKNOWN
    }
    static class ColorResult {
        ColorSensorColor highestColor;
        float highestColorValue;
    }

    public enum WristPosition {
        HIGH(0.29),
        //MID(?),
        LOW(.86),
        INIT(1);

        private final double position;

        public double getPosition(){ return position; }

        WristPosition(double pos){ this.position = pos; }
    }
    public void setWristPosition(WristPosition pos){
        wristServo.setPosition(pos.getPosition());
    }

    public Action roadrunnerExtendHorizontalSlideToLength(double dist){
        return new Action() {
            private boolean initialized = false;
            private DcMotor.RunMode before;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized){
                    before = horizontalArmMotor.getMode();

                    horizontalArmMotor.setTargetPosition((int) (CALIBRATED_HORIZONTAL_SLIDE_LENGTH_TICKS * dist));

                    horizontalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    horizontalArmMotor.setPower(1);

                    initialized = true;
                }

                telemetryPacket.put("Motor At", horizontalArmMotor.getCurrentPosition());
                telemetryPacket.put("Motor Moving To", horizontalArmMotor.getTargetPosition());

                if (horizontalArmMotor.isBusy()) return true;

                horizontalArmMotor.setPower(0);

                horizontalArmMotor.setMode(before);

                return false;
            }
        };
    }

    /**
     * Changes the speed of the horizontal arm servo. Bear in mind the maximum extension distance before damage.
     */
    public void setHorizontalArmPower(double speed){
        // todo needs some way to set/limit distance
        horizontalArmMotor.setPower(speed);
    }

    public boolean horizontalArmFarBoundary() { return horizontalArmMotor.getCurrentPosition() > 2000; }
}