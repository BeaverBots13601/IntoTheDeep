package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.misc.Pose;

import java.util.Arrays;
import java.util.function.BiConsumer;

public class DriveTrain extends HardwareMechanism {
    private DriveMode orientationMode;
    private DcMotorEx[] driveMotors;
    private double referenceAngle;
    private boolean dashboardEnabled;
    private SPEEDS currentSpeedMode = SPEEDS.NORMAL;
    public DriveTrain(HardwareMap hardwareMap, InitData data, BiConsumer<String, Object> telemetryFunc){
        super(hardwareMap, data, telemetryFunc);
        try {
            createDriveMotors(hardwareMap);
        } catch (Exception e) {
            available = false;
            return;
        }

        orientationMode = data.driveMode;
        dashboardEnabled = data.dashboardEnabled;

        if(constants.ROBOT_HEADING != 0){
            referenceAngle = constants.ROBOT_HEADING;
        } else {
            referenceAngle = data.imuAngleRad;
        }

        driveMotors = new DcMotorEx[driveMotorName.values().length];

        available = true;
    }

    public void start(BiConsumer<String, Object> telemetryFunc) {

    }

    public void run(RunData data, BiConsumer<String, Object> telemetryFunc) {
        double speedNow = currentSpeedMode.getNumericalSpeed();

        int tmp_deadzoneadjust = 2;

        float stickX = data.currentGamepadOne.left_stick_x * tmp_deadzoneadjust;
        float stickY = -data.currentGamepadOne.left_stick_y * tmp_deadzoneadjust;
        float stickRotation = data.currentGamepadOne.right_stick_x * tmp_deadzoneadjust;

        double directionRotation = 0;
        if (orientationMode == DriveMode.FIELD) {
            directionRotation = -Pose.normalizeAngle(data.imuAngleRad - referenceAngle);
        }

        Pose rotatedPosition = Pose.rotatePosition(stickX, stickY, directionRotation);
        double rotatedStickX = rotatedPosition.getX();
        double rotatedStickY = rotatedPosition.getY();
        double orientation = data.imuAngleRad;
        telemetryFunc.accept("IMU DATA (rads)", orientation);
        telemetryFunc.accept("Reference Angle (rads)", constants.ROBOT_HEADING);

        double maxPower = Math.max(Math.abs(stickY) + Math.abs(stickX) + Math.abs(stickRotation), 1);

        double leftFrontPower = (rotatedStickY + rotatedStickX + stickRotation) / maxPower * speedNow;
        double leftBackPower = (rotatedStickY - rotatedStickX + stickRotation) / maxPower * speedNow;
        double rightFrontPower = (rotatedStickY - rotatedStickX - stickRotation) / maxPower * speedNow;
        double rightBackPower = (rotatedStickY + rotatedStickX - stickRotation) / maxPower * speedNow;

        telemetryFunc.accept("LeftMotorPower", leftFrontPower);
        telemetryFunc.accept("LeftBackPower", leftBackPower);
        telemetryFunc.accept("RightFrontPower", rightFrontPower);
        telemetryFunc.accept("RightBackPower", rightBackPower);
        telemetryFunc.accept("Current Speed Mode", currentSpeedMode);

        setDriveMotors(new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // speed ctrls (gp 1)
        if (data.currentGamepadOne.dpad_right && !data.previousGamepadOne.dpad_right) {
            currentSpeedMode = SPEEDS.FAST;
        }
        if (data.currentGamepadOne.dpad_up && !data.previousGamepadOne.dpad_up) {
            currentSpeedMode = SPEEDS.NORMAL;
        }
        if (data.currentGamepadOne.dpad_left && !data.previousGamepadOne.dpad_left) {
            currentSpeedMode = SPEEDS.SLOW;
        }
        if (data.currentGamepadOne.dpad_down && !data.previousGamepadOne.dpad_down && dashboardEnabled) {
            currentSpeedMode = SPEEDS.CUSTOM_FTC_DASHBOARD;
        }
    }

    private enum driveMotorName { // expecting to be same for forseeable future
        leftFront, leftBack, rightFront, rightBack
    }

    private enum SPEEDS {
        NORMAL(0.65),
        FAST(.80),
        SLOW(0.4),
        CUSTOM_FTC_DASHBOARD(constants.CUSTOM_FTC_DASHBOARD_SPEED);

        private final double speed;
        public double getNumericalSpeed(){
            return speed;
        }
        SPEEDS(double speed){
            this.speed = speed;
        }
    }

    private void createDriveMotors(HardwareMap hardwareMap) {
        for (driveMotorName driveMotorName : driveMotorName.values()) {
            DcMotorEx driveMotor = createDefaultMotor(hardwareMap, driveMotorName.name());
            driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.driveMotors[driveMotorName.ordinal()] = driveMotor;
        }
    }

    /**
     * Moves with the mecanum wheels a specified number of inches, without turning.
     * @param inches The number of inches to move.
     * @param angle The angle to move at in degrees. 0 would be right, 90 would be forward.
     * @param power The speed to run at from [0, 1]
     * @deprecated Deprecated because driveEncoded() is broken. THIS WILL NOT WORK
     */
    @Deprecated
    public void driveAtAngle(double inches, double angle, double power){
        // technically pose shouldn't be used in base
        Pose move = Pose.rotatePosition(inches, 0, Pose.normalizeAngle(Math.toRadians(angle)));
        double stickRotation = 0; // todo: allow this as argument (to spin while moving)

        double maxPower = Math.max(Math.abs(move.getY()) + Math.abs(move.getX()) + Math.abs(stickRotation), 1);
        double leftFrontPower = (move.getY() + move.getX() + stickRotation) / maxPower;
        double leftBackPower = (move.getY() - move.getX() + stickRotation) / maxPower;
        double rightFrontPower = (move.getY() - move.getX() - stickRotation) / maxPower;
        double rightBackPower = (move.getY() + move.getX() - stickRotation) / maxPower;

        int[] target = new int[]{(int) inchesToEncoder(leftFrontPower * inches), (int) inchesToEncoder(leftBackPower * inches), (int) inchesToEncoder(rightFrontPower * inches), (int) inchesToEncoder(rightBackPower * inches)};
        double[] powers = new double[this.driveMotors.length];
        Arrays.fill(powers, power);

        driveEncoded(target, powers);
    }

    /**
     * Strafe a certain distance. Might be unreliable?
     * @param inches The number of inches to move. + = right, - = left.
     * @param power The speed to move at.
     * @deprecated Deprecated because driveEncoded() is broken. THIS WILL NOT WORK
     */
    @Deprecated
    public void driveStrafe(double inches, double power) {
        int ticks = (int) this.inchesToEncoder(inches);
        int[] target = new int[] {ticks, -ticks, -ticks, ticks};
        double[] powers = new double[] {power, -power, -power, power};

        driveEncoded(target, powers);
    }

    /**
     * turns degrees
     *
     * @param degrees Degrees to turn. Positive is to the right, negative to the left
     * @param power   The power to turn at, from [0, 1]
     * @deprecated Deprecated because driveEncoded() is broken. THIS WILL NOT WORK
     */
    @Deprecated
    public void turnDegrees(int degrees, double power) {
        int targetInches = (int) this.inchesToEncoder(Math.toRadians(degrees) * constants.ROBOT_DIAMETER);
        int[] target = new int[]{targetInches, targetInches, -targetInches, -targetInches};
        double[] powers = new double[]{power, power, -power, -power};

        driveEncoded(target, powers);
    }

    /**
     * Encoder-based drive
     *
     * @param power  [-1.0, 1.0]
     * @deprecated Deprecated because driveEncoded() is broken. THIS WILL NOT WORK
     */
    @Deprecated
    public void driveInches(double inches, double power) {
        int[] target = new int[this.driveMotors.length];
        double[] powers = new double[this.driveMotors.length];
        Arrays.fill(target, (int) this.inchesToEncoder(inches));
        Arrays.fill(powers, power);

        driveEncoded(target, powers);
    }

    /**
     * Drive X number of encoder ticks
     *
     * @param powers Array of powers in order of leftFront, leftBack, rightFront, rightBack
     */
    private void driveEncoded(int[] ticks, double[] powers) {
        // this is broken with the rewrite migration so just doing this for now
        /*for (constants.driveMotorName driveMotorName : constants.driveMotorName.values()) {
            this.driveMotors[driveMotorName.ordinal()].setTargetPosition(ticks[driveMotorName.ordinal()]);
        }

        this.setDriveMotors(powers, DcMotor.RunMode.RUN_TO_POSITION);

        while (this.opMode.opModeIsActive() && this.isDriving()) {
            for (constants.driveMotorName driveMotorName : constants.driveMotorName.values()) {
                writeToTelemetry("Running to", " " + ticks[driveMotorName.ordinal()]);
                writeToTelemetry("Currently at", driveMotorName.name() + " at " + this.driveMotors[driveMotorName.ordinal()].getCurrentPosition());
            }
            updateTelemetry();
        }

        this.stopDrive();*/
    }

    private double inchesToEncoder(double inches) {
        return (inches * constants.ENCODER_TICKS / (constants.WHEEL_DIAMETER * Math.PI));
    }

    private boolean isDriving() {
        for (driveMotorName a : driveMotorName.values()){
            if ((a.name().equals("rightFront")|| a.name().equals("leftBack")) && driveMotors[a.ordinal()].isBusy()){
                return true;
            }
        }
        return false;
    }

    public void stopDrive() {
        double[] powers = new double[this.driveMotors.length];
        Arrays.fill(powers, 0.0);
        setDriveMotors(powers, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotors(powers, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDriveMotors(double[] powers, DcMotor.RunMode mode) {
        for (driveMotorName driveMotorName : driveMotorName.values()) {
            this.driveMotors[driveMotorName.ordinal()].setMode(mode);
            this.driveMotors[driveMotorName.ordinal()].setPower(powers[driveMotorName.ordinal()]);
        }
    }
}
