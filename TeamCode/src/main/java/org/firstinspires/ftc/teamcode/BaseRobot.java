package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.vision.AprilTagModule;
import org.firstinspires.ftc.teamcode.vision.PropIdentificationVisualPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

/*
    TODO (maybe):
        - Have robot and gamepadManager class
        - In Robot, make manager (pub var) = gamepadManager (gamepadManager  implements gamepadManagerInterface)
        - Make specializedRobot and specializedGamepadManager
            - specializedGamepadManager extends gamepadManager implements gamepadManagerInterface
        - Robots manage their own gamepads, interchangeable-ish
        - All the same interface so can do robot.controller.update() universally
        - Allows differentiation
        - Photos in drive: 24-25 season/Programming/lightning mcqueen


    TODO:
        - IF HATE SOMETHING WRITE IT DOWN TO FIX IT LATER
 */


public class BaseRobot {
    DcMotorEx[] driveMotors;
    protected LinearOpMode opMode;
    private final double wheelDiameter;
    private final double robotDiameter;
    private final IMU imu;
    protected final FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();
    private AprilTagModule aprilTagModule;
    private PropIdentificationVisualPipeline propID;

    public BaseRobot(LinearOpMode opmode, double wheelDiameter, double robotDiameter) {
        this.opMode = opmode;
        this.driveMotors = new DcMotorEx[constants.driveMotorName.values().length];
        this.opMode.telemetry.setMsTransmissionInterval(constants.TELEMETRY_MS_TRANSMISSION_INTERVAL);
        createDriveMotors();

        this.wheelDiameter = wheelDiameter;
        this.robotDiameter = robotDiameter;
        this.imu = createImu();

        initBulkReads();

        writeToTelemetry(">", "Hardware Initialized");
        updateTelemetry();
    }

    protected DcMotorEx createDefaultMotor(String motorName) {
        DcMotorEx motor = this.opMode.hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (motorName.toLowerCase().contains("left")) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        return motor;
    }

    private void createDriveMotors() {
        for (constants.driveMotorName driveMotorName : constants.driveMotorName.values()) {
            DcMotorEx driveMotor = createDefaultMotor(driveMotorName.name());
            this.driveMotors[driveMotorName.ordinal()] = driveMotor;
        }
    }

    public void setDriveMotors(double[] powers, DcMotor.RunMode mode) {
        for (constants.driveMotorName driveMotorName : constants.driveMotorName.values()) {
            this.driveMotors[driveMotorName.ordinal()].setMode(mode);
            this.driveMotors[driveMotorName.ordinal()].setPower(powers[driveMotorName.ordinal()]);
        }
    }

    public void stopDrive() {
        double[] powers = new double[this.driveMotors.length];
        Arrays.fill(powers, 0.0);
        setDriveMotors(powers, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotors(powers, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean isDriving() {
        for (DcMotorEx motor : this.driveMotors) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    private double inchesToEncoder(double inches) {
        return (inches * constants.ENCODER_TICKS / (this.wheelDiameter * Math.PI));
    }

    /**
     * Drive X number of encoder ticks
     *
     * @param powers Array of powers in order of leftFront, leftBack, rightFront, rightBack
     */
    private void driveEncoded(int[] ticks, double[] powers) {
        for (constants.driveMotorName driveMotorName : constants.driveMotorName.values()) {
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

        this.stopDrive();
    }

    /**
     * Encoder-based drive
     *
     * @param inches
     * @param power  [-1.0, 1.0]
     */
    public void driveInches(double inches, double power) {
        int[] target = new int[this.driveMotors.length];
        double[] powers = new double[this.driveMotors.length];
        Arrays.fill(target, (int) this.inchesToEncoder(inches));
        Arrays.fill(powers, power);

        driveEncoded(target, powers);
    }

    /**
     * turns degrees
     *
     * @param degrees Degrees to turn. Positive is to the right, negative to the left
     * @param power   The power to turn at, from [0, 1]
     */
    public void turnDegrees(int degrees, double power) {
        int targetInches = (int) this.inchesToEncoder(Math.toRadians(degrees) * this.robotDiameter);
        int[] target = new int[]{targetInches, targetInches, -targetInches, -targetInches};
        double[] powers = new double[]{power, power, -power, -power};

        driveEncoded(target, powers);
    }

    private IMU createImu() {
        BNO055IMUNew.Parameters imuParameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, // must be adjusted if CH moves
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));

        IMU imu = opMode.hardwareMap.get(IMU.class, "imu");
        boolean worked = imu.initialize(imuParameters);
        writeToTelemetry("IMU Initialized Goodly?", worked);

        return imu;
    }

    /**
     * Enables bulk reads which allow faster hardware call times.
     * Set to auto currently but if speed becomes an issue this can be manually configured.
     */
    private void initBulkReads() {
        List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }


    /**
     * @return double imu angle around the vertical axis (rotation).
     */
    public double getImuAngle() {
        return this.imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
    }

    public void writeToTelemetry(String caption, Object value) {
        this.opMode.telemetry.addData(caption, value);
        packet.put(caption, value);
    }

    public void updateTelemetry() {
        this.opMode.telemetry.update();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    protected OpenCvCamera setUpCamera(String cameraName, int cameraWidth, int cameraHeight, OpenCvCameraRotation orientation) {
        WebcamName cameraNameThing = opMode.hardwareMap.get(WebcamName.class, cameraName);
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(cameraNameThing);
        // This sets up the camera n stuff. Basically just does settings
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);
                webcam.startStreaming(cameraWidth, cameraHeight, orientation);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        return webcam;
    }

    protected Servo setUpServo(String servoName) {
        Servo servo = opMode.hardwareMap.get(Servo.class, servoName);
        return servo;
    }

    /**
     * Sets up a pipeline
     *
     * @param propColor
     * @return a pipeline to be attached to a camera using {@link OpenCvCamera#setPipeline(OpenCvPipeline)}
     */
    public PropIdentificationVisualPipeline initializePropIDPipeline(PropIdentificationVisualPipeline.PropColors propColor) {
        propID = new PropIdentificationVisualPipeline(propColor);
        return propID;
    }

    public AprilTagModule initializeAprilTagScanner(WebcamName camera, int cameraWidthPx, int cameraHeightPx) {
        aprilTagModule = new AprilTagModule(camera, cameraWidthPx, cameraHeightPx);
        return aprilTagModule;
    }

    /**
     * call {@link BaseRobot#initializeAprilTagScanner(WebcamName, int, int) } first or will return null
     *
     * @return null if uninitialized
     */
    public AprilTagModule getAprilTagScanner() {
        return aprilTagModule;
    }

    /**
     * call {@link BaseRobot#initializePropIDPipeline(PropIdentificationVisualPipeline.PropColors)} first or will return null
     *
     * @return null if uninitialized
     */
    public PropIdentificationVisualPipeline getPropIDPipeline() {
        return propID;
    }

    public boolean isDashboardEnabled(){
        return dashboard.isEnabled();
    }
}