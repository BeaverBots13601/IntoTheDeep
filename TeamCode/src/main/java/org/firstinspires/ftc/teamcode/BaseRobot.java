package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vision.AprilTagData;
import org.firstinspires.ftc.teamcode.vision.AprilTagModule;
import org.firstinspires.ftc.teamcode.vision.PropIdentificationVisualPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
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

    TODO: Build web-tool that allows robot configuration i.e driver station (ftc-dash)

    TODO:
        - IF HATE SOMETHING WRITE IT DOWN TO FIX IT LATER
 */


public class BaseRobot {
    protected LinearOpMode opMode;
    private final IMU imu;
    protected final FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();
    private AprilTagModule aprilTagModule;
    private PropIdentificationVisualPipeline propID;
    @Nullable
    private DigitalChannel switch_ = null;

    private final Limelight3A limelight = null;

    public ArrayList<AprilTagData> getLastLimelightAprilTags(){
        ArrayList<AprilTagData> out = new ArrayList<>();

        limelight.getLatestResult().getFiducialResults().forEach((LLResultTypes.FiducialResult a) -> out.add(new AprilTagData(a.getFiducialId(), a.getTargetPoseRobotSpace().getPosition().z, 0)));

        return out;
    }

    // todo this limelight stuff shouldn't be here, move back
    public List<LLResultTypes.FiducialResult> getLastLimelightAprilTagsRaw(){
        return limelight.getLatestResult().getFiducialResults();
    }

    public void updateLimelightIMUData(){
        limelight.updateRobotOrientation(getImuAngle());
    }

    public Pose3D getLimelightPositionalData() {
        return limelight.getLatestResult().getBotpose_MT2();
    }

    public BaseRobot(LinearOpMode opmode) {
        this.opMode = opmode;
        this.opMode.telemetry.setMsTransmissionInterval(constants.TELEMETRY_MS_TRANSMISSION_INTERVAL);

        this.imu = createImu();

        initBulkReads();

        writeToTelemetry(">", "Hardware Initialized");
        updateTelemetry();

        try {
            switch_ = opmode.hardwareMap.get(DigitalChannel.class, "switch");
        } catch (Exception ignored){}


        //limelight = opmode.hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(0);
        //limelight.start();
    }

    private IMU createImu() {
        BNO055IMUNew.Parameters imuParameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, // must be adjusted if CH moves
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));

        IMU imu = opMode.hardwareMap.get(IMU.class, "imu");
        boolean worked = imu.initialize(imuParameters);
        imu.resetYaw();
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

    public void writeRobotPositionToTelemetry(double x, double y){
        packet.field().fillRect(x, y, 10, 10);
        writeToTelemetry("Robot Pos X", x);
        writeToTelemetry("Robot Pos Z", y);
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

    /**
     * Initialize a April Tag Scanning system with a custom dumb webcam. If you are using LimeLight, this is not the system you want.
     * @return A reference to an AprilTagModule. Managed by the BaseRobot, so not required to store it yourself.
     */
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

    /**
     * Returns the switch's state. Note that if a switch is not attached (or not configured), this will always return true.
     */
    public boolean getSwitchState(){
        return switch_ == null ? true : switch_.getState();
    }
}