package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SensorDevice;
import org.firstinspires.ftc.teamcode.vision.AprilTagData;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

public class Limelight extends SensorDevice<Boolean> {
    private Limelight3A limelight;
    public Limelight(HardwareMap hardwareMap, SensorInitData initData, BiConsumer<String, Object> telemetryFunc) {
        super(telemetryFunc);
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            available = false;
            return;
        }

        limelight.pipelineSwitch(0);
        limelight.start();

        available = true;
    }

    @Override
    public void start() {

    }

    @Override
    public Boolean poll() {
        return null;
    }

    public ArrayList<AprilTagData> getLastLimelightAprilTags(){
        ArrayList<AprilTagData> out = new ArrayList<>();

        limelight.getLatestResult().getFiducialResults().forEach((LLResultTypes.FiducialResult a) -> out.add(new AprilTagData(a.getFiducialId(), a.getTargetPoseRobotSpace().getPosition().z, 0)));

        return out;
    }

    // todo this limelight stuff shouldn't be here, move back
    public List<LLResultTypes.FiducialResult> getLastLimelightAprilTagsRaw(){
        return limelight.getLatestResult().getFiducialResults();
    }

    public void updateLimelightIMUData(double angleRad){
        limelight.updateRobotOrientation(angleRad);
    }

    public Pose3D getLimelightPositionalData() {
        return limelight.getLatestResult().getBotpose_MT2();
    }
}
