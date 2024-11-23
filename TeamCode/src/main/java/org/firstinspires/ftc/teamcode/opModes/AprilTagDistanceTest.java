package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp
@Disabled
public class AprilTagDistanceTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
       Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
       FtcDashboard dash = FtcDashboard.getInstance();
       TelemetryPacket packet = new TelemetryPacket();
       waitForStart();
       ll.start();

       while (opModeIsActive()){
           LLResult res = ll.getLatestResult();
           if (res == null) continue;
           List<LLResultTypes.FiducialResult> taglist = res.getFiducialResults();
           if (taglist.size() == 0) continue;
           LLResultTypes.FiducialResult tag = taglist.get(0);
           if(tag == null) continue;
           double dist = tag.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH).z;
           packet.put("Distance from Tag (in)", dist);
           dash.sendTelemetryPacket(packet);
           packet = new TelemetryPacket();
       }
    }
}
