package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
    TODO: Build web-tool that allows robot configuration i.e driver station (ftc-dash)
 */

public class TelemetryManager {
    private final Telemetry telemetry;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    public TelemetryManager(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry.setMsTransmissionInterval(constants.TELEMETRY_MS_TRANSMISSION_INTERVAL);

        writeToTelemetry(">", "Hardware Initialized");
        updateTelemetry();
    }

    public void writeToTelemetry(String caption, Object value) {
        this.telemetry.addData(caption, value);
        packet.put(caption, value);
    }

    public void updateTelemetry() {
        this.telemetry.update();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    public void writeRobotPositionToTelemetry(double x, double y){
        packet.field().fillRect(x, y, 10, 10);
        writeToTelemetry("Robot Pos X", x);
        writeToTelemetry("Robot Pos Z", y);
    }

    public boolean isDashboardEnabled(){
        return dashboard.isEnabled();
    }
}