package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
public class constants {
    public static final int TELEMETRY_MS_TRANSMISSION_INTERVAL = 25; // no clue what this does tbh

    // This speed is designed to be set dynamically within FTC Dashboard.
    public static double CUSTOM_FTC_DASHBOARD_SPEED = 0.65;
    // Heading global var
    public static double ROBOT_HEADING = 0;

    /**
     * The distance, in inches, at which the scanner decides we are at the "Far" position if the AprilTags it sees are beyond.
     * Might have to be fudged a bit if the calibration is wonky
     */
    public static double APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES = 85;

    // disable roadrunner tuning opmodes
    public static final boolean DISABLE_TUNING_OPMODES = true;

    /**
     * Enables bulk reads which allow faster hardware call times.
     * Set to auto currently but if speed becomes an issue this can be manually configured.
     * TODO: Re-home me.
     */
    public static void initBulkReads(HardwareMap hardwareMap){
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
