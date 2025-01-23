package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
public class constants {
    public static final int TELEMETRY_MS_TRANSMISSION_INTERVAL = 25; // no clue what this does tbh

    // This speed is designed to be set dynamically within FTC Dashboard.
    public static double CUSTOM_FTC_DASHBOARD_SPEED = 0.65;
    // Heading global var
    public static double ROBOT_HEADING = 0;

    // Camera data
    public static final String FRONT_CAMERA_NAME = "camera";
    public static final int FRONT_CAMERA_WIDTH = 1280;
    public static final int FRONT_CAMERA_HEIGHT = 720;

    public static final String SIDE_CAMERA_NAME = "sideCamera";
    public static final int SIDE_CAMERA_WIDTH = 1280;
    public static final int SIDE_CAMERA_HEIGHT = 720;
    /**
     * The distance, in inches, at which the scanner decides we are at the "Far" position if the AprilTags it sees are beyond.
     * Might have to be fudged a bit if the calibration is wonky
     */
    public static double APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES = 85;
    /**
     * The percentage, from [0, 1.0], at which values equal to or below will not be used to determine the location of the team prop.
     * Ex. If the highest percentage of color is the center area at 8%, and the value is at 0.1, it will be declared 'unknown' instead of 'center'.
     */
    public static double COLOR_UNKNOWN_THRESHOLD_PERCENT = 0.1;
    public static double DETECTION_BOX_OFFSET_SIDES_PX = 128;

    // disable roadrunner tuning opmodes
    public static final boolean DISABLE_TUNING_OPMODES = true;
}
