package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SeasonalRobot;
import org.firstinspires.ftc.teamcode.vision.AprilTagData;
import org.firstinspires.ftc.teamcode.vision.PropIdentificationVisualPipeline;
import org.firstinspires.ftc.teamcode.vision.PropIdentificationVisualPipeline.PropLocation;
import org.firstinspires.ftc.teamcode.constants;

import java.util.List;

@Autonomous(name="Automatic Autonomous", preselectTeleOp = "UnifiedTeleOp")
public class UnifiedAutonomous extends LinearOpMode {
    protected enum Locations {
        BlueClose,
        BlueFar,
        RedClose,
        RedFar,
        Unknown
    }
    private PropLocation propLocation;
    protected Locations currentLocation;
    private PropIdentificationVisualPipeline line;
    public void runOpMode(){
        constants.ROBOT_HEADING = 0;
        if(currentLocation == null) currentLocation = Locations.Unknown;
        // Example autonomous code that can be used. Don't be afraid to expand or remodel it as needed
        SeasonalRobot robot = new SeasonalRobot(this);
        sleep(1000);

        /* !! This code uses AprilTags to determine where we are starting on the field.
        Note that code segments like these aren't always going to be useful; don't feel obligated to
        use them just because they exist.

        AprilTagModule tags = robot.getMod();
        int iterations2 = 0;
        while(tags.updateAprilTagData().size() == 0 && iterations2 < 500){ sleep(10); iterations2++; }
        tags.updateAprilTagData();
        AprilTagData max = new AprilTagData(); // default
        for(AprilTagData tag : tags.updateAprilTagData()){
            if(tag.getDist() > max.getDist()) max = tag;
        }

        robot.writeToTelemetry("Max Tag Dist", max.getDist());
        robot.writeToTelemetry("Max Tag ID", max.getId());
        robot.updateTelemetry();

        // assumes camera is mounted on left side. Sorry it's kinda confusing, using a map helps to understand
        if (max.getId() == 7 || max.getId() == 10) { // sees red wall tag
            if(max.getDist() > constants.APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES){
                currentLocation = Locations.RedClose; // tag far away, we are close to bb
            } else {
                currentLocation = Locations.RedFar; // tag nearby
            }
        } else {
            if(max.getDist() == -10 || max.getDist() > constants.APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES) {
                currentLocation = Locations.BlueFar; // inverse of blue because the camera is pointing at & reading bb now
                // also the default case
            } else {
                currentLocation = Locations.BlueClose; // tag nearby
            }
        }*/

        /* Example code for setting up identification of where our colored team prop is in front of us

        if(currentLocation == Locations.BlueClose || currentLocation == Locations.BlueFar) {
            line = robot.initializePropIDPipeline(PropIdentificationVisualPipeline.PropColors.BLUE);
        } else {
            line = robot.initializePropIDPipeline(PropIdentificationVisualPipeline.PropColors.RED);
        }

        OpenCvCamera frontCamera = robot.getFrontCamera().setPipeline(line);*/

        if(currentLocation == Locations.Unknown) {
            // limelight apriltag
            List<LLResultTypes.FiducialResult> tags = robot.getLastLimelightAprilTagsRaw();
            int iterations2 = 0;
            while (tags.size() == 0 && iterations2 < 500) {
                sleep(10);
                iterations2++;
                tags = robot.getLastLimelightAprilTagsRaw();
            }

            @Nullable
            FiducialResult importantTag = null;
            for (LLResultTypes.FiducialResult tag : tags){
                if(tag.getFiducialId() == 12 || tag.getFiducialId() == 15) importantTag = tag; break;
            }

            if (importantTag == null) {
                // todo panic case
            } else if(importantTag.getFiducialId() == 12) {
                //importantTag.
            } else if (importantTag.getFiducialId() == 15){

            }
            /*AprilTagData max = new AprilTagData(); // default
            for (AprilTagData tag : tags) {
                if (tag.getDist() > max.getDist()) max = tag;
            }


            if (max.getId() == 14) { // sees red wall tag
                if (max.getDist() > constants.APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES) {
                    currentLocation = Locations.RedFar; // tag far away
                } else {
                    currentLocation = Locations.RedClose; // tag nearby
                }
            } else if (max.getId() == 11) { // sees blue wall tag
                if (max.getDist() > constants.APRILTAG_DISTANCE_DETERMINATION_THRESHOLD_INCHES) {
                    currentLocation = Locations.BlueClose; // tag far away
                } else {
                    currentLocation = Locations.BlueFar; // tag nearby
                }
            } else {
                // uh oh todo make this case
            }*/
        }

        waitForStart(); // setup done actually do things

        sleep(1000);

        /* Used for dialing in prop to thing in front of us. Not always relevant

        int iterations = 0;b
        while((line.getLastPropLocation() == PropLocation.UNKNOWN || line.getLastPropLocation() == null) && iterations < 500){ sleep(10); iterations++; }
        propLocation = line.getLastPropLocation();
        robot.writeToTelemetry("Location", currentLocation);
        robot.writeToTelemetry("Prop Location", propLocation);
        robot.writeToTelemetry("iterations: ", iterations);
        robot.updateTelemetry();
        robot.driveInches(26, .25);

        if(propLocation == PropLocation.LEFT){
            // left
        } else if(propLocation == PropLocation.CENTER || propLocation == PropLocation.UNKNOWN) {
            // center and fallback
        } else if(propLocation == PropLocation.RIGHT) {
            // right
        }*/

        // Driving to where we need to go
        /*switch(currentLocation){
            case BlueClose: {
                // Autonomous programs generally consist of these two commands:
                robot.driveInches(12, 1);
                robot.turnDegrees(90, 1);
                // Make a different one for every case, unless the field is symmetrical on every side
                break;
            }
            case BlueFar: {
                robot.driveInches(12, 1);
                robot.turnDegrees(-90, 1);
                break;
            }
            case RedClose: {
                robot.driveInches(-12, 1);
                robot.turnDegrees(90, 1);
                break;
            }
            case RedFar: {
                robot.driveInches(-12, 1);
                robot.turnDegrees(-90, 1);
                break;
            }
        }*/

        switch(currentLocation){
            case RedFar:
            case BlueFar: {
                //under basket
                robot.turnDegrees(-90, .5);
                robot.driveInches(24, 1);
                robot.raiseVerticalArm();
                robot.openSpecimenClaw();
                robot.lowerVerticalArm();
                robot.turnDegrees(90,.5);
                /*for(int i = 0; i < 3; i++) { // samples - can't do this because of revised design
                    robot.driveInches(24, 1);
                    //close_hand
                    robot.turnDegrees(180, .5);
                    robot.driveInches(24, 1);
                    //open_hand
                    robot.turnDegrees(180, .5);
                }*/
                robot.turnDegrees(45,.5);
                robot.driveInches(48,1);
                robot.turnDegrees(45,.5);
                robot.raiseVerticalArm();
                robot.driveInches(6, .5);
                break;

                /*// alternate
                robot.turnDegrees(45,.5);
                robot.driveInches(24,1);
                robot.turnDegrees(45,.5);
                robot.driveInches(8,1);
                robot.turnDegrees(-45,.5);
                //attach_specimen
                robot.turnDegrees(90,.5);
                robot.driveInches(48,1);
                robot.turnDegrees(90,.5);
                robot.driveInches(12,1);
                robot.turnDegrees(-90,.5);
                //close_hand
                robot.turnDegrees(135,.5);
                robot.driveInches(20,1);
                robot.turnDegrees(135,.5);
                //close_hand*/

            }
            case RedClose:
            case BlueClose: {
                //far from basket, seam just beyond human player
                robot.driveInches(-4, .5);
                robot.turnDegrees(-40, .5);
                robot.driveInches(-38, 0.6);
                robot.turnDegrees(40, .5);
                robot.raiseVerticalArmsToHeight(0.20);
                robot.driveInches(-6, 0.2);
                robot.raiseVerticalArmsToHeight(0.15); // todo adjust me also calibrate arms
                robot.openSpecimenClaw();
                robot.driveInches(18, 0.6);
                robot.turnDegrees(-90, .5);
                robot.driveInches(38,0.6);
                robot.turnDegrees(-90, .5);
                robot.driveInches(9, 0.6);
                // extend some amnt
                robot.closeClawMachine();
                robot.turnDegrees(180,.5);
                robot.driveInches(-40,0.6);
                robot.openClawMachine();
                break;

                /*//alternate
                robot.turnDegrees(-45,.5);
                robot.driveInches(24,1);
                robot.turnDegrees(-45,.5);
                robot.driveInches(8,1);
                robot.turnDegrees(90,.5);
                //attach-specimen
                robot.turnDegrees(-90,.5);
                robot.driveInches(48,1);
                robot.turnDegrees(-90,.5);
                robot.turnDegrees(12,.5);
                robot.turnDegrees(90,.5);
                //close_hand
                robot.turnDegrees(-135,.5);
                robot.driveInches(20,1);
                robot.turnDegrees(-135,.5);
                //open_hand*/

            }
        }

        // put our current heading in constants for field teleopmodes to read later
        constants.ROBOT_HEADING = robot.getImuAngle();
    }
}