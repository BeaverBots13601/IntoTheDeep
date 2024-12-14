package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SeasonalRobot;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.PropIdentificationVisualPipeline;
import org.firstinspires.ftc.teamcode.vision.PropIdentificationVisualPipeline.PropLocation;
import org.firstinspires.ftc.teamcode.constants;

import java.util.List;

@Autonomous(name="Automatic Autonomous")
@Disabled
public class UnifiedAutonomous extends LinearOpMode {
    protected enum Locations {
        BlueClose,
        BlueFar,
        RedClose,
        RedFar,
        Unknown
    }
    protected enum Path {
        STANDARD,
        ALTERNATE
    }
    private PropLocation propLocation;
    protected Locations currentLocation;
    protected Path pathToFollow = Path.STANDARD;
    private PropIdentificationVisualPipeline line;
    private MecanumDrive roadrunnerDrive;
    public void runOpMode(){
        constants.ROBOT_HEADING = 0;
        if(currentLocation == null) currentLocation = Locations.Unknown;
        // Example autonomous code that can be used. Don't be afraid to expand or remodel it as needed
        SeasonalRobot robot = new SeasonalRobot(this);
        robot.closeSpecimenClaw();

        /* !! This code uses AprilTags to determine where we are starting on the field.
        Note that code segments like these aren't always going to be useful; don't feel obligated to
        use them just because they exist.
        sleep(1000);

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

        Pose2d startPose = new Pose2d(24, -60, Math.PI / 2);
        roadrunnerDrive = new MecanumDrive(hardwareMap, startPose);
        TrajectoryActionBuilder toChamberPath = roadrunnerDrive.actionBuilder(startPose)
                .strafeToConstantHeading(new Vector2d(-2, -32));

        Action toChamber = toChamberPath.build();

        TrajectoryActionBuilder intoChamberPath = toChamberPath.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-2, -30.5));

        Action intoChamber = intoChamberPath.build();

        TrajectoryActionBuilder chamberToHumanPlayerPath = intoChamberPath.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-2, -35))
                .strafeToConstantHeading(new Vector2d(35, -35)); // strafe

        Action chamberToHumanPlayer = chamberToHumanPlayerPath.build();

        TrajectoryActionBuilder intermediatePath = chamberToHumanPlayerPath.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(35, -8));

        Action intermediate = intermediatePath.build();

        TrajectoryActionBuilder intermediate2Path = intermediatePath.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(45, -12), 0)
                .splineToConstantHeading(new Vector2d(45, -54), 0) //push sample to parking zone
                .splineTo(new Vector2d(44, -30), Math.PI)
                .splineToConstantHeading(new Vector2d(45, -58.5), Math.PI); //drive to pickup

        Action intermediate2 = intermediate2Path.build();

        TrajectoryActionBuilder humanPlayerToChamberPath = intermediate2Path.endTrajectory().fresh()
                .splineTo(new Vector2d(0, -36.5), 0);

        Action humanPlayerToChamber = humanPlayerToChamberPath.build();

        TrajectoryActionBuilder intoChamber2Path = humanPlayerToChamberPath.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(0, -30), 0);

        Action intoChamber2 = intoChamber2Path.build();

        TrajectoryActionBuilder toHumanPlayerPath = intoChamber2Path.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(0, -32), 0)
                .splineTo(new Vector2d(45, -59), Math.PI);

        Action toHumanPlayer = toHumanPlayerPath.build();

        TrajectoryActionBuilder humanPlayerToChamber2Path = toHumanPlayerPath.endTrajectory().fresh()
                .splineTo(new Vector2d(2, -36), 0);

        Action humanPlayerToChamber2 = humanPlayerToChamber2Path.build();

        TrajectoryActionBuilder intoChamber3Path = humanPlayerToChamber2Path.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(2, -30), 0);

        Action intoChamber3 = intoChamber3Path.build();

        TrajectoryActionBuilder inChamberToParkPath = intoChamber3Path.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(60, -60), 0);

        Action inChamberToPark = inChamberToParkPath.build();

        robot.writeToTelemetry("INIT STATUS", "READY");
        robot.updateTelemetry();

        waitForStart(); // setup done actually do things

        switch(currentLocation){
            case RedFar:
            case BlueFar: {
                break;
            }
            case RedClose:
            case BlueClose: {
                //far from basket, seam just beyond human player
                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                toChamber,
                                robot.roadrunnerRaiseSpecimenSlideToHeight(0.7)
                        ),
                        intoChamber,
                        robot.roadrunnerRaiseSpecimenSlideToHeight(0.4),
                        new SleepAction(.25),
                        new InstantAction(robot::openSpecimenClaw),
                        new SleepAction(.25),
                        // time to go get another specimen
                        new ParallelAction(
                                new SequentialAction(
                                        chamberToHumanPlayer,
                                        intermediate,
                                        intermediate2
                                ),
                                robot.roadrunnerRaiseSpecimenSlideToHeight(0)
                        ),
                        new SleepAction(0.3),
                        new InstantAction(robot::closeSpecimenClaw),
                        new SleepAction(0.3),
                        robot.roadrunnerRaiseSpecimenSlideToHeight(0.15),
                        humanPlayerToChamber,
                        robot.roadrunnerRaiseSpecimenSlideToHeight(0.7),
                        new SleepAction(.80),
                        intoChamber2,
                        robot.roadrunnerRaiseSpecimenSlideToHeight(0.4),
                        new SleepAction(.25),
                        new InstantAction(robot::openSpecimenClaw),
                        new SleepAction(.25),
                        // third specimen
                        new ParallelAction(
                                robot.roadrunnerRaiseSpecimenSlideToHeight(0),
                                toHumanPlayer
                        ),
                        new SleepAction(0.3),
                        new InstantAction(robot::closeSpecimenClaw),
                        new SleepAction(0.3),
                        robot.roadrunnerRaiseSpecimenSlideToHeight(0.15),
                        humanPlayerToChamber2,
                        robot.roadrunnerRaiseSpecimenSlideToHeight(0.7),
                        new SleepAction(.80),
                        intoChamber3,
                        robot.roadrunnerRaiseSpecimenSlideToHeight(0.4),
                        new SleepAction(.25),
                        new InstantAction(robot::openSpecimenClaw),
                        new SleepAction(.25),
                        // go park
                        inChamberToPark
                ));
                // put our current heading in constants for field teleopmodes to read later
                // todo THIS WONT WORK what if auto dies early?
                constants.ROBOT_HEADING = robot.getImuAngle() + Math.PI; // add pi, reversed
                break;
            }
        }
    }
}