package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.Nullable;

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
                if(pathToFollow == Path.STANDARD){
                    // put our current heading in constants for field teleopmodes to read later
                    //constants.ROBOT_HEADING = robot.getImuAngle();
                } else {
                    // just l1 ascent from basket
                    robot.driveInches(45, .6);
                    robot.turnDegrees(-83, .5);
                    robot.driveInches(-10, .6);
                    robot.raiseRearVerticalArmsToHeight(0.45);
                    robot.driveInches(-4, .6);
                    robot.raiseRearVerticalArmsToHeight(0.4);
                    // put our current heading in constants for field teleopmodes to read later
                    constants.ROBOT_HEADING = robot.getImuAngle() + (Math.PI / 2);
                }
                break;
            }
            case RedClose:
            case BlueClose: {
                //far from basket, seam just beyond human player
                Pose2d startPose = new Pose2d(24, -60, Math.PI / 2);
                roadrunnerDrive = new MecanumDrive(hardwareMap, startPose);
                TrajectoryActionBuilder toChamber = roadrunnerDrive.actionBuilder(startPose)
                        .splineToConstantHeading(new Vector2d(0, -32), 0);

                TrajectoryActionBuilder intoChamber = toChamber.endTrajectory().fresh()
                        .splineToConstantHeading(new Vector2d(0, -30.5), 0);

                TrajectoryActionBuilder backUpFromChamber = intoChamber.endTrajectory().fresh()
                        .splineToConstantHeading(new Vector2d(0, -34), 0);

                TrajectoryActionBuilder chamberToHumanPlayer = backUpFromChamber.endTrajectory().fresh()
                        .splineToConstantHeading(new Vector2d(36, -34), 0)
                        .splineToConstantHeading(new Vector2d(36, -6), 0)
                        .splineToConstantHeading(new Vector2d(45, -6), 0)
                        .splineToConstantHeading(new Vector2d(45, -54), 0) //push sample to parking zone
                        .splineTo(new Vector2d(48, -30), Math.PI)
                        .splineToConstantHeading(new Vector2d(45, -58), Math.PI); //drive to pickup

                TrajectoryActionBuilder humanPlayerToChamber = chamberToHumanPlayer.endTrajectory().fresh()
                        .splineTo(new Vector2d(0, -36), 0);

                TrajectoryActionBuilder intoChamber2 = humanPlayerToChamber.endTrajectory().fresh()
                        .splineToConstantHeading(new Vector2d(2, -30.5), 0);

                TrajectoryActionBuilder inChamberToPark = intoChamber2.endTrajectory().fresh()
                                .splineToConstantHeading(new Vector2d(60, -60), 0);

                Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                            toChamber.build(),
                            robot.roadrunnerRaiseSpecimenSlideToHeight(0.7)
                        ),
                        intoChamber.build(),
                        robot.roadrunnerRaiseSpecimenSlideToHeight(0.4),
                        new SleepAction(.25),
                        new InstantAction(robot::openSpecimenClaw),
                        new SleepAction(.25),
                        new ParallelAction(
                                new SequentialAction(
                                        backUpFromChamber.build(), // back up
                                        chamberToHumanPlayer.build()
                                ),
                                // replace me w/
                                new InstantAction(() -> robot.raiseSpecimenSlideToHeightAsync(0))
                        ),
                        new SleepAction(0.3),
                        new InstantAction(robot::closeSpecimenClaw),
                        new SleepAction(0.3),
                        robot.roadrunnerRaiseSpecimenSlideToHeight(0.1),
                        humanPlayerToChamber.build(),
                        robot.roadrunnerRaiseSpecimenSlideToHeight(0.7),
                        new SleepAction(.80),
                        intoChamber2.build(),
                        robot.roadrunnerRaiseSpecimenSlideToHeight(0.4),
                        new SleepAction(.25),
                        new InstantAction(robot::openSpecimenClaw),
                        new SleepAction(.25),
                        inChamberToPark.build()
                ));

                /*robot.raiseSpecimenSlideToHeightAsync(0.7);
                robot.driveAtAngle(45, 130, 1);
                robot.raiseSpecimenSlideToHeight(0.45);
                robot.openSpecimenClaw();
                sleep(250);
                robot.raiseSpecimenSlideToHeightAsync(0);
                robot.driveInches(-1, 1);
                robot.driveStrafe(32.5, 1);
                robot.driveInches(24, 1);
                robot.driveStrafe(12, 1);
                robot.driveInches(-40, 1);
                robot.driveInches(12, 1);
                sleep(50);
                robot.turnDegrees(170, .5);
                sleep(50);
                robot.driveInches(26, 1); // 21
                robot.closeSpecimenClaw();
                sleep(300);
                robot.raiseSpecimenSlideToHeight(0.1);
                robot.driveInches(-3, 0.6);
                robot.raiseSpecimenSlideToHeightAsync(0);
                robot.turnDegrees(173, .5);
                robot.driveStrafe(-42, 1); // 48
                robot.turnDegrees(-15, .5);
                robot.raiseSpecimenSlideToHeightAsync(.7);
                robot.driveAtAngle(44, 130, 1); // 43, .8
                robot.raiseSpecimenSlideToHeight(0.45);
                robot.openSpecimenClaw();
                sleep(250);
                robot.driveAtAngle(60, -40, 1);*/
                // put our current heading in constants for field teleopmodes to read later
                constants.ROBOT_HEADING = robot.getImuAngle() + Math.PI; // add pi, reversed
                /*robot.setHorizontalArmPower(0.5);
                sleep(1250); // moving from starting pos
                robot.setHorizontalArmPower(0);
                robot.closeClawMachine();
                robot.setHorizontalArmPower(-0.5);
                sleep(900); // pulling back but not completely.
                // NOTE: Definition of parking is having the 'robot' be 'partially' in human player zone.
                // Can we just extend the horizontal arm and stay out?
                robot.setHorizontalArmPower(0);
                robot.turnDegrees(180,.5);
                robot.driveInches(-40,0.6);
                robot.openClawMachine();
                 */
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
    }
}