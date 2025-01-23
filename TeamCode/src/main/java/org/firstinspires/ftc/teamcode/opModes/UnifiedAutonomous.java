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

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.FlipBar;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.VerticalSlides;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.PropIdentificationVisualPipeline;
import org.firstinspires.ftc.teamcode.vision.PropIdentificationVisualPipeline.PropLocation;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism.InitData;

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
    private Intake intake;
    private FlipBar flipBar;
    private VerticalSlides verticals;
    public void runOpMode(){
        constants.ROBOT_HEADING = 0;
        if(currentLocation == null) currentLocation = Locations.Unknown;
        // Example autonomous code that can be used. Don't be afraid to expand or remodel it as needed
        BaseRobot robot = new BaseRobot(this);
        intake = new Intake(hardwareMap, new InitData(), robot::writeToTelemetry);
        flipBar = new FlipBar(hardwareMap, new InitData(), robot::writeToTelemetry);
        flipBar.closeSpecimenClaw();
        verticals = new VerticalSlides(hardwareMap, new InitData(), robot::writeToTelemetry);

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

        intake.setWristPosition(Intake.WristPosition.INIT);

        Pose2d startPose = new Pose2d(24, -60, -Math.PI / 2);
        roadrunnerDrive = new MecanumDrive(hardwareMap, startPose);
        TrajectoryActionBuilder toChamberPath = roadrunnerDrive.actionBuilder(startPose)
            .strafeTo(new Vector2d(-6, -30.5));

        Action toChamber = toChamberPath.build();

        TrajectoryActionBuilder retrieveSample1Path = toChamberPath.endTrajectory().fresh()
            .strafeTo(new Vector2d(-2, -35))
            .strafeTo(new Vector2d(32, -35)) // strafe
            .strafeTo(new Vector2d(45, -10)) // first sample
            .strafeTo(new Vector2d(45, -54)); // push

        Action moveSample1 = retrieveSample1Path.build();

        TrajectoryActionBuilder retrieveSample2Path = retrieveSample1Path.endTrajectory().fresh()
            .strafeTo(new Vector2d(45, -12)) // back up
            .strafeTo(new Vector2d(55, -12)); // second sample

        Action moveSample2 = retrieveSample2Path.build();

        TrajectoryActionBuilder retrieveSample2Pt2Path = retrieveSample2Path.endTrajectory().fresh()
            .strafeTo(new Vector2d(55, -54)); // push

        Action moveSample2Pt2 = retrieveSample2Pt2Path.build();

        TrajectoryActionBuilder retrieveSample3Path = retrieveSample2Path.endTrajectory().fresh()
            .strafeTo(new Vector2d(55, -12)) // back up
            .strafeTo(new Vector2d(63, -12)); // third sample

        Action moveSample3 = retrieveSample3Path.build();

        TrajectoryActionBuilder moveSample3Pt2Path = retrieveSample3Path.endTrajectory().fresh()
            .strafeTo(new Vector2d(63, -54)); // get sample

        Action moveSample3Pt2 = moveSample3Pt2Path.build();

        TrajectoryActionBuilder humanPlayerToChamberPath1 = retrieveSample2Pt2Path.endTrajectory().fresh()
            .strafeTo(new Vector2d(-3, -30));

        Action humanPlayerToChamber1 = humanPlayerToChamberPath1.build();

        TrajectoryActionBuilder chamberToHumanPlayerPath1 = humanPlayerToChamberPath1.endTrajectory().fresh()
            .strafeTo(new Vector2d(45, -52));

        Action chamberToHumanPlayer1 = chamberToHumanPlayerPath1.build();

        TrajectoryActionBuilder humanPlayerToChamberPath2 = chamberToHumanPlayerPath1.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -30));

        Action humanPlayerToChamber2 = humanPlayerToChamberPath2.build();

        TrajectoryActionBuilder chamberToHumanPlayerPath2 = humanPlayerToChamberPath2.endTrajectory().fresh()
                .strafeTo(new Vector2d(45, -52));

        Action chamberToHumanPlayer2 = chamberToHumanPlayerPath2.build();

        TrajectoryActionBuilder humanPlayerToChamberPath3 = chamberToHumanPlayerPath2.endTrajectory().fresh()
                .strafeTo(new Vector2d(3, -30));

        Action humanPlayerToChamber3 = humanPlayerToChamberPath3.build();

        TrajectoryActionBuilder chamberToHumanPlayerPath3 = humanPlayerToChamberPath3.endTrajectory().fresh()
                .strafeTo(new Vector2d(45, -52));

        Action chamberToHumanPlayer3 = chamberToHumanPlayerPath3.build();

        TrajectoryActionBuilder humanPlayerToChamberPath4 = chamberToHumanPlayerPath3.endTrajectory().fresh()
                .strafeTo(new Vector2d(6, -30));

        Action humanPlayerToChamber4 = humanPlayerToChamberPath4.build();

        TrajectoryActionBuilder inChamberToParkPath = humanPlayerToChamberPath3.endTrajectory().fresh()
            .strafeToConstantHeading(new Vector2d(60, -60));

        Action inChamberToPark = inChamberToParkPath.build();

        // assumes starting at human player. ends at the chamber
        // JAVA SUCKSSSS. must be like this because of non-reusability (Function requires a lot of scoping work)
        SequentialAction clipSpecimen1 = new SequentialAction(
            new InstantAction(flipBar::closeSpecimenClaw),
            new SleepAction(0.25),
            new ParallelAction(
                new SequentialAction(
                    new InstantAction(flipBar::specimenArmToHookAuto),
                    new SleepAction(.3)
                ),
                humanPlayerToChamber1,
                verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0.5)
            ),
            verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0.8),
            new SleepAction(.5),
            new InstantAction(flipBar::openSpecimenClaw),
            new SleepAction(.25)
        );

        SequentialAction clipSpecimen2 = new SequentialAction(
                new InstantAction(flipBar::closeSpecimenClaw),
                new SleepAction(0.25),
                new ParallelAction(
                        new SequentialAction(
                                new InstantAction(flipBar::specimenArmToHookAuto),
                                new SleepAction(.3)
                        ),
                        humanPlayerToChamber2,
                        verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0.5)
                ),
                verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0.8),
                new SleepAction(.5),
                new InstantAction(flipBar::openSpecimenClaw),
                new SleepAction(.25)
        );

        SequentialAction clipSpecimen3 = new SequentialAction(
                new InstantAction(flipBar::closeSpecimenClaw),
                new SleepAction(0.25),
                new ParallelAction(
                        new SequentialAction(
                                new InstantAction(flipBar::specimenArmToHookAuto),
                                new SleepAction(.3)
                        ),
                        humanPlayerToChamber3,
                        verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0.5)
                ),
                verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0.8),
                new SleepAction(.5),
                new InstantAction(flipBar::openSpecimenClaw),
                new SleepAction(.25)
        );

        SequentialAction clipSpecimen4 = new SequentialAction(
                new InstantAction(flipBar::closeSpecimenClaw),
                new SleepAction(0.25),
                new ParallelAction(
                        new SequentialAction(
                                new InstantAction(flipBar::specimenArmToHookAuto),
                                new SleepAction(.3)
                        ),
                        humanPlayerToChamber4,
                        verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0.5)
                ),
                new SleepAction(.25),
                verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0.8),
                new SleepAction(.5),
                new InstantAction(flipBar::openSpecimenClaw),
                new SleepAction(.25)
        );

        ParallelAction resetForNextSpecimen1 = new ParallelAction(
            chamberToHumanPlayer1,
            verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0),
            new InstantAction(flipBar::specimenArmToPickupAuto)
        );

        ParallelAction resetForNextSpecimen2 = new ParallelAction(
                chamberToHumanPlayer2,
                verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0),
                new InstantAction(flipBar::specimenArmToPickupAuto)
        );

        ParallelAction resetForNextSpecimen3 = new ParallelAction(
                chamberToHumanPlayer3,
                verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0),
                new InstantAction(flipBar::specimenArmToPickupAuto)
        );

        TrajectoryActionBuilder spinToIntakePath = humanPlayerToChamberPath3.endTrajectory().fresh()
                .strafeTo(new Vector2d(3, -32))
                .turnTo(Math.PI / 2);

        Action spinToIntake = spinToIntakePath.build();

        robot.writeToTelemetry("INIT STATUS", "READY");
        robot.updateTelemetry();

        waitForStart(); // setup done actually do things

        intake.setWristPosition(Intake.WristPosition.LOW);

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
                        verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0.5),
                        new SequentialAction(
                            new InstantAction(flipBar::closeSpecimenClaw),
                            new InstantAction(flipBar::specimenArmToHookAuto),
                            new SleepAction(.3) // wait for movement
                        )
                    ),
                    verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0.8),
                    new SleepAction(.5),
                    new InstantAction(flipBar::openSpecimenClaw),
                    new SleepAction(.25),
                    new ParallelAction(
                        // time to go move samples
                        new SequentialAction(
                            new InstantAction(flipBar::specimenArmToHookAuto),
                            moveSample1,
                            moveSample2,
                            new InstantAction(flipBar::specimenArmToPickupAuto),
                            moveSample2Pt2
                            //moveSample3,
                            //moveSample3Pt2
                        ), // this puts us at human player spot
                        verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0)
                    ),
                    // do specimens
                    clipSpecimen1,
                    resetForNextSpecimen1,
                    clipSpecimen2,
                    resetForNextSpecimen2,
                    clipSpecimen3,
                    //resetForNextSpecimen3,
                    //clipSpecimen4,
                    // park
                    new SleepAction(.25),
                    new ParallelAction(
                        //inChamberToPark,
                        new SequentialAction(
                            new InstantAction(() -> intake.setWristPosition(Intake.WristPosition.HIGH)),
                            spinToIntake,
                            intake.roadrunnerExtendHorizontalSlideToLength(0.5)
                        ),
                        new SequentialAction(
                            verticals.roadrunnerRaiseSpecimenSlideToHeightBugged(0),
                            new InstantAction(flipBar::specimenArmToPickup),
                            new SleepAction(0.3)
                        )
                    )
                ));
                // put our current heading in constants for field teleopmodes to read later
                // todo THIS WONT WORK what if auto dies early?
                constants.ROBOT_HEADING = robot.getImuAngle() + Math.PI; // add pi, reversed
                break;
            }
        }
    }
}