package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.misc.Pose;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.SeasonalRobot;
import org.firstinspires.ftc.teamcode.SeasonalRobot.LimiterState;
import org.firstinspires.ftc.teamcode.SeasonalRobot.WristPosition;
import org.firstinspires.ftc.teamcode.rr.InterruptableAction;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

public abstract class UnifiedTeleOp extends LinearOpMode {
    /** This field may be immediately changed by the switch state update. */
    protected DriveMode orientationMode = DriveMode.ROBOT; // override me
    protected RobotConfiguration configurationMode = RobotConfiguration.RESTRICTED; // override me
    protected TeamColor teamColor = TeamColor.BLUE; // override me
    protected boolean allowBaskets = false; // override me
    private SeasonalRobot typedRobot;
    private BaseRobot robot;
    private Gamepad currentGamepadOne = new Gamepad();
    private Gamepad previousGamepadOne = new Gamepad();
    private Gamepad currentGamepadTwo = new Gamepad();
    private Gamepad previousGamepadTwo = new Gamepad();
    private boolean ascentMode = false;
    private boolean manualVerticalMode = false;

    protected enum RobotConfiguration {
        RESTRICTED,
        FULL
    }
    protected enum DriveMode {
        FIELD,
        ROBOT
    }
    protected enum TeamColor {
        RED,
        BLUE
    }
    private enum Color {
        RED,
        GREEN,
        BLUE,
        UNKNOWN
    }

    private List<Action> running = new ArrayList<>();
    private InterruptableAction runningSpecimenSlideAction = null;
    // class instead of primitive to allow nullability
    private Integer holdingSpecimenSlidePos = null;
    // we use multiple results to reduce false negatives. 3 works well
    private final int numResultsToUse = 5;
    private List<ColorResult> lastResultsArr = new ArrayList<>(numResultsToUse);
    private boolean pickingUp = false;
    private boolean intakeRunning = false;
    private boolean expellingBad = false;
    private enum ActionRunning {
        TO_BAR,
        TO_PLAYER,
        STOP
    }
    private Action runningTeleonomousAction;
    private ActionRunning actionRunning;
    private MecanumDrive drive = null;
    private static final Pose2d startLoopPose = new Pose2d(38.5, -59.5, -Math.PI / 2);
    private static final double endLoopPoseY = -41;
    private static final Vector2d[] endLoopPoses = {
            new Vector2d(-12, endLoopPoseY),
            new Vector2d(-10, endLoopPoseY),
            new Vector2d(-8, endLoopPoseY),
            new Vector2d(-6, endLoopPoseY),
            new Vector2d(-4, endLoopPoseY),
            new Vector2d(-2, endLoopPoseY),
            new Vector2d(0, endLoopPoseY),
            new Vector2d(2, endLoopPoseY),
            new Vector2d(4, endLoopPoseY),
            new Vector2d(6, endLoopPoseY),
    };
    private int posesIndex = 0;
    public void runOpMode() {
        if (configurationMode == RobotConfiguration.RESTRICTED) {
            robot = new BaseRobot(this, constants.WHEEL_DIAMETER, constants.ROBOT_DIAMETER);
        } else {
            robot = new SeasonalRobot(this);
            typedRobot = (SeasonalRobot) robot;
        }

        //updateSwitchState(robot.getSwitchState()); temporarily removed
        double referenceAngle;
        if(constants.ROBOT_HEADING != 0){
            referenceAngle = constants.ROBOT_HEADING;
        } else {
            referenceAngle = robot.getImuAngle();
        }
        int tmp_deadzoneadjust = 2;
        previousGamepadOne.copy(currentGamepadOne);
        previousGamepadTwo.copy(currentGamepadTwo);

        // preload array; values will be expunged shortly
        for (int i = 0; i < numResultsToUse; i++){
            lastResultsArr.add(null);
        }

        waitForStart();
        // we can't extend past the boundary in init (is that true?) so do it here
        if (typedRobot != null) { typedRobot.setWristPosition(WristPosition.HIGH); }
        while (opModeIsActive()) {
            currentGamepadOne.copy(gamepad1);
            currentGamepadTwo.copy(gamepad2);

            if (drive != null){
                robot.processRoadrunnerPose(drive.pose);
            }

            if (runningTeleonomousAction != null){
                // hijacking the loop
                if (currentGamepadOne.triangle && !previousGamepadOne.triangle){
                    // abort
                    runningTeleonomousAction = null;
                    drive = null;
                    previousGamepadOne.copy(currentGamepadOne);
                    previousGamepadTwo.copy(currentGamepadTwo);
                    continue;
                }
                TelemetryPacket packet = new TelemetryPacket();
                boolean state = runningTeleonomousAction.run(packet);
                robot.packetToTelemetry(packet);
                if (!state){
                    if (actionRunning == ActionRunning.TO_BAR){
                        typedRobot.setSpecimenSlideTargetPos(holdingSpecimenSlidePos);
                        typedRobot.setSpecimenSlidePower(1);
                        typedRobot.setSpecimenSlideMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }

                    // done moving; tuning now
                    while (opModeIsActive() && !(currentGamepadOne.square && !previousGamepadOne.square)){
                        previousGamepadOne.copy(currentGamepadOne);
                        previousGamepadTwo.copy(currentGamepadTwo);
                        currentGamepadOne.copy(gamepad1);
                        currentGamepadTwo.copy(gamepad2);

                        float stickX = gamepad1.left_stick_x * tmp_deadzoneadjust;
                        float stickY = -gamepad1.left_stick_y * tmp_deadzoneadjust;
                        float stickRotation = gamepad1.right_stick_x * tmp_deadzoneadjust;

                        double maxPower = Math.max(Math.abs(stickY) + Math.abs(stickX) + Math.abs(stickRotation), 1);

                        double leftFrontPower = (stickY + stickX + stickRotation) / maxPower * 0.55; // very slow speed
                        double leftBackPower = (stickY - stickX + stickRotation) / maxPower * 0.55;
                        double rightFrontPower = (stickY - stickX - stickRotation) / maxPower * 0.55;
                        double rightBackPower = (stickY + stickX - stickRotation) / maxPower * 0.55;

                        robot.setDriveMotors(new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        robot.writeToTelemetry("Specimen Slide At", typedRobot.getSpecimenSlideHeight());
                        robot.writeToTelemetry("Specimen Slide Target", holdingSpecimenSlidePos);
                        robot.writeToTelemetry("Former Square", previousGamepadOne.square);
                        robot.writeToTelemetry("Current Square", currentGamepadOne.square);
                        robot.updateTelemetry();

                        if (currentGamepadOne.triangle && !previousGamepadOne.triangle){
                            // abort
                            runningTeleonomousAction = null;
                            actionRunning = ActionRunning.STOP;
                            drive = null;
                            break;
                        }
                    }

                    // done tuning now GO
                    if (actionRunning == ActionRunning.TO_BAR){
                        runningTeleonomousAction = chamberHookAndReturnFactory();
                        actionRunning = ActionRunning.TO_PLAYER;
                    } else if (actionRunning == ActionRunning.TO_PLAYER) {
                        runningTeleonomousAction = driveToChamberFactory();
                        actionRunning = ActionRunning.TO_BAR;
                    } else {
                        runningTeleonomousAction = null;
                        actionRunning = null;
                    }
                    holdingSpecimenSlidePos = null;
                    continue;
                }

                previousGamepadOne.copy(currentGamepadOne);
                previousGamepadTwo.copy(currentGamepadTwo);
                continue;
            }

            updateButtons();
            //updateSwitchState(robot.getSwitchState()); temporarily removed
            robot.writeToTelemetry("Current Orientation Mode", orientationMode);
            robot.writeToTelemetry("Slides In Manual Mode", manualVerticalMode);

            double speedNow = constants.currentSpeedMode.getNumericalSpeed();

            float stickX = gamepad1.left_stick_x * tmp_deadzoneadjust;
            float stickY = -gamepad1.left_stick_y * tmp_deadzoneadjust;
            float stickRotation = gamepad1.right_stick_x * tmp_deadzoneadjust;

            double directionRotation = 0;
            if (orientationMode == DriveMode.FIELD) {
                directionRotation = -Pose.normalizeAngle(robot.getImuAngle() - referenceAngle);
            }

            Pose rotatedPosition = Pose.rotatePosition(stickX, stickY, directionRotation);
            double rotatedStickX = rotatedPosition.getX();
            double rotatedStickY = rotatedPosition.getY();
            double orientation = robot.getImuAngle();
            robot.writeToTelemetry("IMU DATA (rads)", orientation);
            robot.writeToTelemetry("Reference Angle (rads)", constants.ROBOT_HEADING);

            double maxPower = Math.max(Math.abs(stickY) + Math.abs(stickX) + Math.abs(stickRotation), 1);

            double leftFrontPower = (rotatedStickY + rotatedStickX + stickRotation) / maxPower * speedNow;
            double leftBackPower = (rotatedStickY - rotatedStickX + stickRotation) / maxPower * speedNow;
            double rightFrontPower = (rotatedStickY - rotatedStickX - stickRotation) / maxPower * speedNow;
            double rightBackPower = (rotatedStickY + rotatedStickX - stickRotation) / maxPower * speedNow;

            robot.writeToTelemetry("LeftMotorPower", leftFrontPower);
            robot.writeToTelemetry("LeftBackPower", leftBackPower);
            robot.writeToTelemetry("RightFrontPower", rightFrontPower);
            robot.writeToTelemetry("RightBackPower", rightBackPower);
            robot.writeToTelemetry("Current Speed Mode", constants.currentSpeedMode);

            robot.setDriveMotors(new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // we can leave this in base robot because checks occur when adding actions
            List<Action> continued = new ArrayList<>();
            for(Action action : running){
                if(action.run(new TelemetryPacket())) continued.add(action);
            }

            previousGamepadOne.copy(currentGamepadOne);
            previousGamepadTwo.copy(currentGamepadTwo);

            // update limelight imu data
            //robot.updateLimelightIMUData();
            //Pose3D robotPos = robot.getLimelightPositionalData();
            //robot.writeRobotPositionToTelemetry(robotPos.getPosition().toUnit(DistanceUnit.INCH).x, robotPos.getPosition().toUnit(DistanceUnit.INCH).y);
            //robot.writeToTelemetry("Limelight Reported Alpha", robotPos.getOrientation().getYaw(AngleUnit.RADIANS));

            // After this, can use SeasonalRobot
            if(typedRobot == null) {
                previousGamepadOne.copy(currentGamepadOne);
                previousGamepadTwo.copy(currentGamepadTwo);
                running = continued;
                robot.updateTelemetry();
                continue;
            }

            // horizontal arm (gp 1)
            double val = currentGamepadOne.right_trigger - currentGamepadOne.left_trigger;
            robot.writeToTelemetry("Horizontal Arm Power", val);
            typedRobot.setHorizontalArmPower(val);

            // vertical arm (gp 2)
            float a = currentGamepadTwo.right_trigger - currentGamepadTwo.left_trigger;
            robot.writeToTelemetry("Vertical Arm Power", a);

            // specimen slide ctrl (gp2)
            if (runningSpecimenSlideAction == null){
                // either in manual or need to hold our height
                if (manualVerticalMode){
                    // allow manual controls
                    if (!ascentMode){
                        //LimiterState lim = typedRobot.getSpecimenSlideLimiterState();
                        LimiterState lim = LimiterState.NONE;
                        robot.writeToTelemetry("Limiter State", lim);
                        //if(a == 0) a = 0.005f;
                        typedRobot.setSpecimenSlideMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        if(lim == LimiterState.LOW) typedRobot.setSpecimenSlidePower(Math.max(0, a));
                        if(lim == LimiterState.HIGH) typedRobot.setSpecimenSlidePower(Math.min(0, a));
                        if(lim == LimiterState.NONE) typedRobot.setSpecimenSlidePower(a);
                    }
                } else {
                    if (holdingSpecimenSlidePos != null){
                        // hold height
                        typedRobot.setSpecimenSlideTargetPos(holdingSpecimenSlidePos);
                        typedRobot.setSpecimenSlidePower(1);
                        typedRobot.setSpecimenSlideMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else {
                        typedRobot.setSpecimenSlidePower(0);
                    }
                }
            } else {
                // run action. clear if done
                if(!runningSpecimenSlideAction.run(new TelemetryPacket())) runningSpecimenSlideAction = null;
                // save our height to keep at
                holdingSpecimenSlidePos = typedRobot.getSpecimenSlidePos();
            }

            // rear slide ctrl (gp2)
            // These limiters are prone to drift maybe? Depends how precise motors are. Investigate
            // If drivers report issues, look into making a reset button.
            if (manualVerticalMode && ascentMode) {
                //LimiterState lim = typedRobot.getRearVerticalSlideLimiterState();
                LimiterState lim = LimiterState.NONE;
                robot.writeToTelemetry("Limiter State", lim);
                // at lower limit: only allow positive speeds
                if(lim == LimiterState.LOW) typedRobot.setRearVerticalArmPower(Math.max(0, a));
                // at upper limit: only allow negative speeds
                if(lim == LimiterState.HIGH) typedRobot.setRearVerticalArmPower(Math.min(0, a));
                if(lim == LimiterState.NONE) typedRobot.setRearVerticalArmPower(a);
            }

            ColorResult lastResult = processColorSensorResult(typedRobot.getColorSensorColor());
            lastResultsArr.remove(0);
            lastResultsArr.add(lastResult);
            robot.writeToTelemetry("Color Sensor Color", lastResult.highestColor);
            robot.writeToTelemetry("Color Sensor Strength", lastResult.highestColorValue);

            if (intakeRunning){
                if (pickingUp){
                    if (typedRobot.getColorSensorProximity(DistanceUnit.MM) < 25) { // have one
                        Color consensusColor = Color.UNKNOWN;
                        boolean consensusGoodReading = true;
                        for (ColorResult res : lastResultsArr) {
                            if (res == null) { consensusGoodReading = false; break; } // safety case
                            if (consensusColor == Color.UNKNOWN) consensusColor = res.highestColor;
                            // .002 is a bit higher than the field consistently reads.
                            if (res.highestColorValue < .002) { consensusGoodReading = false; break; }
                            // Want all colors to match
                            if (res.highestColor != consensusColor) { consensusGoodReading = false; break; }
                        }

                        if (consensusGoodReading){ // otherwise defer judgement
                            if (
                                (consensusColor == Color.BLUE && teamColor == TeamColor.BLUE) ||
                                (consensusColor == Color.RED && teamColor == TeamColor.RED)   ||
                                (consensusColor == Color.GREEN && allowBaskets)
                            ){
                                // sample we want
                                typedRobot.stopIntake();
                                typedRobot.setWristPosition(WristPosition.HIGH);
                                intakeRunning = false;
                                gamepad1.rumble(500);
                            } else {
                                // wrong color; reject
                                expellingBad = true;
                                pickingUp = false;
                                typedRobot.reverseIntake();
                            }
                        }
                    }
                } else {
                    if (typedRobot.getColorSensorProximity(DistanceUnit.MM) > 40) {
                        if (!expellingBad){
                            typedRobot.stopIntake();
                            typedRobot.setWristPosition(WristPosition.HIGH);
                            intakeRunning = false;
                        } else {
                            expellingBad = false;
                            pickingUp = true;
                            typedRobot.forwardIntake();
                        }
                    }
                }
            }

            robot.writeToTelemetry("Color Sensor Prox", typedRobot.getColorSensorProximity(DistanceUnit.MM));
            robot.writeToTelemetry("Intake On", intakeRunning);
            robot.writeToTelemetry("Intake Expelling", expellingBad);
            robot.writeToTelemetry("Intake Intaking", pickingUp);

            previousGamepadOne.copy(currentGamepadOne);
            previousGamepadTwo.copy(currentGamepadTwo);
            running = continued;
            robot.updateTelemetry();
        }
    }

    private boolean specimenClawAtPickup = false;
    private void updateButtons() {
        // put button actions here in this format

        // speed ctrls (gp 1)
        if (currentGamepadOne.dpad_right && !previousGamepadOne.dpad_right) {
            constants.currentSpeedMode = constants.SPEEDS.FAST;
        }
        if (currentGamepadOne.dpad_up && !previousGamepadOne.dpad_up) {
            constants.currentSpeedMode = constants.SPEEDS.NORMAL;
        }
        if (currentGamepadOne.dpad_left && !previousGamepadOne.dpad_left) {
            constants.currentSpeedMode = constants.SPEEDS.SLOW;
        }
        if (currentGamepadOne.dpad_down && !previousGamepadOne.dpad_down && robot.isDashboardEnabled()) {
            constants.currentSpeedMode = constants.SPEEDS.CUSTOM_FTC_DASHBOARD;
        }

        // After this, can use SeasonalRobot
        if (typedRobot == null) return;

        // wall specimen grabber ctrl back (gp 2)
        if (currentGamepadTwo.circle && !previousGamepadTwo.circle) {
            if (specimenClawAtPickup){
                typedRobot.closeSpecimenClaw();
                sleep(100); // this is unorthodox, but it stops drivers leaving before closed
                typedRobot.specimenArmToHook();
                specimenClawAtPickup = false;
            } else {
                typedRobot.specimenArmToPickup();
                sleep(100);
                typedRobot.openSpecimenClaw();
                specimenClawAtPickup = true;
            }
        }

        // wall specimen grabber ctrl front (gp 2)
        if (currentGamepadTwo.square && !previousGamepadTwo.square) {
            if (specimenClawAtPickup){
                typedRobot.closeSpecimenClaw();
                sleep(100); // this is unorthodox, but it stops drivers leaving before closed
                typedRobot.specimenArmToHookAuto();
                specimenClawAtPickup = false;
            } else {
                typedRobot.specimenArmToPickupAuto();
                sleep(100);
                typedRobot.openSpecimenClaw();
                specimenClawAtPickup = true;
            }
        }

        // high basket angle (gp2)
        if (allowBaskets && currentGamepadTwo.x && !previousGamepadTwo.x) {
            if (specimenClawAtPickup){
                typedRobot.closeSpecimenClaw();
                sleep(100); // this is unorthodox, but it stops drivers leaving before closed
                typedRobot.specimenArmToHighBasket();
                specimenClawAtPickup = false;
            } else {
                typedRobot.specimenArmToPickupAuto();
                sleep(100);
                typedRobot.openSpecimenClaw();
                specimenClawAtPickup = true;
            }
        }

        // ascent ctrls (gp2) (rising edge)
        if (!currentGamepadTwo.ps && previousGamepadTwo.ps){
            if(ascentMode){
                typedRobot.setRearVerticalArmPower(-0.6); // climb
                while (opModeIsActive() && gamepad2.ps); // wait until button not pressed
                while (opModeIsActive()) {
                    if (gamepad2.ps) break;
                    typedRobot.writeToTelemetry("ASCENDING", "");
                    typedRobot.updateTelemetry();
                } // run until abort
                typedRobot.setRearVerticalArmPower(0);
                ascentMode = false;
                manualVerticalMode = false;
                while (opModeIsActive() && gamepad2.ps);
            } else {
                // enter ascent mode: control change
                ascentMode = true;
                manualVerticalMode = true; // no auto controls because slide issue
                typedRobot.setSpecimenSlidePower(0);
                typedRobot.specimenArmToHang();
                gamepad2.rumble(300);
                //if (!manualVerticalMode) typedRobot.raiseRearVerticalArmsToHeightAsync(0.5); // need to dial value
            }
        }

        // intake automation (gp1)
        if (currentGamepadOne.left_bumper && !previousGamepadOne.left_bumper){
            if (intakeRunning){
                typedRobot.setWristPosition(WristPosition.HIGH);
                // abort case todo needs work?
                typedRobot.stopIntake();
                intakeRunning = false;
                expellingBad = false; // clear state
            } else {
                typedRobot.setWristPosition(WristPosition.LOW);
                if(typedRobot.getColorSensorProximity(DistanceUnit.MM) > 25){ // typically rests ~20mm? away; floor ~44mm
                    // don't have one, so...
                    typedRobot.forwardIntake();
                    intakeRunning = true;
                    pickingUp = true; // save state for later
                } else {
                    // have one that needs to LEAVE
                    typedRobot.reverseIntake();
                    intakeRunning = true;
                    pickingUp = false;
                }
            }
        }

        // toggle manual verticals (gp2)
        if (currentGamepadTwo.options && !previousGamepadTwo.options){
            manualVerticalMode = !manualVerticalMode;
            if (manualVerticalMode){
                // reset powers to 0 to avoid state where slides are stuck at power lvl
                typedRobot.setRearVerticalArmPower(0);
                typedRobot.setSpecimenSlidePower(0);
                if(runningSpecimenSlideAction != null) {
                    runningSpecimenSlideAction.interrupt();
                }
                runningSpecimenSlideAction = null;
                holdingSpecimenSlidePos = null;
            }
        }

        // down (gp 2)
        if (!manualVerticalMode && !ascentMode && currentGamepadTwo.dpad_down && !previousGamepadTwo.dpad_down){
            if(runningSpecimenSlideAction != null) runningSpecimenSlideAction.interrupt();
            runningSpecimenSlideAction = typedRobot.roadrunnerRaiseSpecimenSlideToHeight(0);
            // Don't require automatic/slide mode to allow post-ascent mode lowering
        }

        // clipped (gp 2)
        if (!manualVerticalMode && !ascentMode && currentGamepadTwo.dpad_up && !previousGamepadTwo.dpad_up){
            if(runningSpecimenSlideAction != null) runningSpecimenSlideAction.interrupt();
            runningSpecimenSlideAction = typedRobot.roadrunnerRaiseSpecimenSlideToHeight(0.75);
            // Don't require automatic/slide mode to allow post-ascent mode lowering
        }

        // pre-clip (gp 2)
        if (!manualVerticalMode && !ascentMode && currentGamepadTwo.dpad_right && !previousGamepadTwo.dpad_right){
            if(runningSpecimenSlideAction != null) runningSpecimenSlideAction.interrupt();
            runningSpecimenSlideAction = typedRobot.roadrunnerRaiseSpecimenSlideToHeight(0.5);
            // Don't require automatic/slide mode to allow post-ascent mode lowering
        }

        // reset (gp 2)
        if (!manualVerticalMode && !ascentMode && currentGamepadTwo.triangle && !previousGamepadTwo.triangle){
            if(runningSpecimenSlideAction != null) runningSpecimenSlideAction.interrupt();
            typedRobot.resetSpecimenSlide();
        }

        // auto clip (gp2)
        if (currentGamepadOne.right_bumper && !previousGamepadOne.right_bumper){
            drive = new MecanumDrive(hardwareMap, new Pose2d(38.5, -59.5, -Math.PI / 2));
            runningTeleonomousAction = driveToChamberFactory();
            actionRunning = ActionRunning.TO_BAR;
            return;
        }
    }

    private void updateSwitchState(boolean switchState) {
        if (switchState) {
            // if no switch is attached, fall back to robot mode.
            orientationMode = DriveMode.ROBOT;
        } else {
            orientationMode = DriveMode.FIELD;
        }
    }

    static class ColorResult {
        Color highestColor;
        float highestColorValue;
    }
    private static ColorResult processColorSensorResult(NormalizedRGBA result){
        ColorResult out = new ColorResult();

        // todo can this logic be bettered?
        if (result.blue > Math.max(result.red, result.green)){
            out.highestColor = Color.BLUE;
            out.highestColorValue = result.blue;
        } else if (result.red > Math.max(result.blue, result.green)){
            out.highestColor = Color.RED;
            out.highestColorValue = result.red;
        } else if (result.green > Math.max(result.blue, result.red)) {
            out.highestColor = Color.GREEN;
            out.highestColorValue = result.green;
        }

        return out;
    }

    private Action driveToChamberFactory(){
        Vector2d target = endLoopPoses[posesIndex];
        posesIndex++;
        if (posesIndex == endLoopPoses.length) posesIndex = 0;

        Action humanPlayerToChamber = drive.actionBuilder(drive.pose)
                .strafeTo(target).build();

        return new SequentialAction(
                new InstantAction(typedRobot::closeSpecimenClaw),
                new SleepAction(0.25),
                new ParallelAction(
                        new SequentialAction(
                                new InstantAction(typedRobot::specimenArmToHookAuto),
                                new SleepAction(.3)
                        ),
                        humanPlayerToChamber,
                        new SequentialAction(
                                typedRobot.roadrunnerRaiseSpecimenSlideToHeightBugged(0.5),
                                new InstantAction(() -> saveSlideHeight(0.5))
                        )
                )
        );
    }

    private Action chamberHookAndReturnFactory(){
        Action chamberToHumanPlayer = drive.actionBuilder(drive.pose)
                .strafeTo(startLoopPose.position).build();

        return new SequentialAction(
                typedRobot.roadrunnerRaiseSpecimenSlideToHeightBugged(0.8),
                new SleepAction(.5),
                new InstantAction(typedRobot::openSpecimenClaw),
                new SleepAction(.25),
                new ParallelAction(
                        typedRobot.roadrunnerRaiseSpecimenSlideToHeightBugged(0),
                        new InstantAction(typedRobot::specimenArmToPickupAuto),
                        chamberToHumanPlayer
                )
        );
    }

    private void saveSlideHeight(double num){
        holdingSpecimenSlidePos = (int) (constants.CALIBRATED_SPECIMEN_SLIDE_HEIGHT_TICKS * num);
    }
}