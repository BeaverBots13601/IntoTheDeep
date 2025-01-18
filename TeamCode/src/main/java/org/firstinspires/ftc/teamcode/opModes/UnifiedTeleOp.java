package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.HardwareMechanism;
import org.firstinspires.ftc.teamcode.HardwareMechanismClassManager;
import org.firstinspires.ftc.teamcode.misc.Pose;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.SeasonalRobot;
import org.firstinspires.ftc.teamcode.SeasonalRobot.LimiterState;
import org.firstinspires.ftc.teamcode.rr.InterruptableAction;
import org.firstinspires.ftc.teamcode.TeamColor;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;

import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.StandardLocation;
import javax.tools.ToolProvider;

public abstract class UnifiedTeleOp extends LinearOpMode {
    /** This field may be immediately changed by the switch state update. */
    protected HardwareMechanism.DriveMode orientationMode = HardwareMechanism.DriveMode.ROBOT; // override me
    protected RobotConfiguration configurationMode = RobotConfiguration.RESTRICTED; // override me
    protected TeamColor teamColor = TeamColor.BLUE; // override me
    protected boolean allowBaskets = false;
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

    private List<Action> running = new ArrayList<>();
    private InterruptableAction runningSpecimenSlideAction = null;
    // class instead of primitive to allow nullability
    private Integer holdingSpecimenSlidePos = null;
    private ArrayList<HardwareMechanism> mechanisms = new ArrayList<>();
    // we use multiple results to reduce false negatives. 3 works well
    public void runOpMode() {
        if (configurationMode == RobotConfiguration.RESTRICTED) {
            robot = new BaseRobot(this);
        } else {
            robot = new SeasonalRobot(this);
            typedRobot = (SeasonalRobot) robot;
        }

        // InitData
        HardwareMechanism.InitData data = new HardwareMechanism.InitData();
        data.allowBaskets = allowBaskets;
        data.teamColor = teamColor;
        data.driveMode = orientationMode;
        data.imuAngleRad = robot.getImuAngle();
        data.dashboardEnabled = robot.isDashboardEnabled();

        // get all the classes and instantiate & keep the ones matching HardwareMechanism
        List<Class<HardwareMechanism>> classes = HardwareMechanismClassManager.getMechanisms();
        for (Class<HardwareMechanism> clazz : classes){
            try {
                HardwareMechanism mech = clazz.getDeclaredConstructor(HardwareMap.class, HardwareMechanism.InitData.class, BiConsumer.class).newInstance(hardwareMap, data, (BiConsumer<String, Object>) robot::writeToTelemetry);
                if (mech.available) mechanisms.add(mech);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        //updateSwitchState(robot.getSwitchState()); temporarily removed
        previousGamepadOne.copy(currentGamepadOne);
        previousGamepadTwo.copy(currentGamepadTwo);

        waitForStart();
        for (HardwareMechanism mechanism : mechanisms){
            mechanism.start(robot::writeToTelemetry);
        }

        while (opModeIsActive()) {
            currentGamepadOne.copy(gamepad1);
            currentGamepadTwo.copy(gamepad2);
            updateButtons();
            //updateSwitchState(robot.getSwitchState()); temporarily removed
            robot.writeToTelemetry("Current Orientation Mode", orientationMode);
            robot.writeToTelemetry("Slides In Manual Mode", manualVerticalMode);

            // we can leave this in base robot because checks occur when adding actions
            List<Action> continued = new ArrayList<>();
            for(Action action : running){
                if(action.run(new TelemetryPacket())) continued.add(action);
            }

            HardwareMechanism.RunData runData = new HardwareMechanism.RunData();
            runData.currentGamepadOne = currentGamepadOne;
            runData.previousGamepadOne = previousGamepadOne;
            runData.currentGamepadTwo = currentGamepadTwo;
            runData.previousGamepadTwo = previousGamepadTwo;
            runData.imuAngleRad = robot.getImuAngle();

            for (HardwareMechanism mechanism : mechanisms){
                mechanism.run(runData, robot::writeToTelemetry);
            }

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

            previousGamepadOne.copy(currentGamepadOne);
            previousGamepadTwo.copy(currentGamepadTwo);
            running = continued;
            robot.updateTelemetry();
            sleep(1);
        }
    }

    private void updateButtons() {
        // put button actions here in this format

        // After this, can use SeasonalRobot
        if (typedRobot == null) return;

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

        // toggle manual verticals (gp2)
        if (currentGamepadTwo.dpad_left && !previousGamepadTwo.dpad_left){
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
    }

    private void updateSwitchState(boolean switchState) {
        if (switchState) {
            // if no switch is attached, fall back to robot mode.
            orientationMode = HardwareMechanism.DriveMode.ROBOT;
        } else {
            orientationMode = HardwareMechanism.DriveMode.FIELD;
        }
    }
}