package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismClassManager;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.SeasonalRobot;
import org.firstinspires.ftc.teamcode.LimiterState;
import org.firstinspires.ftc.teamcode.rr.InterruptableAction;
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

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

    protected enum RobotConfiguration {
        RESTRICTED,
        FULL
    }

    private List<Action> running = new ArrayList<>();
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

            previousGamepadOne.copy(currentGamepadOne);
            previousGamepadTwo.copy(currentGamepadTwo);
            running = continued;
            robot.updateTelemetry();
        }
    }

    private void updateButtons() {
        // put button actions here in this format

        // After this, can use SeasonalRobot
        if (typedRobot == null) return;

        // ascent ctrls (gp2) (rising edge)
        if (!currentGamepadTwo.ps && previousGamepadTwo.ps){
            if(Globals.ascentMode){
                typedRobot.setRearVerticalArmPower(-0.6); // climb
                while (opModeIsActive() && gamepad2.ps); // wait until button not pressed
                while (opModeIsActive()) {
                    if (gamepad2.ps) break;
                    typedRobot.writeToTelemetry("ASCENDING", "");
                    typedRobot.updateTelemetry();
                } // run until abort
                typedRobot.setRearVerticalArmPower(0);
                Globals.ascentMode = false;
                Globals.manualVerticalMode = false;
                while (opModeIsActive() && gamepad2.ps);
            } else {
                // enter ascent mode: control change
                Globals.ascentMode = true;
                Globals.manualVerticalMode = true; // no auto controls because slide issue todo uncomment
                typedRobot.setSpecimenSlidePower(0);
                typedRobot.specimenArmToHang();
                gamepad2.rumble(300);
                //if (!manualVerticalMode) typedRobot.raiseRearVerticalArmsToHeightAsync(0.5); // need to dial value
            }
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