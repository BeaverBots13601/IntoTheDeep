package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.teamcode.GamepadButtons;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanismClassManager;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.robotcontroller.teamcode.TeamColor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.function.BiConsumer;

// right-front (par0) & left-back (perp) are our drive motors for measuring (port 0 & 3 issue)
// right-rear slide & specimen slide our precise motors (port 0 & 3 issue)

public abstract class UnifiedTeleOp extends LinearOpMode {
    /** This field may be immediately changed by the switch state update. */
    protected HardwareMechanism.DriveMode orientationMode = HardwareMechanism.DriveMode.ROBOT; // override me
    protected TeamColor teamColor = TeamColor.BLUE; // override me
    protected boolean allowBaskets = false; // override me
    private BaseRobot robot;
    private Gamepad currentGamepadOne = new Gamepad();
    private Gamepad previousGamepadOne = new Gamepad();
    private Gamepad currentGamepadTwo = new Gamepad();
    private Gamepad previousGamepadTwo = new Gamepad();
    private ArrayList<HardwareMechanism> mechanisms = new ArrayList<>();

    // Manually added exceptions to bypass button duplication checks.
    // To add an exception, add the button in lowercase to the array (i.e. "left_bumper")
    private static final List<GamepadButtons> buttonDuplicationExceptions = Arrays.asList();
    public void runOpMode() {
        robot = new BaseRobot(this);

        // InitData
        HardwareMechanism.InitData data = new HardwareMechanism.InitData();
        data.allowBaskets = allowBaskets;
        data.teamColor = teamColor;
        data.driveMode = orientationMode;
        data.imuAngleRad = robot.getImuAngle();
        data.dashboardEnabled = robot.isDashboardEnabled();

        // get all the classes and instantiate & keep the ones matching HardwareMechanism
        List<Class<HardwareMechanism>> classes = HardwareMechanismClassManager.getMechanisms();
        HashSet<GamepadButtons> buttons = new HashSet<>();
        for (Class<HardwareMechanism> clazz : classes){
            try {
                HardwareMechanism mech = clazz.getDeclaredConstructor(HardwareMap.class, HardwareMechanism.InitData.class, BiConsumer.class).newInstance(hardwareMap, data, (BiConsumer<String, Object>) robot::writeToTelemetry);
                // we do this sanity checking before determining whether the class is valid to catch issues earlier in dev
                for (GamepadButtons button : mech.getUsedButtons()){
                    if (!buttons.add(button) && !buttonDuplicationExceptions.contains(button)){
                        // button is already in array & isn't in the exception list
                        throw new RuntimeException("WARNING! Duplicate button detected. Button: " + button + ". If this was intentional, you must add the button to the exception list.");
                    }
                }
                if (mech.available) mechanisms.add(mech);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        previousGamepadOne.copy(currentGamepadOne);
        previousGamepadTwo.copy(currentGamepadTwo);

        waitForStart();
        for (HardwareMechanism mechanism : mechanisms){
            mechanism.start();
        }

        while (opModeIsActive()) {
            currentGamepadOne.copy(gamepad1);
            currentGamepadTwo.copy(gamepad2);

            // There are still some situations where code may need to be placed in this loop.
            // For example, purposes which MUST use components from multiple mechanisms or need to
            // temporarily seize control of the entire control loop. That code can go here.

            // ascent ctrls (gp2) (rising edge) (Legacy but not particularly able to be ported)
            if (!currentGamepadTwo.ps && previousGamepadTwo.ps){
                try {
                    // this is honestly just terrible. But this new system isn't
                    // built to handle seizing control from all other subsystems.
                    Servo specimenFlipServo = HardwareMechanism.setUpServo(hardwareMap, "specimenFlipServo");
                    DcMotorEx specimenSlideMotor = HardwareMechanism.createDefaultMotor(hardwareMap, "specimenSlideMotor");
                    specimenSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    DcMotorEx leftRearVerticalArmMotor = HardwareMechanism.createDefaultMotor(hardwareMap, "leftRearVerticalArmMotor");
                    DcMotorEx rightRearVerticalArmMotor = HardwareMechanism.createDefaultMotor(hardwareMap, "rightRearVerticalArmMotor");
                    rightRearVerticalArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                    if (Globals.ascentMode) {
                        // climb
                        leftRearVerticalArmMotor.setPower(0.6);
                        rightRearVerticalArmMotor.setPower(0.6);
                        while (opModeIsActive() && gamepad2.ps){
                            robot.writeToTelemetry("ASCENDING", "");
                            robot.updateTelemetry();
                        } // wait until button not pressed
                        while (opModeIsActive()) {
                            if (gamepad2.ps) break;
                            robot.writeToTelemetry("ASCENDING", "");
                            robot.updateTelemetry();
                        } // run until abort
                        leftRearVerticalArmMotor.setPower(0);
                        rightRearVerticalArmMotor.setPower(0);
                        Globals.ascentMode = false;
                        Globals.manualVerticalMode = false;
                        while (opModeIsActive() && gamepad2.ps) ;
                    } else {
                        // enter ascent mode: control change
                        Globals.ascentMode = true;
                        Globals.manualVerticalMode = true; // no auto controls because slide issue
                        specimenSlideMotor.setPower(0);
                        specimenFlipServo.setPosition(0.73);
                        gamepad2.rumble(300);
                        //if (!manualVerticalMode) typedRobot.raiseRearVerticalArmsToHeightAsync(0.5); // need to dial value
                    }
                } catch (Exception ignored) {}
            }

            HardwareMechanism.RunData runData = new HardwareMechanism.RunData();
            runData.currentGamepadOne = currentGamepadOne;
            runData.previousGamepadOne = previousGamepadOne;
            runData.currentGamepadTwo = currentGamepadTwo;
            runData.previousGamepadTwo = previousGamepadTwo;
            runData.imuAngleRad = robot.getImuAngle();

            for (HardwareMechanism mechanism : mechanisms){
                mechanism.run(runData);
            }

            // update limelight imu data
            //robot.updateLimelightIMUData();
            //Pose3D robotPos = robot.getLimelightPositionalData();
            //robot.writeRobotPositionToTelemetry(robotPos.getPosition().toUnit(DistanceUnit.INCH).x, robotPos.getPosition().toUnit(DistanceUnit.INCH).y);
            //robot.writeToTelemetry("Limelight Reported Alpha", robotPos.getOrientation().getYaw(AngleUnit.RADIANS));

            previousGamepadOne.copy(currentGamepadOne);
            previousGamepadTwo.copy(currentGamepadTwo);
            robot.updateTelemetry();
        }
    }
}