package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.misc.Pose;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.Robot24_25;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;

@TeleOp(name="TeleOp Controls (Robot)", group = "Competition")
public class UnifiedTeleOp extends LinearOpMode {
    private constants.DriveMode orientationMode = constants.DriveMode.ROBOT;
    private Robot24_25 typedRobot;
    private BaseRobot robot;
    private Gamepad currentGamepad = new Gamepad();
    private Gamepad previousGamepad = new Gamepad();
    private DigitalChannel driveModeSwitch;

    public void runOpMode(constants.DriveMode mode) {
        if(mode == constants.DriveMode.ROBOT) { return; } // don't do this ever
        if(mode == constants.DriveMode.RESTRICTED) { orientationMode = constants.DriveMode.RESTRICTED; runOpMode(); }
        if(mode == constants.DriveMode.FIELD) { orientationMode = constants.DriveMode.FIELD; runOpMode(); }
    }

    public void runOpMode() {
        if(orientationMode == constants.DriveMode.RESTRICTED){
            robot = new BaseRobot(this, constants.WHEEL_DIAMETER, constants.ROBOT_DIAMETER);
        } else {
            robot = new Robot24_25(this);
            typedRobot = (Robot24_25) robot;
        }

        driveModeSwitch = hardwareMap.get(DigitalChannel.class, "switch");
        updateSwitchState();
        double referenceAngle;
        try {
            referenceAngle = (double) new FileReader("robotHeading.txt").read();
        } catch (IOException ignored) {
            referenceAngle = robot.getImuAngle();
        }
        int tmp_deadzoneadjust = 2;
        previousGamepad.copy(currentGamepad);

        waitForStart();
        while(opModeIsActive()){
            // if Robot24_25-specific functions are to be used here, check if typedRobot is null FIRST! (see updateButtons)

            currentGamepad.copy(gamepad1);
            updateButtons(currentGamepad);
            updateSwitchState();

            double speedNow = constants.currentSpeedMode.getNumericalSpeed();

            float stickX = gamepad1.left_stick_x * tmp_deadzoneadjust;
            float stickY = -gamepad1.left_stick_y * tmp_deadzoneadjust;
            float stickRotation = gamepad1.right_stick_x * tmp_deadzoneadjust;

            double directionRotation = 0;
            if(orientationMode == constants.DriveMode.FIELD) {
                directionRotation = -Pose.normalizeAngle(robot.getImuAngle() - referenceAngle);
            }

            Pose rotatedPosition = Pose.rotatePosition(stickX, stickY, directionRotation);
            double rotatedStickX = rotatedPosition.getX();
            double rotatedStickY = rotatedPosition.getY();
            double orientation = robot.getImuAngle();
            robot.writeToTelemetry("IMU DATA (rads):", orientation);

            double maxPower = Math.max(Math.abs(stickY) + Math.abs(stickX) + Math.abs(stickRotation), 1);

            double leftFrontPower  = (rotatedStickY + rotatedStickX + stickRotation) / maxPower * speedNow;
            double leftBackPower  = (rotatedStickY - rotatedStickX + stickRotation) / maxPower * speedNow;
            double rightFrontPower  = (rotatedStickY - rotatedStickX - stickRotation) / maxPower * speedNow;
            double rightBackPower  = (rotatedStickY + rotatedStickX - stickRotation) / maxPower * speedNow;

            robot.writeToTelemetry("LeftMotorPower", leftFrontPower);
            robot.writeToTelemetry("LeftBackPower", leftBackPower);
            robot.writeToTelemetry("RightFrontPower", rightFrontPower);
            robot.writeToTelemetry("RightBackPower", rightBackPower);
            robot.writeToTelemetry("Current Speed Mode", constants.currentSpeedMode);
            robot.writeToTelemetry("Current Orientation Mode", orientationMode);

            robot.setDriveMotors(new double[] {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_USING_ENCODER);

            robot.updateTelemetry();

            previousGamepad.copy(currentGamepad);
        }
    }
    private void updateButtons(Gamepad currentGamepad){
        // put button actions here in this format
        if(currentGamepad.dpad_up && !previousGamepad.dpad_up){
            constants.currentSpeedMode = constants.SPEEDS.FAST;
        }
        if(currentGamepad.dpad_right && !previousGamepad.dpad_right){
            constants.currentSpeedMode = constants.SPEEDS.NORMAL;
        }
        if(currentGamepad.dpad_down && !previousGamepad.dpad_down){
            constants.currentSpeedMode = constants.SPEEDS.SLOW;
        }
        if(currentGamepad.dpad_left && !previousGamepad.dpad_left && robot.isDashboardEnabled()){
            constants.currentSpeedMode = constants.SPEEDS.CUSTOM_FTC_DASHBOARD;
        }

        // After this, can use typedRobot
        if(typedRobot == null) return;
    }

    private void updateSwitchState(){
        if(orientationMode != constants.DriveMode.RESTRICTED){
            if(driveModeSwitch.getState()){
                // completely arbitrary
                orientationMode = constants.DriveMode.FIELD;
            } else {
                orientationMode = constants.DriveMode.ROBOT;
            }
        }
    }
}

@TeleOp(name = "Practice Bot TeleOp", group = "Testing")
class practiceBotOpmode extends UnifiedTeleOp {
    @Override
    public void runOpMode() {
        super.runOpMode(constants.DriveMode.RESTRICTED);
    }
}

@TeleOp(name = "TeleOp Controls (Field)", group = "Competition")
class fieldTeleOpOpmode extends UnifiedTeleOp {
    @Override
    public void runOpMode(){
        super.runOpMode(constants.DriveMode.FIELD);
    }
}