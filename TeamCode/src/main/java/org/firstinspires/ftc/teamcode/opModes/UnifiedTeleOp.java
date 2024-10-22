package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.misc.Pose;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.SeasonalRobot;

public abstract class UnifiedTeleOp extends LinearOpMode {
    /** This field may be immediately changed by the switch state update. */
    protected DriveMode orientationMode = DriveMode.ROBOT;
    protected RobotConfiguration configurationMode = RobotConfiguration.RESTRICTED;
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
    protected enum DriveMode {
        FIELD,
        ROBOT
    }

    public void runOpMode() {
        if (configurationMode == RobotConfiguration.RESTRICTED) {
            robot = new BaseRobot(this, constants.WHEEL_DIAMETER, constants.ROBOT_DIAMETER);
        } else {
            robot = new SeasonalRobot(this);
            typedRobot = (SeasonalRobot) robot;
        }

        //updateSwitchState();
        double referenceAngle;
        if(constants.ROBOT_HEADING != 0){
            referenceAngle = constants.ROBOT_HEADING;
        } else {
            referenceAngle = robot.getImuAngle();
        }
        int tmp_deadzoneadjust = 2;
        previousGamepadOne.copy(currentGamepadOne);
        previousGamepadTwo.copy(currentGamepadTwo);

        waitForStart();
        while (opModeIsActive()) {
            currentGamepadOne.copy(gamepad1);
            currentGamepadTwo.copy(gamepad2);
            updateButtons();
            updateSwitchState(robot.getSwitchState());

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
            robot.writeToTelemetry("Current Orientation Mode", orientationMode);

            robot.setDriveMotors(new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_USING_ENCODER);

            previousGamepadOne.copy(currentGamepadOne);
            previousGamepadTwo.copy(currentGamepadTwo);

            // After this, can use SeasonalRobot
            if(typedRobot == null) continue;

            // horizontal arm (gp 2)
            robot.writeToTelemetry("Horizontal Arm Power", currentGamepadOne.right_trigger - currentGamepadOne.left_trigger);
            typedRobot.setHorizontalArmPower(currentGamepadOne.right_trigger - currentGamepadOne.left_trigger);

            // vertical arm (gp 1)
            robot.writeToTelemetry("Vertical Arm Power", currentGamepadTwo.right_trigger - currentGamepadTwo.left_trigger);
            typedRobot.setVerticalArmPower(currentGamepadTwo.right_trigger - currentGamepadTwo.left_trigger);

            //typedRobot.setPanicServoPower(currentGamepadTwo.ps ? 1 : 0);
            //ypedRobot.setPanicServoPower(currentGamepadTwo.square ? -1 : 0);

            // update limelight imu data
            typedRobot.updateLimelightIMUData();
            Position a = typedRobot.getLimelightPositionalData();
            robot.writeRobotPositionToTelemetry(a.x, a.z);

            robot.updateTelemetry();
        }
    }

    private boolean clawMachineOpen = true;
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

        // submersible grabber ctrls (gp 1)
        if (currentGamepadOne.right_bumper && !previousGamepadOne.right_bumper) {
            typedRobot.rotateWristDown();
        }
        if (currentGamepadOne.left_bumper && !previousGamepadOne.left_bumper) {
            typedRobot.rotateWristUp();
        }
        if (currentGamepadOne.triangle && !previousGamepadOne.triangle) {
            if (clawMachineOpen){
                typedRobot.closeClawMachine();
                clawMachineOpen = false;
            } else {
                typedRobot.openClawMachine();
                clawMachineOpen = true;
            }
        }

        // ascent ctrls (gp 1)
        if (currentGamepadOne.square && !previousGamepadOne.square){ // l1 ascent
            typedRobot.latchLowerAscentHooks();
            typedRobot.reelLowerAscentHooks();
        }
        if (currentGamepadOne.circle && !previousGamepadOne.circle){ // l2 ascent
            typedRobot.upperAscentMotorsToLatched();
            typedRobot.setVerticalArmPower(-0.75f);
            typedRobot.pullUpperAscentMotors();
            sleep(2500);
            typedRobot.setVerticalArmPower(0);
        }

        // wall specimen grabber ctrl (gp 2)
        if (currentGamepadTwo.options && !previousGamepadTwo.options){
            typedRobot.openSpecimenClaw();
        }
        if (currentGamepadTwo.share && !previousGamepadTwo.share){
            typedRobot.closeSpecimenClaw();
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
}