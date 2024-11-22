package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.misc.Pose;
import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.SeasonalRobot;
import org.firstinspires.ftc.teamcode.SeasonalRobot.LimiterState;

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

    public void runOpMode() {
        if (configurationMode == RobotConfiguration.RESTRICTED) {
            robot = new BaseRobot(this, constants.WHEEL_DIAMETER, constants.ROBOT_DIAMETER);
        } else {
            robot = new SeasonalRobot(this);
            typedRobot = (SeasonalRobot) robot;
        }

        updateSwitchState(robot.getSwitchState());
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
            robot.writeToTelemetry("Current Orientation Mode", orientationMode);
            robot.writeToTelemetry("Slides In Manual Mode", manualVerticalMode);

            double speedNow = constants.currentSpeedMode.getNumericalSpeed();

            float stickX = gamepad1.left_stick_x * tmp_deadzoneadjust;
            float stickY = -gamepad1.left_stick_y * tmp_deadzoneadjust;
            float
                    stickRotation = gamepad1.right_stick_x * tmp_deadzoneadjust;

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

            robot.setDriveMotors(new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_USING_ENCODER);

            previousGamepadOne.copy(currentGamepadOne);
            previousGamepadTwo.copy(currentGamepadTwo);

            // update limelight imu data
            //robot.updateLimelightIMUData();
            //Pose3D robotPos = robot.getLimelightPositionalData();
            //robot.writeRobotPositionToTelemetry(robotPos.getPosition().toUnit(DistanceUnit.INCH).x, robotPos.getPosition().toUnit(DistanceUnit.INCH).y);
            //robot.writeToTelemetry("Limelight Reported Alpha", robotPos.getOrientation().getYaw(AngleUnit.RADIANS));

            // After this, can use SeasonalRobot
            if(typedRobot == null) { robot.updateTelemetry(); continue; }

            // horizontal arm (gp 1)
            robot.writeToTelemetry("Horizontal Arm Power", currentGamepadOne.right_trigger - currentGamepadOne.left_trigger);
            typedRobot.setHorizontalArmPower(currentGamepadOne.right_trigger - currentGamepadOne.left_trigger);

            // vertical arm (gp 2)
            float a = currentGamepadTwo.right_trigger - currentGamepadTwo.left_trigger;
            robot.writeToTelemetry("Vertical Arm Power", a);

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

            if (manualVerticalMode && !ascentMode) {
                //LimiterState lim = typedRobot.getSpecimenSlideLimiterState();
                LimiterState lim = LimiterState.NONE;
                robot.writeToTelemetry("Limiter State", lim);
                //if(a == 0) a = 0.005f;
                if(lim == LimiterState.LOW) typedRobot.setSpecimenSlidePower(Math.max(0, a));
                if(lim == LimiterState.HIGH) typedRobot.setSpecimenSlidePower(Math.min(0, a));
                if(lim == LimiterState.NONE) typedRobot.setSpecimenSlidePower(a);
            }

            robot.updateTelemetry();
        }
    }

    private boolean specimenClawOpen = true;
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
        if (currentGamepadOne.triangle && !previousGamepadOne.triangle){
            typedRobot.toggleIntake();
        }
        if (currentGamepadOne.circle && !previousGamepadOne.circle){
            typedRobot.reverseIntakeDirection();
        }

        // wall specimen grabber ctrl (gp 2)
        if (currentGamepadTwo.square && !previousGamepadTwo.square) {
            if (specimenClawOpen){
                typedRobot.closeSpecimenClaw();
                specimenClawOpen = false;
            } else {
                typedRobot.openSpecimenClaw();
                specimenClawOpen = true;
            }
        }

        // ascent ctrls (gp2) (rising edge)
        if (!currentGamepadTwo.ps && previousGamepadTwo.ps){
            if(ascentMode){
                typedRobot.setRearVerticalArmPower(-0.6); // climb
                while (opModeIsActive() && gamepad2.ps); // wait until button not pressed
                while (opModeIsActive()) { if (gamepad2.ps) break; } // run until abort
                typedRobot.setRearVerticalArmPower(0);
                ascentMode = false;
                manualVerticalMode = false;
                while (opModeIsActive() && gamepad2.ps);
            } else {
                // enter ascent mode: control change
                ascentMode = true;
                manualVerticalMode = true; // no auto controls because slide issue
                typedRobot.setSpecimenSlidePower(0);
                gamepad2.rumble(300);
                //if (!manualVerticalMode) typedRobot.raiseRearVerticalArmsToHeightAsync(0.5); // need to dial value
            }
        }

        // manual verticals (gp2)
        if (currentGamepadTwo.dpad_right && !previousGamepadTwo.dpad_right){
            manualVerticalMode = !manualVerticalMode;
            if (manualVerticalMode){
                // reset powers to 0 to avoid state where slides are stuck at power lvl
                typedRobot.interruptCurrentSpecimenSlideTask();
                typedRobot.interruptCurrentRearSlideTask();
                typedRobot.setRearVerticalArmPower(0);
                typedRobot.setSpecimenSlidePower(0);
            }
        }

        // Preset height controls (gp2) (automatic)
        if (!manualVerticalMode && !ascentMode && currentGamepadTwo.dpad_up && !previousGamepadTwo.dpad_up){
            typedRobot.interruptCurrentSpecimenSlideTask();
            typedRobot.raiseSpecimenSlideToHeightAsync(0.7); // high chamber
        }
        if (!manualVerticalMode && !ascentMode && currentGamepadTwo.dpad_left && !previousGamepadTwo.dpad_left){
            typedRobot.interruptCurrentSpecimenSlideTask();
            typedRobot.raiseSpecimenSlideToHeightAsync(0.3); // low chamber
        }
        if (currentGamepadTwo.dpad_down && !previousGamepadTwo.dpad_down){
            typedRobot.interruptCurrentSpecimenSlideTask();
            // Don't require automatic/slide mode to allow post-ascent mode lowering
            typedRobot.raiseSpecimenSlideToHeightAsync(0); // ground
        }

        // Clip specimen to chamber (gp2) (automatic)
        if (!manualVerticalMode && !ascentMode && currentGamepadTwo.x && !previousGamepadTwo.x){
            typedRobot.interruptCurrentSpecimenSlideTask();
            typedRobot.raiseSpecimenSlideToHeightAsync(Math.max(typedRobot.getSpecimenSlideHeight() - 0.2, 0)); // decrease height
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