package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SeasonalRobot;
import org.firstinspires.ftc.teamcode.misc.Pose;

@Autonomous
public class DemoLimelightLocalizedMovementOpmode extends LinearOpMode {
    private static final Pose targetPose = new Pose(24, 24, 0);
    private Pose currentPose;

    @Override
    public void runOpMode() throws InterruptedException {
        SeasonalRobot robot = new SeasonalRobot(this);
        robot.updateLimelightIMUData(); // todo need to rotate this to satisfy ll requirements
        currentPose = robot.getLimelightPositionalData();

        waitForStart();
        while(!targetPose.isAtPoseWithinTolerance(currentPose)){
            Pose difference = new Pose(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY(), Pose.normalizeAngle(targetPose.getAngle() - currentPose.getAngle()));
            double stickRotation = 0;

            double maxPower = Math.max(Math.abs(difference.getY()) + Math.abs(difference.getX()) + Math.abs(stickRotation), 1);
            double leftFrontPower = (difference.getY() + difference.getX() + stickRotation) / maxPower;
            double leftBackPower = (difference.getY() - difference.getX() + stickRotation) / maxPower;
            double rightFrontPower = (difference.getY() - difference.getX() - stickRotation) / maxPower;
            double rightBackPower = (difference.getY() + difference.getX() - stickRotation) / maxPower;

            robot.setDriveMotors(new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}, DcMotor.RunMode.RUN_USING_ENCODER);

            robot.writeToTelemetry("LeftMotorPower", leftFrontPower);
            robot.writeToTelemetry("LeftBackPower", leftBackPower);
            robot.writeToTelemetry("RightFrontPower", rightFrontPower);
            robot.writeToTelemetry("RightBackPower", rightBackPower);
            robot.writeToTelemetry("Current At", currentPose);
            robot.writeToTelemetry("Going To", targetPose);
            robot.writeToTelemetry("Pose Difference", difference);

            robot.updateTelemetry();

            currentPose = robot.getLimelightPositionalData();
        }
    }
}
