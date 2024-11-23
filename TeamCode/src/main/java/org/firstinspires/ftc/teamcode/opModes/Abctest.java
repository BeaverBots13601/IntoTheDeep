package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SeasonalRobot;

@Disabled
@Autonomous
public class Abctest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SeasonalRobot robot = new SeasonalRobot(this);

        waitForStart();

        robot.driveAtAngle(24, 180, .5);
    }
}
