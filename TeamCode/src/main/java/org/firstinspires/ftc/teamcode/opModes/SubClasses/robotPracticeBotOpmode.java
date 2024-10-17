package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOp;

@TeleOp(name = "Practice Bot TeleOp (Robot)", group = "Testing")
public class robotPracticeBotOpmode extends UnifiedTeleOp {
    @Override
    public void runOpMode() {
        this.orientationMode = DriveMode.ROBOT;
        this.configurationMode = RobotConfiguration.RESTRICTED;
        super.runOpMode();
    }
}