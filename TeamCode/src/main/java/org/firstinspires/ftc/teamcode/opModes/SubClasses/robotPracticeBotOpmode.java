package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOp;

@TeleOp(name = "Practice Bot TeleOp (Robot)", group = "Testing")
@Disabled
public class robotPracticeBotOpmode extends UnifiedTeleOp {
    @Override
    public void runOpMode() {
        this.orientationMode = HardwareMechanism.DriveMode.ROBOT;
        this.configurationMode = RobotConfiguration.RESTRICTED;
        super.runOpMode();
    }
}