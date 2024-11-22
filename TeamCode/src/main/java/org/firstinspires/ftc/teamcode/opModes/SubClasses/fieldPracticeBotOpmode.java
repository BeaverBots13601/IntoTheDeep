package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOp;

@TeleOp(name = "Practice Bot TeleOp (Field)", group = "Testing")
@Disabled
public class fieldPracticeBotOpmode extends UnifiedTeleOp {
    @Override
    public void runOpMode() {
        this.orientationMode = DriveMode.FIELD;
        this.configurationMode = RobotConfiguration.RESTRICTED;
        super.runOpMode();
    }
}