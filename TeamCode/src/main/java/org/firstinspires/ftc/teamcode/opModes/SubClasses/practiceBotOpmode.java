package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOp;

@TeleOp(name = "Practice Bot TeleOp", group = "Testing")
public class practiceBotOpmode extends UnifiedTeleOp {
    @Override
    public void runOpMode() {
        this.orientationMode = constants.DriveMode.RESTRICTED;
        super.runOpMode();
    }
}