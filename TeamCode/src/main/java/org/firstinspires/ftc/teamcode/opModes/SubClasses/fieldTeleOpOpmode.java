package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants;
import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOp;

@TeleOp(name = "TeleOp Controls (Field)", group = "Competition")
public class fieldTeleOpOpmode extends UnifiedTeleOp {
    @Override
    public void runOpMode(){
        this.orientationMode = constants.DriveMode.FIELD;
        super.runOpMode();
    }
}