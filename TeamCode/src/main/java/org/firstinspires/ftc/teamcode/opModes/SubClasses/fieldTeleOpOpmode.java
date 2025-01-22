package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOp;

@TeleOp(name = "TeleOp Controls (Field)", group = "Competition")
@Disabled
public class fieldTeleOpOpmode extends UnifiedTeleOp {
    @Override
    public void runOpMode(){
        this.orientationMode = HardwareMechanism.DriveMode.FIELD;
        this.configurationMode = RobotConfiguration.FULL;
        super.runOpMode();
    }
}