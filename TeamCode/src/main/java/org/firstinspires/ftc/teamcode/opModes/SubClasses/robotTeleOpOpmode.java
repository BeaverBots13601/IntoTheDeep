package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOp;

@TeleOp(name="TeleOp Controls (Robot)", group = "Competition")
public class robotTeleOpOpmode extends UnifiedTeleOp {
    @Override
    public void runOpMode(){
        this.orientationMode = DriveMode.ROBOT;
        this.configurationMode = RobotConfiguration.FULL;
        super.runOpMode();
    }
}