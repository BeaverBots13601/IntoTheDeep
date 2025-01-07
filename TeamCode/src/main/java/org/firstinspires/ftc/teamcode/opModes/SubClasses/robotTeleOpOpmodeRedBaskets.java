package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOp;

@TeleOp(name="Red TeleOp Controls (Baskets)", group = "Competition")
public class robotTeleOpOpmodeRedBaskets extends UnifiedTeleOp {
    @Override
    public void runOpMode(){
        this.orientationMode = DriveMode.ROBOT;
        this.configurationMode = RobotConfiguration.FULL;
        this.teamColor = TeamColor.RED;
        this.allowBaskets = true;
        super.runOpMode();
    }
}