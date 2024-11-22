package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.UnifiedTeleOp;

//@TeleOp(name="TeleOp Controls (Robot)", group = "Competition")
//@TeleOp(name="TeleOp Controls", group = "Competition")
@TeleOp(name="The Resplendently Grand Harmonized and Exquisitely Coordinated Teleoperated Framework of Supreme Unified Control, Ingeniously and Meticulously Crafted for Unprecedented and Revolutionary Remote Manipulation and Orchestration, Embodying the Pinnacle of Human-Machine Integration and Synchronized Command, Enabling Unrivaled Precision and Magnificent Coordination Across a Multifaceted and Diverse Array of Technological Apparatuses and Mechanisms, Thus Standing as a Monument to the Ultimate Convergence of Electronic and Mechanical Prowess and Mastery in the Realm of Advanced Teleoperation", group = "Competition")
public class robotTeleOpOpmode extends UnifiedTeleOp {
    @Override
    public void runOpMode(){
        this.orientationMode = DriveMode.ROBOT;
        this.configurationMode = RobotConfiguration.FULL;
        super.runOpMode();
    }
}