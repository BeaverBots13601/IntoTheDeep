package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomous;

//@Autonomous(name = "Red Close Autonomous", group = "Competition")
@Autonomous(name="Human Player Autonomous", group = "Competition", preselectTeleOp = "The Resplendently Grand Harmonized and Exquisitely Coordinated Teleoperated Framework of Supreme Unified Control, Ingeniously and Meticulously Crafted for Unprecedented and Revolutionary Remote Manipulation and Orchestration, Embodying the Pinnacle of Human-Machine Integration and Synchronized Command, Enabling Unrivaled Precision and Magnificent Coordination Across a Multifaceted and Diverse Array of Technological Apparatuses and Mechanisms, Thus Standing as a Monument to the Ultimate Convergence of Electronic and Mechanical Prowess and Mastery in the Realm of Advanced Teleoperation")
public class RedCloseAutonomous extends UnifiedAutonomous {
    @Override
    public void runOpMode() {
        currentLocation = Locations.RedClose;
        super.runOpMode();
    }
}