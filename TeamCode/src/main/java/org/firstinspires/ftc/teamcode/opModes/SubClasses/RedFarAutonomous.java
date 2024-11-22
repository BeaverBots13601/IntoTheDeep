package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomous;

//@Autonomous(name = "Red Far Autonomous", group = "Competition")
@Autonomous(name = "Basket Autonomous", group = "Competition")
@Disabled
public class RedFarAutonomous extends UnifiedAutonomous {
    @Override
    public void runOpMode() {
        currentLocation = Locations.RedFar;
        super.runOpMode();
    }
}