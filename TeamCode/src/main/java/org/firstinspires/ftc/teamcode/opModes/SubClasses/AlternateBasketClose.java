package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomous;

@Autonomous(name="Alternative Basket-Side Autonomous")
public class AlternateBasketClose extends UnifiedAutonomous {
    @Override
    public void runOpMode() {
        currentLocation = Locations.BlueFar;
        pathToFollow = Path.ALTERNATE;
        super.runOpMode();
    }
}
