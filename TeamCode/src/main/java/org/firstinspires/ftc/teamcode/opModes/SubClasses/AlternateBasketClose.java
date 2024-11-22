package org.firstinspires.ftc.teamcode.opModes.SubClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opModes.UnifiedAutonomous;

@Disabled
@Autonomous(name="Alternative Basket-Side Autonomous")
public class AlternateBasketClose extends UnifiedAutonomous {
    @Override
    public void runOpMode() {
        currentLocation = Locations.BlueFar;
        pathToFollow = Path.ALTERNATE;
        super.runOpMode();
    }
}
