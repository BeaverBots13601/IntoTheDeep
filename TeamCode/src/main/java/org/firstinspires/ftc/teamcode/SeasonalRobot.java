package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

public class SeasonalRobot extends BaseRobot {
    private final DcMotorEx verticalArmMotor;
    private final DcMotorEx horizontalArmMotor;

    public SeasonalRobot(LinearOpMode opmode) {
        super(opmode, constants.WHEEL_DIAMETER, constants.ROBOT_DIAMETER);
        // setup specialized stuff
        verticalArmMotor = createDefaultMotor("VerticalArmMotor");
        horizontalArmMotor = createDefaultMotor("horizontalArmMotor");
    }
    /*
    This is where all non-standard hardware components should be initialized, stored, and gotten.
    For example, if there is a servo that moves a piece to put a scoring component where it needs to go, but we
    won't need that next year probably, put it here.
    */

    /**
     * Raise the Vertical Arm to its maximum height. Takes at least 550ms to end.
     * TODO these might have to be refactored to allow arbitrary heights within Auto :(
     */
    public void raiseVerticalArm(){
        ArrayList<Integer> lastFiveChanges = new ArrayList<Integer>(5);
        float avgChange = 999;
        int lastEncoderPos = verticalArmMotor.getCurrentPosition();

        verticalArmMotor.setPower(0.75);

        // run motor as long as not stopped
        while(avgChange >= 5){
            avgChange = 0;
            while(lastFiveChanges.size() < 5){
                // get 5 starting values
                lastFiveChanges.add(Math.abs(verticalArmMotor.getCurrentPosition() - lastEncoderPos));
                lastEncoderPos = verticalArmMotor.getCurrentPosition();
                opMode.sleep(100);
            }
            opMode.sleep(50);
            // take out the oldest one & add in a new one
            lastFiveChanges.remove(0);
            lastFiveChanges.add(Math.abs(verticalArmMotor.getCurrentPosition() - lastEncoderPos));
            lastEncoderPos = verticalArmMotor.getCurrentPosition();
            for (int num : lastFiveChanges) { avgChange += num; }
            avgChange /= lastFiveChanges.size();
        }

        verticalArmMotor.setPower(0);
    }

    /**
     * Lower the Vertical Arm to its minimum height. Takes at least 550ms to end.
     */
    public void lowerVerticalArm(){
        ArrayList<Integer> lastFiveChanges = new ArrayList<Integer>(5);
        float avgChange = 999;
        int lastEncoderPos = verticalArmMotor.getCurrentPosition();

        verticalArmMotor.setPower(-0.75);

        // run motor as long as not stopped
        while(avgChange >= 5){
            avgChange = 0;
            while(lastFiveChanges.size() < 5){
                // get 5 starting values
                lastFiveChanges.add(Math.abs(verticalArmMotor.getCurrentPosition() - lastEncoderPos));
                lastEncoderPos = verticalArmMotor.getCurrentPosition();
                opMode.sleep(100);
            }
            opMode.sleep(50);
            // take out the oldest one & add in a new one
            lastFiveChanges.remove(0);
            lastFiveChanges.add(Math.abs(verticalArmMotor.getCurrentPosition() - lastEncoderPos));
            lastEncoderPos = verticalArmMotor.getCurrentPosition();
            for (int num : lastFiveChanges) { avgChange += num; }
            avgChange /= lastFiveChanges.size();
        }

        verticalArmMotor.setPower(0);
    }
}
