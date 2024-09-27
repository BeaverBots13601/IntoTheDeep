package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class SeasonalRobot extends BaseRobot {
    private final DcMotorEx leftVerticalArmMotor;
    private final DcMotorEx rightVerticalArmMotor;
    private final CRServo horizontalArmServo;
    private final Servo wristServo;
    private final Servo clawMachineServo;
    private final Servo specimenClawServo;
    private final CRServo leftAscentServo;
    private final CRServo rightAscentServo;

    public SeasonalRobot(LinearOpMode opmode) {
        super(opmode, constants.WHEEL_DIAMETER, constants.ROBOT_DIAMETER);
        // setup specialized stuff
        leftVerticalArmMotor = createDefaultMotor("leftVerticalArmMotor");
        rightVerticalArmMotor = createDefaultMotor("rightVerticalArmMotor");
        horizontalArmServo = opmode.hardwareMap.get(CRServo.class, "horizontalArmServo");
        wristServo = setUpServo("wristServo");
        wristServo.setPosition(0); // shouldn't move if properly set
        clawMachineServo = setUpServo("clawMachineServo");
        specimenClawServo = setUpServo("specimenClawServo");
        leftAscentServo = opmode.hardwareMap.get(CRServo.class, "leftAscentServo");
        leftAscentServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightAscentServo = opmode.hardwareMap.get(CRServo.class, "rightAscentServo");

        /*
        How to get cube from submersible:

        rotateWristUp()
        extendHorizontalArm()
        adjust to where you want
        rotateWristDown()
        closeClaw()
        rotateWristUp()
        retractHorizontalArm()
        go to where we want it
        openClaw()
         */
    }
    /*
    This is where all non-standard hardware components should be initialized, stored, and gotten.
    For example, if there is a servo that moves a piece to put a scoring component where it needs to go, but we
    won't need that next year probably, put it here.
    */

    public void rotateWristDown(){
        wristServo.setPosition(0);
        // 0 has been mechanically-aligned to be facing down on the bot (thx philip)
    }

    public void rotateWristUp(){
        wristServo.setPosition(0.31);
        // tuned to be pretty close to 90 deg
    }

    public void openClawMachine(){
        clawMachineServo.setPosition(0);
    }

    public void closeClawMachine(){
        clawMachineServo.setPosition(0.5);
    }

    /**
     * Raise the Vertical Arm to its maximum height. Takes at least 550ms to end.
     * TODO these might have to be refactored to allow arbitrary heights within Auto :(
     */
    public void raiseVerticalArm(){
        ArrayList<Integer> lastFiveChanges = new ArrayList<Integer>(5);
        float avgChange = 999;
        int lastEncoderPos = leftVerticalArmMotor.getCurrentPosition();

        leftVerticalArmMotor.setPower(0.75);
        rightVerticalArmMotor.setPower(0.75);

        // run motor as long as not stopped
        while(avgChange >= 5){
            avgChange = 0;
            while(lastFiveChanges.size() < 5){
                // get 5 starting values
                lastFiveChanges.add(Math.abs(leftVerticalArmMotor.getCurrentPosition() - lastEncoderPos));
                lastEncoderPos = leftVerticalArmMotor.getCurrentPosition();
                opMode.sleep(100);
            }
            opMode.sleep(50);
            // take out the oldest one & add in a new one
            lastFiveChanges.remove(0);
            lastFiveChanges.add(Math.abs(leftVerticalArmMotor.getCurrentPosition() - lastEncoderPos));
            lastEncoderPos = leftVerticalArmMotor.getCurrentPosition();
            for (int num : lastFiveChanges) { avgChange += num; }
            avgChange /= lastFiveChanges.size();
        }

        leftVerticalArmMotor.setPower(0);
    }

    /**
     * Lower the Vertical Arm to its minimum height. Takes at least 550ms to end.
     */
    public void lowerVerticalArm(){
        ArrayList<Integer> lastFiveChanges = new ArrayList<Integer>(5);
        float avgChange = 999;
        int lastEncoderPos = leftVerticalArmMotor.getCurrentPosition();

        leftVerticalArmMotor.setPower(-0.75);
        rightVerticalArmMotor.setPower(-0.75);

        // run motor as long as not stopped
        while(avgChange >= 5){
            avgChange = 0;
            while(lastFiveChanges.size() < 5){
                // get 5 starting values
                lastFiveChanges.add(Math.abs(leftVerticalArmMotor.getCurrentPosition() - lastEncoderPos));
                lastEncoderPos = leftVerticalArmMotor.getCurrentPosition();
                opMode.sleep(100);
            }
            opMode.sleep(50);
            // take out the oldest one & add in a new one
            lastFiveChanges.remove(0);
            lastFiveChanges.add(Math.abs(leftVerticalArmMotor.getCurrentPosition() - lastEncoderPos));
            lastEncoderPos = leftVerticalArmMotor.getCurrentPosition();
            for (int num : lastFiveChanges) { avgChange += num; }
            avgChange /= lastFiveChanges.size();
        }

        leftVerticalArmMotor.setPower(0);
        rightVerticalArmMotor.setPower(0);
    }

    /**
     * Extends the horizontal arm, up to the maximum extension length.
     * @param speed The speed to run at. No limiter. Keep in mind the maximum physical servo rotation speed.
     */
    public void extendHorizontalArm(float speed){
        // todo this doesn't work. needs some way to set distance
        horizontalArmServo.setPower(speed);
    }

    /**
     * Retracts the horizontal arm, up to the minimum extension length.
     * @param speed The speed to run at. No limiter. Keep in mind the maximum physical servo rotation speed.
     */
    public void retractHorizontalArm(float speed){
        // todo this doesn't work. needs some way to set distance
        horizontalArmServo.setPower(-speed);
    }

    public void closeSpecimenClaw(){
        // todo these numbers need tweaking (talk to philip)
        specimenClawServo.setPosition(.5);
    }

    public void openSpecimenClaw(){
        specimenClawServo.setPosition(0);
    }

    /**
     * Once positioned, latches the hooks onto L3 the bar. Takes 2.5s.
     */
    public void latchAscentHooks(){
        leftAscentServo.setPower(0.75);
        rightAscentServo.setPower(0.75);
        opMode.sleep(2500);
        leftAscentServo.setPower(0);
        rightAscentServo.setPower(0);
    }
}
