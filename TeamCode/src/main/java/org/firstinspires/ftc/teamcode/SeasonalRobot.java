package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/*
    - All motors have std ratio
    - Bottom servos have 24 RPM
    - Panic servo, Power servos, and upper ascent motors
*/
public class SeasonalRobot extends BaseRobot {
    private final DcMotorEx leftVerticalArmMotor;
    private final DcMotorEx rightVerticalArmMotor;
    private final CRServo horizontalArmServo;
    private final Servo wristServo;
    private final Servo clawMachineServo;
    private final Servo specimenClawServo;
    private final DcMotorEx leftAscentMotor;
    private final DcMotorEx rightAscentMotor;

    // candidate to be moved to base robot

    public SeasonalRobot(LinearOpMode opmode) {
        super(opmode, constants.WHEEL_DIAMETER, constants.ROBOT_DIAMETER);
        // setup specialized stuff
        leftVerticalArmMotor = createDefaultMotor("leftVerticalArmMotor");
        rightVerticalArmMotor = createDefaultMotor("rightVerticalArmMotor");
        horizontalArmServo = opmode.hardwareMap.get(CRServo.class, "horizontalArmServo");
        wristServo = setUpServo("wristServo");
        clawMachineServo = setUpServo("clawMachineServo");
        specimenClawServo = setUpServo("specimenClawServo");
        leftAscentMotor = createDefaultMotor("leftUpperAscentMotor");
        rightAscentMotor = createDefaultMotor("rightUpperAscentMotor");


        //wristServo.setPosition(0.31); // angles down. NOTE this breaks the Shark-2 horizontal servo
        // for some ungodly reason. It just makes it spin constantly. Not anyones fault (except ftc maybe)
        // do not comment this in unless something has changed.
    }
    /*
    This is where all non-standard hardware components should be initialized, stored, and gotten.
    For example, if there is a servo that moves a piece to put a scoring component where it needs to go, but we
    won't need that next year probably, put it here.
    */

    private enum wristPos {
        UP,
        MID,
        DOWN
    }
    private wristPos currentPos = wristPos.DOWN;

    public void rotateWristDown(){
        if(currentPos == wristPos.UP){
            wristServo.setPosition(0.31 / 2); // 45 DEG
            currentPos = wristPos.MID;
        } else {
            wristServo.setPosition(0.31); // 90 DEG
            currentPos = wristPos.DOWN;
        }
    }

    public void rotateWristUp(){
        wristServo.setPosition(0);
        currentPos = wristPos.UP;
        // 0 has been mechanically-aligned to be facing mostly forward on the bot (thx philip)
    }

    public void openClawMachine(){
        clawMachineServo.setPosition(0);
    }

    public void closeClawMachine(){
        clawMachineServo.setPosition(0.5);
    }

    /**
     * Raise the Vertical Arm to its maximum height. Takes at least 550ms to end.
     */
    public void raiseVerticalArm(){
        ArrayList<Integer> lastFiveChanges = new ArrayList<>(5);
        float avgChange = 999;
        int lastEncoderPos = leftVerticalArmMotor.getCurrentPosition();

        leftVerticalArmMotor.setPower(0.15);
        rightVerticalArmMotor.setPower(0.15);

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
     * Lower the Vertical Arm to its minimum height. Takes at least 550ms to end.
     */
    public void lowerVerticalArm(){
        ArrayList<Integer> lastFiveChanges = new ArrayList<>(5);
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
     * Raises the arms to a specific height. Might not work the best?
     * @param height The height to raise it to, in a percentage. [0, 1]
     */
    public void raiseVerticalArmsToHeight(double height){
        DcMotor.RunMode before = leftVerticalArmMotor.getMode();

        rightVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_VERTICALS_HEIGHT_TICKS * height));
        leftVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_VERTICALS_HEIGHT_TICKS * height));

        leftVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftVerticalArmMotor.setPower(0.10);
        rightVerticalArmMotor.setPower(0.10);

        while (rightVerticalArmMotor.isBusy() && opMode.opModeIsActive());

        leftVerticalArmMotor.setPower(0);
        rightVerticalArmMotor.setPower(0);

        leftVerticalArmMotor.setMode(before);
        rightVerticalArmMotor.setMode(before);
    }

    public void setVerticalArmPower(float speed){
        float limitedSpeed = Math.min(Math.max(speed, -0.40f), 0.40f); // primary motors
        leftVerticalArmMotor.setPower(limitedSpeed);
        rightVerticalArmMotor.setPower(limitedSpeed);
    }

    /**
     * Changes the speed of the horizontal arm servo. Bear in mind the maximum extension distance before damage.
     */
    public void setHorizontalArmPower(float speed){
        // todo needs some way to set/limit distance
        horizontalArmServo.setPower(speed);
    }

    public void closeSpecimenClaw(){
        // todo these numbers need tweaking (talk to philip)
        specimenClawServo.setPosition(.5);
    }

    public void openSpecimenClaw(){
        specimenClawServo.setPosition(.2);
    }

    public void driveAscentToCompletion(){
        leftAscentMotor.setPower(1);
        rightAscentMotor.setPower(1);
    }
}
