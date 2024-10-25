package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private final DcMotorEx leftAscentMotor;
    private final DcMotorEx rightAscentMotor;

    // candidate to be moved to base robot

    public SeasonalRobot(LinearOpMode opmode) {
        super(opmode, constants.WHEEL_DIAMETER, constants.ROBOT_DIAMETER);
        // setup specialized stuff
        leftVerticalArmMotor = createDefaultMotor("leftVerticalArmMotor");
        rightVerticalArmMotor = createDefaultMotor("rightVerticalArmMotor");
        rightVerticalArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        opmode.hardwareMap.get(CRServo.class, "horizontalArmServo").setPower(0); // terrible work around. causes twitch. todo investigate
        horizontalArmServo = opmode.hardwareMap.get(CRServo.class, "horizontalArmServo");
        wristServo = setUpServo("wristServo");
        wristServo.setPosition(0.31 / 2); // todo okay?
        clawMachineServo = setUpServo("clawMachineServo");
        specimenClawServo = setUpServo("specimenClawServo");
        leftAscentMotor = createDefaultMotor("leftAscentMotor");
        rightAscentMotor = createDefaultMotor("rightAscentMotor");


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
    private wristPos currentPos = wristPos.MID;

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

    public void setVerticalArmPower(double speed){
        double limitedSpeed = Math.min(Math.max(speed, -0.60), 0.60); // primary motors
        leftVerticalArmMotor.setPower(limitedSpeed);
        rightVerticalArmMotor.setPower(limitedSpeed);
    }

    /**
     * Changes the speed of the horizontal arm servo. Bear in mind the maximum extension distance before damage.
     */
    public void setHorizontalArmPower(double speed){
        // todo needs some way to set/limit distance
        horizontalArmServo.setPower(speed);
    }

    public void closeSpecimenClaw(){
        specimenClawServo.setPosition(.5);
    }

    public void openSpecimenClaw(){
        // 1 is hardware aligned to be open
        specimenClawServo.setPosition(1);
    }

    public void setAscentMotorSpeeds(double speed){
        leftAscentMotor.setPower(speed);
        rightAscentMotor.setPower(speed);
    }
}
