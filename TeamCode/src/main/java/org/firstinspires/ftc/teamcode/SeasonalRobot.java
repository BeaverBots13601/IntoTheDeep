package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class SeasonalRobot extends BaseRobot {
    private final DcMotorEx leftRearVerticalArmMotor;
    private final DcMotorEx rightRearVerticalArmMotor;
    private final DcMotorEx horizontalArmMotor;
    private final Servo wristServo;
    private final Servo specimenClawServo;
    private final CRServo intakeServo;

    // candidate to be moved to base robot

    public SeasonalRobot(LinearOpMode opmode) {
        super(opmode, constants.WHEEL_DIAMETER, constants.ROBOT_DIAMETER);
        // setup specialized stuff
        leftRearVerticalArmMotor = createDefaultMotor("leftRearVerticalArmMotor");
        rightRearVerticalArmMotor = createDefaultMotor("rightRearVerticalArmMotor");
        horizontalArmMotor = createDefaultMotor("horizontalArmMotor");
        wristServo = setUpServo("wristServo");
        wristServo.setPosition(0.31 / 2); // todo okay?
        specimenClawServo = setUpServo("specimenClawServo");
        intakeServo = opmode.hardwareMap.get(CRServo.class, "intakeServo");
        intakeServo.setPower(0); // remove potential floating state

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

    public void toggleIntake(){
        intakeServo.setPower(intakeServo.getPower() == 0 ? 1 : 0);
    }

    public void reverseIntakeDirection(){
        if (intakeServo.getDirection() == DcMotorSimple.Direction.FORWARD) {
            intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        }

    }

    /**
     * Raises the arms to a specific height. Might not work the best?
     * @param height The height to raise it to, in a percentage. [0, 1]
     */
    public void raiseRearVerticalArmsToHeight(double height){
        DcMotor.RunMode before = leftRearVerticalArmMotor.getMode();

        rightRearVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS * height));
        leftRearVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS * height));

        leftRearVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRearVerticalArmMotor.setPower(0.20);
        rightRearVerticalArmMotor.setPower(0.20);

        while (rightRearVerticalArmMotor.isBusy() && opMode.opModeIsActive());

        leftRearVerticalArmMotor.setPower(0);
        rightRearVerticalArmMotor.setPower(0);

        leftRearVerticalArmMotor.setMode(before);
        rightRearVerticalArmMotor.setMode(before);
    }

    private Thread currentAction = null;
    public void raiseRearVerticalArmsToHeightAsync(double height){
        if(currentAction != null && currentAction.isAlive()){
            currentAction.interrupt(); // interrupt current movement if running
        }

        currentAction = new Thread(() -> {
            DcMotor.RunMode before = leftRearVerticalArmMotor.getMode();

            rightRearVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS * height));
            leftRearVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS * height));

            leftRearVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftRearVerticalArmMotor.setPower(0.20);
            rightRearVerticalArmMotor.setPower(0.20);

            while (rightRearVerticalArmMotor.isBusy() && opMode.opModeIsActive() && !Thread.currentThread().isInterrupted());

            leftRearVerticalArmMotor.setPower(0);
            rightRearVerticalArmMotor.setPower(0);

            leftRearVerticalArmMotor.setMode(before);
            rightRearVerticalArmMotor.setMode(before);
        });

        currentAction.start();
    }

    public void setRearVerticalArmPower(double speed){
        double limitedSpeed = Math.min(Math.max(speed, -0.60), 0.60); // primary motors
        leftRearVerticalArmMotor.setPower(limitedSpeed);
        rightRearVerticalArmMotor.setPower(limitedSpeed);
    }

    /**
     * Changes the speed of the horizontal arm servo. Bear in mind the maximum extension distance before damage.
     */
    public void setHorizontalArmPower(double speed){
        // todo needs some way to set/limit distance
        horizontalArmMotor.setPower(speed);
    }

    public void closeSpecimenClaw(){
        specimenClawServo.setPosition(.5);
    }

    public void openSpecimenClaw(){
        // 1 is hardware aligned to be open
        specimenClawServo.setPosition(.9);
    }
}
