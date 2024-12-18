package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// right-front (par0) & left-back (perp) are our drive motors for measuring (port 0 & 3 issue)
// right-rear slide & specimen slide our precise motors (port 0 & 3 issue)

public class SeasonalRobot extends BaseRobot {
    private final DcMotorEx leftRearVerticalArmMotor;
    private final DcMotorEx rightRearVerticalArmMotor;
    private final DcMotorEx horizontalArmMotor;
    private final DcMotorEx specimenSlideMotor;
    private final Servo specimenClawServo;
    private final Servo specimenFlipServo;
    private final Servo wristServo;
    private final CRServo leftRotationServo;
    private final CRServo rightRotationServo;

    // candidate to be moved to base robot

    public SeasonalRobot(LinearOpMode opmode) {
        super(opmode, constants.WHEEL_DIAMETER, constants.ROBOT_DIAMETER);
        // setup specialized stuff
        leftRearVerticalArmMotor = createDefaultMotor("leftRearVerticalArmMotor");
        leftRearVerticalArmMotor.setDirection(DcMotorSimple.Direction.FORWARD); // hardware thing
        rightRearVerticalArmMotor = createDefaultMotor("rightRearVerticalArmMotor");
        rightRearVerticalArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalArmMotor = createDefaultMotor("horizontalArmMotor");
        horizontalArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        specimenClawServo = setUpServo("specimenClawServo");
        specimenSlideMotor = createDefaultMotor("specimenSlideMotor");
        specimenSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        specimenFlipServo = setUpServo("specimenFlipServo");
        wristServo = setUpServo("wristServo");
        leftRotationServo = opmode.hardwareMap.get(CRServo.class, "rotationServo");
        leftRotationServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRotationServo = opmode.hardwareMap.get(CRServo.class, "rightRotationServo");
        openSpecimenClaw();
    }
    /*
    This is where all non-standard hardware components should be initialized, stored, and gotten.
    For example, if there is a servo that moves a piece to put a scoring component where it needs to go, but we
    won't need that next year probably, put it here.
    */

//    public void toggleIntake(){
//        intakeServo.setPower(intakeServo.getPower() == 0 ? 0.5 : 0);
//    }

//    public void reverseIntakeDirection(){
//        if (intakeServo.getDirection() == DcMotorSimple.Direction.FORWARD) {
//            intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
//        } else {
//            intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//        toggleIntake();
//        toggleIntake();
//    }

    public void raiseRearVerticalArmsToHeight(double height){
        raiseRearVerticalsToHeightInternal(height, false);
    }

    public void raiseRearVerticalArmsToHeightAsync(double height){
        raiseRearVerticalsToHeightInternal(height, true);
    }

    public Action roadrunnerMoveRearVerticalSlidesToHeight(double height){
        return new Action() {
            private boolean initialized = false;
            private DcMotor.RunMode before;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    before = leftRearVerticalArmMotor.getMode();

                    rightRearVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS * height));
                    leftRearVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS * height));

                    leftRearVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightRearVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftRearVerticalArmMotor.setPower(0.40); // human-controlled uses 70%, see about boosting
                    rightRearVerticalArmMotor.setPower(0.40);

                    initialized = true;
                }

                telemetryPacket.put("Motor At", leftRearVerticalArmMotor.getCurrentPosition());
                telemetryPacket.put("Motor Moving To", leftRearVerticalArmMotor.getTargetPosition());

                if(leftRearVerticalArmMotor.isBusy()) return false;

                leftRearVerticalArmMotor.setPower(0);
                rightRearVerticalArmMotor.setPower(0);

                leftRearVerticalArmMotor.setMode(before);
                rightRearVerticalArmMotor.setMode(before);

                return true;
            }
        };
    }

    private Thread currentAction = null;
    private void raiseRearVerticalsToHeightInternal(double height, boolean async){
        if(async && currentAction != null && currentAction.isAlive()){
            currentAction.interrupt(); // interrupt current movement if running
        }

        currentAction = new Thread(() -> {
            DcMotor.RunMode before = leftRearVerticalArmMotor.getMode();

            rightRearVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS * height));
            leftRearVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS * height));

            leftRearVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftRearVerticalArmMotor.setPower(0.40); // human-controlled uses 70%, see about boosting
            rightRearVerticalArmMotor.setPower(0.40);

            while (rightRearVerticalArmMotor.isBusy() && opMode.opModeIsActive() && !Thread.currentThread().isInterrupted());

            leftRearVerticalArmMotor.setPower(0);
            rightRearVerticalArmMotor.setPower(0);

            leftRearVerticalArmMotor.setMode(before);
            rightRearVerticalArmMotor.setMode(before);
        });

        if(async) { currentAction.start(); return; }
        currentAction.run();
    }

    public void interruptCurrentRearSlideTask(){
        if(currentAction != null && currentAction.isAlive()){
            currentAction.interrupt(); // interrupt current movement if running
        }
    }

    /**
     * Move the specimen (front) slide to a specific height, expressed as a fraction of total height.
     * @param height The height to move to, [0, 1]
     */
    public void raiseSpecimenSlideToHeight(double height){
        raiseSpecimenSlideToHeightInternal(height, false);
    }

    public Action roadrunnerRaiseSpecimenSlideToHeight(double height){
        return new Action() {
            private boolean initialized = false;
            private DcMotor.RunMode before;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized){
                    before = specimenSlideMotor.getMode();

                    specimenSlideMotor.setTargetPosition((int) (constants.CALIBRATED_SPECIMEN_SLIDE_HEIGHT_TICKS * height));

                    specimenSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    specimenSlideMotor.setPower(1);

                    initialized = true;
                }

                telemetryPacket.put("Motor At", specimenSlideMotor.getCurrentPosition());
                telemetryPacket.put("Motor Moving To", specimenSlideMotor.getTargetPosition());

                if(specimenSlideMotor.isBusy()) return false;

                specimenSlideMotor.setPower(0);

                specimenSlideMotor.setMode(before);

                return true;
            }
        };
    }

    /**
     * Move the specimen (front) slide to a specific height, expressed as a fraction of total height.
     * Asynchronous, will not halt execution of main loop.
     * @param height The height to move to, [0, 1]
     */
    public void raiseSpecimenSlideToHeightAsync(double height){
        raiseSpecimenSlideToHeightInternal(height, true);
    }

    private Thread currentSpecimenSlideAction = null;
    private void raiseSpecimenSlideToHeightInternal(double height, boolean async){
        if(async && currentSpecimenSlideAction != null && currentSpecimenSlideAction.isAlive()){
            currentSpecimenSlideAction.interrupt(); // interrupt current movement if running
        }

        currentSpecimenSlideAction = new Thread(() -> {
            DcMotor.RunMode before = specimenSlideMotor.getMode();

            specimenSlideMotor.setTargetPosition((int) (constants.CALIBRATED_SPECIMEN_SLIDE_HEIGHT_TICKS * height));

            specimenSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            specimenSlideMotor.setPower(1);

            while (specimenSlideMotor.isBusy() && opMode.opModeIsActive() && !Thread.currentThread().isInterrupted());

            specimenSlideMotor.setPower(0);

            specimenSlideMotor.setMode(before);
        });

        if(async) { currentSpecimenSlideAction.start(); return; }
        currentSpecimenSlideAction.run();
    }

    public void interruptCurrentSpecimenSlideTask(){
        if(currentSpecimenSlideAction != null && currentSpecimenSlideAction.isAlive()){
            currentSpecimenSlideAction.interrupt(); // interrupt current movement if running
        }
    }

    public double getSpecimenSlideHeight(){
        return (double) specimenSlideMotor.getCurrentPosition() / constants.CALIBRATED_SPECIMEN_SLIDE_HEIGHT_TICKS;
    }

    public void setRearVerticalArmPower(double speed){
        double limitedSpeed = Math.min(Math.max(speed, -0.70), 0.70); // primary motors
        leftRearVerticalArmMotor.setPower(limitedSpeed);
        rightRearVerticalArmMotor.setPower(limitedSpeed);
    }

    public void setSpecimenSlidePower(double speed){
        specimenSlideMotor.setPower(speed);
    }

    /**
     * Changes the speed of the horizontal arm servo. Bear in mind the maximum extension distance before damage.
     */
    public void setHorizontalArmPower(double speed){
        // todo needs some way to set/limit distance
        double limitedSpeed = Math.min(Math.max(speed, -0.70), 0.70);
        horizontalArmMotor.setPower(limitedSpeed);
    }

    public void closeSpecimenClaw(){
        specimenClawServo.setPosition(.5);
    }

    public void openSpecimenClaw(){
        specimenClawServo.setPosition(.2);
    }

    public void specimenArmToPickup(){ specimenFlipServo.setPosition(0.5); }

    public void specimenArmToHook(){ specimenFlipServo.setPosition(0.7); }

    public enum LimiterState {
        // is this enum hell?
        HIGH,
        LOW,
        NONE
    }

    public LimiterState getSpecimenSlideLimiterState(){
        if(specimenSlideMotor.getCurrentPosition() < 10) return LimiterState.LOW; // negative: reversed
        if(specimenSlideMotor.getCurrentPosition() > constants.CALIBRATED_SPECIMEN_SLIDE_HEIGHT_TICKS - 10) return LimiterState.HIGH;
        return LimiterState.NONE;
    }

    public LimiterState getRearVerticalSlideLimiterState(){
        if(rightRearVerticalArmMotor.getCurrentPosition() < 10) return LimiterState.LOW; // negative: reversed
        if(rightRearVerticalArmMotor.getCurrentPosition() > constants.CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS - 10) return LimiterState.HIGH;
        return LimiterState.NONE;
    }

    public enum WristPosition {
        // todo needs tuning
        HIGH(0.75),
        MID(0.5),
        LOW(0.25);

        private final double position;

        public double getPosition(){ return position; }
        
        WristPosition(double pos){ this.position = pos; }
    }
    private WristPosition currentPos;
    public void setWristPosition(WristPosition pos){
        wristServo.setPosition(pos.getPosition());
        currentPos = pos;
    }

    public WristPosition getWristPosition(){
        return currentPos;
    }

    private double intakeSpeed = 1;
    public void reverseIntakeDirection(){
        leftRotationServo.setPower(leftRotationServo.getPower() * -1);
        rightRotationServo.setPower(rightRotationServo.getPower() * -1);
        intakeSpeed *= -1;
    }

    public void toggleIntake(){
        if(leftRotationServo.getPower() == 0){
            leftRotationServo.setPower(intakeSpeed);
            rightRotationServo.setPower(intakeSpeed);
        } else {
            leftRotationServo.setPower(0);
            rightRotationServo.setPower(0);
        }
    }
}
