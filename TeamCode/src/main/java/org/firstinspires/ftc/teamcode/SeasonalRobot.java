package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.rr.InterruptableAction;

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
    private final RevColorSensorV3 colorSensor;

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
        leftRotationServo = opmode.hardwareMap.get(CRServo.class, "leftRotationServo");
        leftRotationServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRotationServo.setPower(0);
        rightRotationServo = opmode.hardwareMap.get(CRServo.class, "rightRotationServo");
        rightRotationServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRotationServo.setPower(0);
        colorSensor = opmode.hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        openSpecimenClaw();
        specimenArmToPickup();
        setWristPosition(WristPosition.LOW);
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

    public InterruptableAction roadrunnerMoveRearVerticalSlidesToHeight(double height){
        return new InterruptableAction() {
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

                if(leftRearVerticalArmMotor.isBusy() && !interrupted) return true;

                leftRearVerticalArmMotor.setPower(0);
                rightRearVerticalArmMotor.setPower(0);

                leftRearVerticalArmMotor.setMode(before);
                rightRearVerticalArmMotor.setMode(before);

                return false;
            }
            private boolean interrupted = false;
            public void interrupt(){
                interrupted = true;
                run(new TelemetryPacket());
            }
        };
    }

    public Action roadrunnerRaiseSpecimenSlideToHeightBugged(double height){
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

                // FIXME: This is a big bug. Our auto will require too much of a rework to fix it
                // atm. Should return true; current implementation is weird and holds height.
                if(specimenSlideMotor.isBusy()) return false;

                specimenSlideMotor.setPower(0);

                specimenSlideMotor.setMode(before);

                return true;
            }
        };
    }

    public InterruptableAction roadrunnerRaiseSpecimenSlideToHeight(double height){
        return new InterruptableAction() {
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

                if(specimenSlideMotor.isBusy() && !interrupted) return true;

                specimenSlideMotor.setPower(0);

                specimenSlideMotor.setMode(before);

                return false;
            }
            private boolean interrupted = false;
            public void interrupt(){
                interrupted = false;
                run(new TelemetryPacket());
            }
        };
    }

    public Action roadrunnerExtendHorizontalSlideToLength(double dist){
        return new Action() {
            private boolean initialized = false;
            private DcMotor.RunMode before;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized){
                    before = horizontalArmMotor.getMode();

                    horizontalArmMotor.setTargetPosition((int) (constants.CALIBRATED_HORIZONTAL_SLIDE_LENGTH_TICKS * dist));

                    horizontalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    horizontalArmMotor.setPower(1);

                    initialized = true;
                }

                telemetryPacket.put("Motor At", horizontalArmMotor.getCurrentPosition());
                telemetryPacket.put("Motor Moving To", horizontalArmMotor.getTargetPosition());

                if (horizontalArmMotor.isBusy()) return true;

                horizontalArmMotor.setPower(0);

                horizontalArmMotor.setMode(before);

                return false;
            }
        };
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

    public void setSpecimenSlideMode(DcMotor.RunMode mode){
        specimenSlideMotor.setMode(mode);
    }

    public void setSpecimenSlideTargetPos(int target){ specimenSlideMotor.setTargetPosition(target); }

    public int getSpecimenSlidePos(){
        return specimenSlideMotor.getCurrentPosition();
    }

    /**
     * Changes the speed of the horizontal arm servo. Bear in mind the maximum extension distance before damage.
     */
    public void setHorizontalArmPower(double speed){
        // todo needs some way to set/limit distance
        horizontalArmMotor.setPower(speed);
    }

    public void closeSpecimenClaw(){
        specimenClawServo.setPosition(.52);
    }

    public void openSpecimenClaw(){ specimenClawServo.setPosition(.23); }

    public void specimenArmToPickup(){ specimenFlipServo.setPosition(1); }
    public void specimenArmToHook(){ specimenFlipServo.setPosition(0.2); }
    public void specimenArmToHang(){ specimenFlipServo.setPosition(.73); }
    public void specimenArmToHighBasket(){ specimenFlipServo.setPosition(0.45); }

    // auto uses flipped positions
    public void specimenArmToPickupAuto(){ specimenFlipServo.setPosition(.2); }
    public void specimenArmToHookAuto(){ specimenFlipServo.setPosition(.97); }

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
        HIGH(0.53),
        //MID(0.5),
        LOW(.09),
        INIT(0);

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

    public void reverseIntake(){
        leftRotationServo.setPower(1);
        rightRotationServo.setPower(1);
    }

    public void forwardIntake(){
        leftRotationServo.setPower(-1);
        rightRotationServo.setPower(-1);
    }

    public void stopIntake(){
        leftRotationServo.setPower(0);
        rightRotationServo.setPower(0);
    }

    public boolean horizontalArmFarBoundary() { return horizontalArmMotor.getCurrentPosition() > 2000; }

    public double getColorSensorProximity(DistanceUnit unit){
        return colorSensor.getDistance(unit);
    }

    public NormalizedRGBA getColorSensorColor(){
        return colorSensor.getNormalizedColors();
    }

    public void resetSpecimenSlide(){
        DcMotor.RunMode oldMode = specimenSlideMotor.getMode();
        specimenSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimenSlideMotor.setMode(oldMode);
    }
}