package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.vision.AprilTagData;

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
    private final DcMotorEx leftUpperAscentMotor;
    private final DcMotorEx rightUpperAscentMotor;

    // candidate to be moved to base robot
    private final Limelight3A limelight;

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
        leftUpperAscentMotor = createDefaultMotor("leftUpperAscentMotor");
        leftUpperAscentMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightUpperAscentMotor = createDefaultMotor("rightUpperAscentMotor");
        rightUpperAscentMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        limelight = opmode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
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
        ArrayList<Integer> lastFiveChanges = new ArrayList<>(5);
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

        leftVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVerticalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_VERTICALS_HEIGHT_TICKS * height));
        leftVerticalArmMotor.setTargetPosition((int) (constants.CALIBRATED_VERTICALS_HEIGHT_TICKS * height));

        while (rightVerticalArmMotor.isBusy());

        leftVerticalArmMotor.setMode(before);
        rightVerticalArmMotor.setMode(before);
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
        specimenClawServo.setPosition(0);
    }

    /**
     * Once positioned, latches the hooks onto L3 the bar. Takes 2.5s.
     */
    public void latchLowerAscentHooks(){
        leftAscentServo.setPower(0.75);
        rightAscentServo.setPower(0.75);
        opMode.sleep(2500);
        leftAscentServo.setPower(0);
        rightAscentServo.setPower(0);
    }

    public void reelLowerAscentHooks(){
        leftAscentServo.setPower(-0.75);
        rightAscentServo.setPower(-0.75);
        opMode.sleep(3000);
        leftAscentServo.setPower(0);
        rightAscentServo.setPower(0);
    }

    public void upperAscentMotorsToLatched(){
        leftUpperAscentMotor.setTargetPosition(-285);
        rightUpperAscentMotor.setTargetPosition(285);
        while (leftUpperAscentMotor.isBusy()){
            opMode.sleep(5);
        }
    }

    public void pullUpperAscentMotors(){
        leftUpperAscentMotor.setTargetPosition(0);
        rightUpperAscentMotor.setTargetPosition(0);
    }

    public ArrayList<AprilTagData> getLastLimelightAprilTags(){
        ArrayList<AprilTagData> out = new ArrayList<>();

        limelight.getLatestResult().getFiducialResults().forEach((LLResultTypes.FiducialResult a) -> out.add(new AprilTagData(a.getFiducialId(), a.getTargetPoseRobotSpace().getPosition().z, 0)));

        return out;
    }

    public void updateLimelightIMUData(){
        limelight.updateRobotOrientation(getImuAngle());
    }

    public Position getLimelightPositionalData() {
        return limelight.getLatestResult().getBotpose_MT2().getPosition();
    }

    public void setVerticalArmPower(float speed){
        leftVerticalArmMotor.setPower(speed);
        rightVerticalArmMotor.setPower(speed);
    }
}
