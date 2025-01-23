package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.Globals.ascentMode;
import static org.firstinspires.ftc.teamcode.Globals.manualVerticalMode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;
import org.firstinspires.ftc.teamcode.LimiterState;
import org.firstinspires.ftc.teamcode.rr.InterruptableAction;

import java.util.function.BiConsumer;

// note about this class: there's a strong argument to be made that rear and specimen slides
// should be separated. however, that introduces issues with global state management and who
// decides what that state is. all because of one little switch... not dealing with that right now
public class VerticalSlides extends HardwareMechanism {
    // magic numbers
    public static final int CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS = 4000;
    public static final int CALIBRATED_SPECIMEN_SLIDE_HEIGHT_TICKS = 2200;

    // hardware
    private DcMotorEx leftRearVerticalArmMotor;
    private DcMotorEx rightRearVerticalArmMotor;
    private DcMotorEx specimenSlideMotor;
    
    public VerticalSlides(HardwareMap hardwareMap, InitData data, BiConsumer<String, Object> telemetryFunc){
        super(hardwareMap, data, telemetryFunc);
        try {
            leftRearVerticalArmMotor = HardwareMechanism.createDefaultMotor(hardwareMap, "leftRearVerticalArmMotor");
            rightRearVerticalArmMotor = HardwareMechanism.createDefaultMotor(hardwareMap, "rightRearVerticalArmMotor");
            specimenSlideMotor = HardwareMechanism.createDefaultMotor(hardwareMap, "specimenSlideMotor");
        } catch (Exception e) {
            available = false;
            return;
        }
        
        leftRearVerticalArmMotor.setDirection(DcMotorSimple.Direction.FORWARD); // hardware thing
        rightRearVerticalArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        specimenSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        available = true;
    }
    
    public void start() {
        
    }

    private InterruptableAction runningSpecimenSlideAction = null;
    // class instead of primitive to allow nullability
    private Integer holdingSpecimenSlidePos = null;
    public void run(RunData data) {
        // toggle manual verticals (gp2)
        if (data.currentGamepadTwo.dpad_left && !data.previousGamepadTwo.dpad_left){
            manualVerticalMode = !manualVerticalMode;
            if (manualVerticalMode){
                // reset powers to 0 to avoid state where slides are stuck at power lvl
                setRearVerticalArmPower(0);
                setSpecimenSlidePower(0);
                if(runningSpecimenSlideAction != null) {
                    runningSpecimenSlideAction.interrupt();
                }
                runningSpecimenSlideAction = null;
                holdingSpecimenSlidePos = null;
            }
        }

        // down (gp 2)
        if (!manualVerticalMode && !ascentMode && data.currentGamepadTwo.dpad_down && !data.previousGamepadTwo.dpad_down){
            if(runningSpecimenSlideAction != null) runningSpecimenSlideAction.interrupt();
            runningSpecimenSlideAction = roadrunnerRaiseSpecimenSlideToHeight(0);
            // Don't require automatic/slide mode to allow post-ascent mode lowering
        }

        // clipped (gp 2)
        if (!manualVerticalMode && !ascentMode && data.currentGamepadTwo.dpad_up && !data.previousGamepadTwo.dpad_up){
            if(runningSpecimenSlideAction != null) runningSpecimenSlideAction.interrupt();
            runningSpecimenSlideAction = roadrunnerRaiseSpecimenSlideToHeight(0.75);
            // Don't require automatic/slide mode to allow post-ascent mode lowering
        }

        // pre-clip (gp 2)
        if (!manualVerticalMode && !ascentMode && data.currentGamepadTwo.dpad_right && !data.previousGamepadTwo.dpad_right){
            if(runningSpecimenSlideAction != null) runningSpecimenSlideAction.interrupt();
            runningSpecimenSlideAction = roadrunnerRaiseSpecimenSlideToHeight(0.5);
            // Don't require automatic/slide mode to allow post-ascent mode lowering
        }

        // vertical arm (gp 2)
        float a = data.currentGamepadTwo.right_trigger - data.currentGamepadTwo.left_trigger;
        telemetry.accept("Vertical Arm Power", a);

        // specimen slide ctrl (gp2)
        if (runningSpecimenSlideAction == null){
            // either in manual or need to hold our height
            if (manualVerticalMode){
                // allow manual controls
                if (!ascentMode){
                    //LimiterState lim = typedRobot.getSpecimenSlideLimiterState();
                    LimiterState lim = LimiterState.NONE;
                    telemetry.accept("Limiter State", lim);
                    //if(a == 0) a = 0.005f;
                    setSpecimenSlideMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if(lim == LimiterState.NEAR) setSpecimenSlidePower(Math.max(0, a));
                    if(lim == LimiterState.FAR) setSpecimenSlidePower(Math.min(0, a));
                    if(lim == LimiterState.NONE) setSpecimenSlidePower(a);
                }
            } else {
                if (holdingSpecimenSlidePos != null){
                    // hold height
                    setSpecimenSlideTargetPos(holdingSpecimenSlidePos);
                    setSpecimenSlidePower(1);
                    setSpecimenSlideMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else {
                    setSpecimenSlidePower(0);
                }
            }
        } else {
            // run action. clear if done
            if(!runningSpecimenSlideAction.run(new TelemetryPacket())) runningSpecimenSlideAction = null;
            // save our height to keep at
            holdingSpecimenSlidePos = getSpecimenSlidePos();
        }

        // rear slide ctrl (gp2)
        // These limiters are prone to drift maybe? Depends how precise motors are. Investigate
        // If drivers report issues, look into making a reset button.
        if (manualVerticalMode && ascentMode) {
            //LimiterState lim = typedRobot.getRearVerticalSlideLimiterState();
            LimiterState lim = LimiterState.NONE;
            telemetry.accept("Limiter State", lim);
            // at lower limit: only allow positive speeds
            if(lim == LimiterState.NEAR) setRearVerticalArmPower(Math.max(0, a));
            // at upper limit: only allow negative speeds
            if(lim == LimiterState.FAR) setRearVerticalArmPower(Math.min(0, a));
            if(lim == LimiterState.NONE) setRearVerticalArmPower(a);
        }

        telemetry.accept("Slides In Manual Mode", manualVerticalMode);
    }

    public InterruptableAction roadrunnerMoveRearVerticalSlidesToHeight(double height){
        return new InterruptableAction() {
            private boolean initialized = false;
            private DcMotor.RunMode before;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    before = leftRearVerticalArmMotor.getMode();

                    rightRearVerticalArmMotor.setTargetPosition((int) (CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS * height));
                    leftRearVerticalArmMotor.setTargetPosition((int) (CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS * height));

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

    public LimiterState getSpecimenSlideLimiterState(){
        if(specimenSlideMotor.getCurrentPosition() < 10) return LimiterState.NEAR; // negative: reversed
        if(specimenSlideMotor.getCurrentPosition() > CALIBRATED_SPECIMEN_SLIDE_HEIGHT_TICKS - 10) return LimiterState.FAR;
        return LimiterState.NONE;
    }

    public LimiterState getRearVerticalSlideLimiterState(){
        if(rightRearVerticalArmMotor.getCurrentPosition() < 10) return LimiterState.NEAR; // negative: reversed
        if(rightRearVerticalArmMotor.getCurrentPosition() > CALIBRATED_REAR_VERTICALS_HEIGHT_TICKS - 10) return LimiterState.FAR;
        return LimiterState.NONE;
    }

    public void setRearVerticalArmPower(double speed){
        double limitedSpeed = Math.min(Math.max(speed, -0.70), 0.70); // primary motors
        leftRearVerticalArmMotor.setPower(limitedSpeed);
        rightRearVerticalArmMotor.setPower(limitedSpeed);
    }

    public void setSpecimenSlideMode(DcMotor.RunMode mode){
        specimenSlideMotor.setMode(mode);
    }

    public void setSpecimenSlideTargetPos(int target){ specimenSlideMotor.setTargetPosition(target); }

    public int getSpecimenSlidePos(){
        return specimenSlideMotor.getCurrentPosition();
    }

    /**
     * @return The current height of the slide, [0, 1] as a percentage of its maximum.
     */
    public double getSpecimenSlideHeight(){
        return (double) specimenSlideMotor.getCurrentPosition() / CALIBRATED_SPECIMEN_SLIDE_HEIGHT_TICKS;
    }

    public InterruptableAction roadrunnerRaiseSpecimenSlideToHeight(double height){
        return new InterruptableAction() {
            private boolean initialized = false;
            private DcMotor.RunMode before;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized){
                    before = specimenSlideMotor.getMode();

                    specimenSlideMotor.setTargetPosition((int) (CALIBRATED_SPECIMEN_SLIDE_HEIGHT_TICKS * height));

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

    public void setSpecimenSlidePower(double speed){
        specimenSlideMotor.setPower(speed);
    }

    @Deprecated
    public Action roadrunnerRaiseSpecimenSlideToHeightBugged(double height){
        return new Action() {
            private boolean initialized = false;
            private DcMotor.RunMode before;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized){
                    before = specimenSlideMotor.getMode();

                    specimenSlideMotor.setTargetPosition((int) (CALIBRATED_SPECIMEN_SLIDE_HEIGHT_TICKS * height));

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
}
