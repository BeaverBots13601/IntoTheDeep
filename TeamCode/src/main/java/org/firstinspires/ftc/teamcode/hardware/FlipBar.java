package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.teamcode.HardwareMechanism;

import java.util.function.BiConsumer;

public class FlipBar extends HardwareMechanism {
    private Servo specimenClawServo;
    private Servo specimenFlipServo;
    private boolean specimenClawDown = false;
    public FlipBar(HardwareMap hardwareMap, InitData data, BiConsumer<String, Object> telemetryFunc){
        super(hardwareMap, data, telemetryFunc);
        try {
            specimenClawServo = HardwareMechanism.setUpServo(hardwareMap, "specimenClawServo");
            specimenFlipServo = HardwareMechanism.setUpServo(hardwareMap, "specimenFlipServo");
        } catch (Exception e){
            available = false;
            return;
        }
        specimenArmToPickup();
        available = true;
    }

    public void start(BiConsumer<String, Object> telemetryFunc) {

    }

    public void run(RunData data, BiConsumer<String, Object> telemetryFunc) {
        // wall specimen grabber ctrl back (gp 2)
        if (data.currentGamepadTwo.square && !data.previousGamepadTwo.square) {
            if (specimenClawDown){
                closeSpecimenClaw();
                try {
                    Thread.sleep(100); // this is unorthodox, but it stops drivers leaving before closed
                } catch (InterruptedException e) { throw new RuntimeException(e); }
                specimenArmToHook();
                specimenClawDown = false;
            } else {
                specimenArmToPickup();
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) { throw new RuntimeException(e); }
                openSpecimenClaw();
                specimenClawDown = true;
            }
        }

        // wall specimen grabber ctrl front (gp 2)
        if (data.currentGamepadTwo.circle && !data.previousGamepadTwo.circle) {
            if (specimenClawDown){
                closeSpecimenClaw();
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) { throw new RuntimeException(e); }
                specimenArmToHookAuto();
                specimenClawDown = false;
            } else {
                specimenArmToPickupAuto();
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) { throw new RuntimeException(e); }
                openSpecimenClaw();
                specimenClawDown = true;
            }
        }
    }

    public void closeSpecimenClaw(){ specimenClawServo.setPosition(.52); }
    public void openSpecimenClaw(){ specimenClawServo.setPosition(.23); }

    public void specimenArmToPickup(){ specimenFlipServo.setPosition(1); }
    public void specimenArmToHook(){ specimenFlipServo.setPosition(0.21); }
    public void specimenArmToHang(){ specimenFlipServo.setPosition(.73); }

    // auto uses flipped positions
    public void specimenArmToPickupAuto(){ specimenFlipServo.setPosition(.21); }
    public void specimenArmToHookAuto(){ specimenFlipServo.setPosition(.97); }
}
