package org.firstinspires.ftc.teamcode.rr;

import com.acmerobotics.roadrunner.Action;

public interface InterruptableAction extends Action {
    void interrupt();
}
