package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

public class ClawState {
    public double clawPos;


    public ClawState() {
        this(0);
    }

    public ClawState(double clawPos) {
        this.clawPos = clawPos;

    }


    @NonNull
    @Override
    public String toString() {
        return "Claw State(Claw: " + clawPos;
    }
}
