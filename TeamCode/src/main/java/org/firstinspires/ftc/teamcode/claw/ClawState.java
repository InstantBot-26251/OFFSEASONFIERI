package org.firstinspires.ftc.teamcode.claw;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.arm.Pivot;

public class ClawState {
    public double clawPos;
    public double wristPos;

    public ClawState() {
        this(0, 0);
    }

    public ClawState(double clawPos, double wristPos) {
        this.clawPos = clawPos;
        this.wristPos = wristPos;

    }

    @NonNull
    @Override
    public String toString() {
        return "Claw State(Claw: " + clawPos + ", Wrist: " + wristPos;
    }
}