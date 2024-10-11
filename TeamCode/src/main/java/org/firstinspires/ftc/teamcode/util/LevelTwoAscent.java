package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Intake;

public class LevelTwoAscent {
    Arm arm;
    ArmAndIntakeFunctions functions;
    Intake intake;

    public LevelTwoAscent(Arm arm,  Intake intake, ArmAndIntakeFunctions functions) {
        this.arm = arm;
        this.intake = intake;
        this.functions = functions;
    }

    public void drive() {
        functions.levelTwoAscent();
    }
}
