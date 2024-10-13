package org.firstinspires.ftc.teamcode.util;

public class ScoreHighBasket {
    Arm arm;
    ArmAndIntakeFunctions functions;
    Intake intake;

    public ScoreHighBasket(Arm arm,  Intake intake, ArmAndIntakeFunctions functions) {
        this.arm = arm;
        this.intake = intake;
        this.functions = functions;
    }

    public void drive() {
        functions.scoreHighBasket();
    }
}
