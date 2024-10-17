package org.firstinspires.ftc.teamcode.util;


public class CollectSample {
    public Arm arm;
    public Intake intake;
    public ArmAndIntakeFunctions functions;

    public CollectSample(Arm arm, Intake intake, ArmAndIntakeFunctions functions) {
        this.arm = arm;
        this.intake = intake;
        this.functions = functions;
    }

    public void drive()  {
        functions.collectSample();
    }
}
