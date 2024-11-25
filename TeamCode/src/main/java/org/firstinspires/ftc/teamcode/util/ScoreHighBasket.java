package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ScoreHighBasket {
    private final Arm2 arm;
    private final Intake intake;
    private final Gamepad gamepad;
    private final ArmAndIntakeFunctions functions;
    private boolean isScoringHighBasket = false;

    // Constructor to initialize arm, intake, and gamepad
    public ScoreHighBasket(Arm2 arm, Intake intake, Gamepad gamepad, ArmAndIntakeFunctions functions) {
        this.arm = arm;
        this.intake = intake;
        this.gamepad = gamepad;
        this.functions = functions;
    }

    // Method to score in the high basket
    public void execute() {
        // functions.depositGameElement();
    }

    // Method to check if scoring in the high basket is finished
    public boolean isFinished() {
        // Check if the arm is at the target position and intake is stopped
        boolean armAtPosition = arm.getPivotEncoderValue() == Arm.ROTATE_60; // Check if at lowered position
        boolean intakeStopped = intake.getIntakePosition() == 0.0; // Ensure intake is stopped

        if (armAtPosition && intakeStopped) {
            isScoringHighBasket = false; // Reset scoring state
        }

        return armAtPosition && intakeStopped; // Return if scoring is finished
    }

    // Method to check if scoring is currently in progress
    public boolean isScoringInProgress() {
        return isScoringHighBasket;
    }
}