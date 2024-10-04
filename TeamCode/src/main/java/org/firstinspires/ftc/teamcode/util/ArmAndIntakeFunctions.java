package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Intake;


/**
 * This is the Arm Functions class. It contains all the arm functions.
 *
 * @author Lakshya Khandelwal - 26251 Instantbots
 * @version 1.0, 10/4/2024
 */
public class ArmAndIntakeFunctions {
    Arm arm;
    Intake intake;

    public ArmAndIntakeFunctions(Arm arm, Intake intake) {
        this.arm = arm;
        this.intake = intake;
    }
    
    // Method to reset the arm's rotation position
    public void resetArmRotationPosition() {
        double currentArmPosition = arm.getRotatedArmPosition(); // Get current arm position in degrees
        arm.rotateArm(0); // Stop arm rotation
        arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset arm motor encoder
        arm.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Resume normal arm operation
    }

    public void collectSample() {
        arm.rotateArm(0.5);
        intake.setIntakePower(1.0);
    }
    public void scoreHighBasket() {
        arm.toPoint(-5000);
        arm.rotateArm(-0.5);
        intake.setIntakePower(-1.0);
    }
    public void scoreLowBasket() {
        arm.toPoint(-1000);
        arm.rotateArm(-0.5);
        intake.setIntakePower(-1);
    }
    public void scoreSpecimen() {
        arm.rotateArm(-0.5);
        intake.setPivotPosition(0.5);
    }
    public void levelTwoAscent(){
        arm.rotateArm(-0.5);
        arm.toPoint(-1250);
        arm.toPoint(0);
    }
}
