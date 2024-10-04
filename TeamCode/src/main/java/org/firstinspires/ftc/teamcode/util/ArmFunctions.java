package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Intake;


/**
 * This is the Arm Functions class. It contains all the arm functions.
 *
 * @author Lakshya Khandelwal - 26251 Instantbots
 * @version 1.0, 10/4/2024
 */
public class ArmFunctions {
    Arm arm;
    Intake intake;

    public void collectSample() {
        arm.rotateArm(0.5);
        intake.setIntakePower(1.0);
    }
}
