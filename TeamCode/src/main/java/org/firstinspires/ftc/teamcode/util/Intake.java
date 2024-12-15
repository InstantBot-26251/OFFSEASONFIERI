package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.*;

public class Intake {
    public final Servo claw; // Servo to control in the intake (for samples)
    public final CRServo wrist;
    public final ClawState state;

    public static double OPEN_CLAW = 0;
    public static double CLOSE_CLAW = 1;
    // Constructor to initialize intake motor and pivot servo
    public Intake(HardwareMap hardwareMap, ClawState state) {
      wrist = hardwareMap.get(CRServo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        this.state = state;
    }


    public void setWristPower(double power) {
        double wristPos = state.wristPos = power;
        wrist.setPower(wristPos);
    }
    public void openClaw() {
        state.clawPos = 0;
        claw.setPosition(OPEN_CLAW);
    }

    public void closeClaw() {
        state.clawPos = 1;
        claw.setPosition(CLOSE_CLAW);
    }

    // Method to get the current position of the intake servo
    public double getIntakePosition() {
        return claw.getPosition(); // This returns the current power set for the intake motor
    }


    // Method to set the pivot servo position
     public void setPivotPosition(double position) {
        // pivotServo.setPosition(position);
    }
}