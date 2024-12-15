package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.*;

public class Intake {
    public final CRServo claw; // Servo to control in the intake (for samples)
  public final CRServo wrist;
    public final ClawState state;
    // Constructor to initialize intake motor and pivot servo
    public Intake(HardwareMap hardwareMap, ClawState state) {
      wrist = hardwareMap.get(CRServo.class, "wristServo");
        claw = hardwareMap.get(CRServo.class, "intakeServo");
        this.state = state;
    }


    public void setWristPower(double power) {
        double wristPos = state.wristPos = power;
        wrist.setPower(wristPos);
    }
    public void openClaw() {
        double clawPos = state.clawPos = 0;
        claw.setPower(clawPos);
    }

    public void closeClaw() {
        double clawPos = state.clawPos = 1;
        claw.setPower(clawPos);
    }

    // Method to get the current position of the intake servo
    public double getIntakePosition() {
        return claw.getPower(); // This returns the current power set for the intake motor
    }


    // Method to set the pivot servo position
     public void setPivotPosition(double position) {
        // pivotServo.setPosition(position);
    }
}