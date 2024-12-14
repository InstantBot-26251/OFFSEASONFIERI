package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.*;

public class Intake {
    public final CRServo claw; // Servo to control in the intake (for samples)
  //  public final CRServo wrist;
    // Servo pivotServo;  // Servo to control the pivot (for specimens)
    public final ClawState state;
    // Constructor to initialize intake motor and pivot servo
    public Intake(HardwareMap hardwareMap, ClawState state) {
      //-+  wrist = hardwareMap.get(CRServo.class, "wristServo");
        claw = hardwareMap.get(CRServo.class, "intakeServo");
        this.state = state;
        // pivotServo = hardwareMap.get(Servo.class, "pivotServo");  // Initialize the pivot servo
    }

    // Method to control the intake servo
    public void setIntakePower(double power) {
        claw.setPower(power);
    }

    public void openClaw() {
        state.clawPos = 0;
    }

    public void closeClaw() {
        state.clawPos = 1;
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