package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public CRServo intakeServo; // Servo to control in the intake(for blocks)
    // Servo pivotServo;  // Servo to control the pivot(for specimens)

    // Constructor to initialize intake motor and pivot servo
    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        // pivotServo = hardwareMap.get(Servo.class, "pivotServo");  // Initialize the pivot servo
    }

    // Method to control the intake servo
    public void setIntakePower(double power) {
        intakeServo.setPower(power);
    }

    // Method to get the current position of the intake servo
    public double getIntakePosition() {
        return intakeServo.getPower(); // This returns the current power set for the intake motor
    }

    // Method to set the pivot servo position
     public void setPivotPosition(double position) {
        // pivotServo.setPosition(position);
    }
}