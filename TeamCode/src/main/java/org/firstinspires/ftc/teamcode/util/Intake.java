package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public final CRServo intakeServo; // Servo to control in the intake (for samples)
    // Servo pivotServo;  // Servo to control the pivot (for specimens)

    // Intake power settings
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    // Constructor to initialize intake motor and pivot servo
    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        // pivotServo = hardwareMap.get(Servo.class, "pivotServo");  // Initialize the pivot servo

        setIntakePower(INTAKE_OFF); // Ensure the intake is off initially
    }

    // Method to control the intake servo
    public void setIntakePower(double power) {
        intakeServo.setPower(power);
    }

    public void collect() {
        setIntakePower(INTAKE_COLLECT);
    }


    public void deposit() {
        setIntakePower(INTAKE_DEPOSIT);
    }

    public void stop() {
        setIntakePower(INTAKE_OFF);
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