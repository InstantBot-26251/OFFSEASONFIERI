package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

public class Arm {

    public DcMotorEx rotationMotor;
    public DcMotorEx armMotor;

    public PIDFController pidf;
    public CustomPIDFCoefficients coefficients;

    public double armTolerance = 90; // 90-degree limit for scoring
    public double output;

    // Encoder ticks per revolution
    public static final int TICKS_PER_REVOLUTION = 28 * 20;  // 28 ticks * 20:1 gear ratio = 560 ticks per revolution


    // Constructor
    public Arm(HardwareMap hardwareMap, double p, double i, double d, double f) {
        // Initialize motor
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize rotation motor
        rotationMotor = hardwareMap.get(DcMotorEx.class, "rotationMotor");
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize PIDF coefficients
        coefficients = new CustomPIDFCoefficients(p, i, d, f);

        // Initialize PIDF controller with the coefficients
        pidf = new PIDFController(coefficients);

        // Set an initial target position for the arm
        pidf.setTargetPosition(0);  // Start at encoder position 0
    }

    // Method to set the arm power using PIDF control
    public void setThePower() {
        double encoderValue = getEncoderValue();

        // Update the PIDF controller with the current encoder value
        pidf.updatePosition(encoderValue);

        // Calculate the output using the PIDF controller
        output = pidf.runPIDF();

        // Set the motor's velocity to the calculated output
        armMotor.setVelocity(output);
    }

    // Method to rotate the arm within the allowed range (e.g., for scoring)
    public void rotateArm(double power) {
        double currentDegrees = getRotatedArmPosition();

        // Limit rotation to a maximum of 90 degrees
        if (currentDegrees >= armTolerance) {
            // Stop the rotation if it reaches 90 degrees
            power = 0;
        }

        // Set motor power (only if under the limit)
        rotationMotor.setPower(power);
    }

    // Method to set arm tolerance for scoring (ensures arm doesn't exceed 90 degrees)
    public void setArmTolerance() {
        double rotatedPosition = getRotatedArmPosition();
        if (rotatedPosition > armTolerance) {
            // If the arm exceeds 90 degrees, stop it
            rotateArm(0);  // Stop rotation
        }
    }

    // Method to get the rotated arm position in degrees
    public double getRotatedArmPosition() {
        // Get the current encoder position from the rotation motor
        int encoderPosition = rotationMotor.getCurrentPosition();

        // Convert encoder ticks to degrees (assuming 360 degrees corresponds to TICKS_PER_REVOLUTION)
        return (encoderPosition / (double) TICKS_PER_REVOLUTION) * 360.0;
    }

    public void toPoint(double position) {
        pidf.setTargetPosition(position);
    }

    public double getSetPoint() {
        return pidf.getTargetPosition();
    }

    // Method to get the encoder value for the arm motor
    public double getEncoderValue() {
        return armMotor.getCurrentPosition();
    }

    // Method to stop the arm motor
    public void stopArmMotor() {
        armMotor.setVelocity(0);
    }

    // Method to stop the rotation motor
    public void stopRotationMotor() {
        rotationMotor.setPower(0);
    }
}
