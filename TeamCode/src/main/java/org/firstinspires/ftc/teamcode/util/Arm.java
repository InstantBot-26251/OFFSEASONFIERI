package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

@Config
public class Arm {

    public DcMotorEx rotationMotor;
    public DcMotorEx armMotor;

    public PIDFController armPidf;
    public PIDFController rotationPidf;
    public CustomPIDFCoefficients armCoefficients;
    public CustomPIDFCoefficients rotationCoefficients;
    public double armTolerance = 90; // 90-degree limit for scoring
    public double output;

    public static double kPArm = 1; // constants
    public static double kIArm = 0;
    public static double kDArm = 0;
    public static double kFArm = 1;
    public static double kPPivot = 1;
    public static double kIPivot = 0;
    public static double kDPivot = 0;
    public static double kFPivot = 1;

    final double ARM_TICKS_PER_DEGREE = 7.7778;

    // Arm positions
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_HIGH = 170 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;

    private double armPosition = ARM_COLLAPSED_INTO_ROBOT;

    // Encoder ticks per revolution
    public static final int TICKS_PER_REVOLUTION = 28 * 20;  // 28 ticks * 20:1 gear ratio = 560 ticks per revolution

    private static final double MAX_POWER = 1.0; // Maximum power to apply

    // Encoder counts corresponding to the rotation angles (60 and 75 degrees)
    public static final int ROTATE_60 = 600; // Example value, adjust based on testing
    public static final int ROTATE_75 = 750; // Example value, adjust based on testing

    // Constructor
    public Arm(HardwareMap hardwareMap) {
        // Initialize motor
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize rotation motor
        rotationMotor = hardwareMap.get(DcMotorEx.class, "rotationMotor");
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the ZeroPowerBehavior for the rotation motor
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Use BRAKE behavior

        // Initialize PIDF coefficients
        armCoefficients = new CustomPIDFCoefficients(kPArm, kIArm, kDArm, kFArm);
        rotationCoefficients = new CustomPIDFCoefficients(kPPivot, kIPivot, kDPivot, kFPivot);


        // Initialize PIDF controller with the coefficients
        armPidf = new PIDFController(armCoefficients);
        rotationPidf = new PIDFController(rotationCoefficients);


        // Set an initial target position for the lift & arm(rotated)
        armPidf.setTargetPosition(0);  // Start at encoder position 0
        rotationPidf.setTargetPosition(0); // Start at encoder position 0 for rotation

        resetArm();
    }

    public void resetArm() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveArmToPosition(double position) {
        armPosition = position;
        rotationMotor.setTargetPosition((int) position);
        rotationMotor.setPower(1.0); // Set power to move to position
    }

    // Method to set the arm power using PIDF control
    public void setArmPower() {
        double encoderValue = getArmEncoderValue();

        // Update the PIDF controller with the current encoder value
        armPidf.updatePosition(encoderValue);

        // Calculate the output using the PIDF controller
        output = armPidf.runPIDF();

        // Set the motor's velocity to the calculated output
        armMotor.setVelocity(output);
    }

    private double calculateRotationPower(double targetPosition) {
        double currentPosition = getRotatedArmPosition();
        double error = targetPosition - currentPosition;

        // Simple proportional control for power calculation
        double power = error * 0.01; // Adjust gain as needed
        return Math.max(-MAX_POWER, Math.min(MAX_POWER, power)); // Limit power to range [-MAX_POWER, MAX_POWER]
    }

    // Method to rotate arm to a target angle with calculated power
    public void rotateArm(double targetDegrees) {
        rotationPidf.setTargetPosition(targetDegrees);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Calculate the power to apply
        double rotationOutput = calculateRotationPower(targetDegrees);

        // Limit rotation to a maximum of 90 degrees
        if (getRotatedArmPosition() >= armTolerance) {
            rotationOutput = 0; // Stop if at limit
        }   

        // Set the rotation motor's power to the calculated output
        rotationMotor.setPower(rotationOutput);
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

    public void setPower() {
        armPidf.updatePosition(getArmEncoderValue());  // Update the current position (encoder value)
        output = armPidf.runPIDF();       // Calculate the output using the PIDF
        armMotor.setVelocity(output);            // Set the motor velocity based on the PIDF output
    }

    public void toPoint(double position) {
        armPidf.setTargetPosition(position);
    }

    public double getSetPoint() {
        return armMotor.getTargetPosition();
    }

    // Method to get the encoder value for the arm motor
    public double getArmEncoderValue() {
        return armMotor.getCurrentPosition();
    }
    // Method to get the encoder value for the rotation motor
    public double getRotationEncoderValue() {
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
