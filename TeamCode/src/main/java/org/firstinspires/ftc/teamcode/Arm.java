package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

public class Arm {

    public DcMotor rotationMotor;
    public DcMotorEx armMotor;

    public PIDFController pidf;
    public CustomPIDFCoefficients coefficients;

    public double output;


    // Constructor
    public Arm(HardwareMap hardwareMap, double p, double i, double d, double f) {
        // Initialize motor
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize rotation motor
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize PIDF coefficients
        coefficients = new CustomPIDFCoefficients(p, i, d, f);

        // Initialize PIDF controller with the coefficients
        pidf = new PIDFController(coefficients);

        // Set an initial target position for the arm
        pidf.setTargetPosition(0);  // Start at encoder position 0
    }

    // Method to set the arm position
    public void setThePower() {

        double encoderValue = getEncoderValue();

        // Update the PIDF controller with the current encoder value
        pidf.updatePosition(encoderValue);

        // Calculate the output using the PIDF controller
        output = pidf.runPIDF();

        // Get the current position from the encoder
        int currentPosition = armMotor.getCurrentPosition();

        // Update the PIDF controller with the current position
        pidf.updatePosition(currentPosition);

        // Calculate the PID output
        double power = pidf.runPIDF();

        // Set the motor's velocity to the calculated output
        armMotor.setVelocity(output);
    }

    // Method to rotate the arm
    public void rotateArm(double power) {
        power = 1;
        rotationMotor.setPower(power);
    }


    public void toPoint(double position) {
        pidf.setTargetPosition(position);
    }

    public double getSetPoint() {
        return pidf.getTargetPosition();
    }

    // Method to get the encoder value
    public double getEncoderValue() {
        return armMotor.getCurrentPosition();
    }

    // Method to stop the arm
    public void stopArmMotor() {
        armMotor.setVelocity(0);
    }

    public void stopRotationMotor() {
        rotationMotor.setPower(0);
    }
}