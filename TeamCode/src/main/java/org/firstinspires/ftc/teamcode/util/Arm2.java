package org.firstinspires.ftc.teamcode.util;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Arm2 {
    public DcMotorEx armMotor;
    public DcMotorEx pivotMotor;

    public PIDFController armPidf;
    public PIDFController pivotPidf;

    public CustomPIDFCoefficients armCoefficients;
    public CustomPIDFCoefficients pivotCoefficients;
    public static double armKp = 1, armKi = 0, armKd = 0, armKf = 1;

    private final double ticks_in_degree = (double) 168 / 360;

    public static double PivotKp = 1.0;
    public static double PivotKi = 0.0;
    public static double PivotKd = 0.02;
    public static double PivotKf = 1.0;
    public double output;

    final double ARM_TICKS_PER_DEGREE = 4.67;

    // Arm positions
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SAMPLE_IN_HIGH = 170 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;

    public static final int TICKS_PER_REVOLUTION = 28 * 60;  // 28 ticks * 60:1 gear ratio = 1680 ticks per revolution



    public Arm2(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize rotation motor
        pivotMotor = hardwareMap.get(DcMotorEx.class, "rotationMotor");
        pivotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armCoefficients = new CustomPIDFCoefficients(armKp, armKi, armKi, armKf);
        pivotCoefficients = new CustomPIDFCoefficients(PivotKp, PivotKi, PivotKd, PivotKf);
        armPidf = new PIDFController(armCoefficients);
        pivotPidf = new PIDFController(pivotCoefficients);
    }

    public void setPower() {
        double encoderValue = getEncoderValue();

        armPidf.updatePosition(encoderValue);

        output = armPidf.runPIDF();

        armMotor.setVelocity(output);
    }

    public void setPivotPower() {
        // Set power for pivot motor
        double pivotEncoderValue = getPivotEncoderValue();
        pivotPidf.updatePosition(pivotEncoderValue);
        output = pivotPidf.runPIDF();
        pivotMotor.setVelocity(output);
    }


    public void toPoint(double position) {
        armPidf.setTargetPosition(position);
    }

    public void toPivotPoint(double position) {
        pivotPidf.setTargetPosition(position);
    }

    public double getSetPoint() {
        return armPidf.getTargetPosition();
    }

    public double getPivotSetPoint() {
        return pivotPidf.getTargetPosition();
    }

    public double getEncoderValue() {
        return armMotor.getCurrentPosition();
    }

    public void stopExtending() {
        armMotor.setPower(0);
    }

    public void stopRotating() {
        pivotMotor.setPower(0);
    }
    // Method to get the encoder value for the rotation motor
    public double getPivotEncoderValue() {
        return pivotMotor.getCurrentPosition();
    }

}
