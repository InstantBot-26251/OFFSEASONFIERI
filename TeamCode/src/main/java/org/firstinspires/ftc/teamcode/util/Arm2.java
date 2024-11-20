package org.firstinspires.ftc.teamcode.util;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile.State;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.arm.*;

@Config
public class Arm2 extends ArmConstants {
    public DcMotorEx armMotor;
    public static DcMotorEx pivotMotor;

    public PIDFController armPid;
    public static PIDFController pivotPid;

    public static double maxVelocity = 500;  // ticks per second
    public static double maxAcceleration = 100;  // ticks per second^2
    public static double pivotMaxVelocity = 400;  // ticks per second for pivot
    public static double pivotMaxAcceleration = 80;  // ticks per second^2 for pivot

    public static double armKp = 1;
    public static double armKi = 0;
    public static double armKd = 0.2;
    public static double armKf = 1;
    public static double pivotKp = 1.15;
    public static double pivotKi = 0;
    public static double pivotKd = 0.25;
    public static double pivotKf = 1.1;

    final double ARM_TICKS_PER_DEGREE = 4.67;

    // Arm positions
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SAMPLE_IN_HIGH = 170 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;

    public static final int TICKS_PER_REVOLUTION = 28 * 60;  // 28 ticks * 60:1 gear ratio = 1680 ticks per revolution

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile pivotMotionProfile;
    private State pivotGoalState;
    private State pivotCurrentState;
    private State goalState;
    private State currentState;

    public Arm2(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize rotation motor
        pivotMotor = hardwareMap.get(DcMotorEx.class, "rotationMotor");
        pivotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPid = new PIDFController(armKp, armKi, armKd, armKf);
        pivotPid = new PIDFController(pivotKp, pivotKi, pivotKd, pivotKf);

        goalState = new State(0, 0);
        currentState = new State(0, 0);

        pivotGoalState = new State(0, 0);
        pivotCurrentState = new State(0, 0);
    }

    public void setPower(double input) {
//        double output = armPid.calculate(getEncoderValue());
        armMotor.setPower(input);
    }

    public void setPivotPower(double input) {
        double output = pivotPid.calculate(getEncoderValue());
        pivotMotor.setPower(input);
    }


    public void toPoint(double position) {
        // Set the goal state for motion profiling
        goalState = new State(position, 0); // Position target with 0 velocity
        motionProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
                goalState,
                currentState
        );
    }

    public void update() {
        // Update the motion profile based on elapsed time
        double dt = 0.02; // 20ms loop time
        currentState = motionProfile.calculate(dt);

        // Compute feedforward + feedback control
        double feedforward = armKf * currentState.velocity;
        double feedback = armPid.calculate(getEncoderValue(), currentState.position);

        double power = feedforward + feedback;
        armMotor.setPower(power);

        // Update pivot motion profile
        pivotCurrentState = pivotMotionProfile.calculate(dt);
        double pivotFeedforward = pivotKf * pivotCurrentState.velocity;
        double pivotFeedback = pivotPid.calculate(getPivotEncoderValue(), pivotCurrentState.position);
        double pivotPower = pivotFeedforward + pivotFeedback;
        pivotMotor.setPower(pivotPower);
    }

    public void toPivotPoint(double position) {
        // Set the goal state for pivot motion profiling
        pivotGoalState = new State(position, 0); // Position target with 0 velocity
        pivotMotionProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(pivotMaxVelocity, pivotMaxAcceleration),
                pivotGoalState,
                pivotCurrentState
        );
    }

    public double getSetPoint() {
        return armPid.getSetPoint();
    }

    public double getPivotSetPoint() {
        return pivotPid.getSetPoint();
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
