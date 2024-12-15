package org.firstinspires.ftc.teamcode.util;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile.State;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmConstants;

@Config
public class Arm2 {
    public ClawState clawState;
    static Telemetry telemetry;
    public DcMotorEx armMotor;
    public DcMotorEx pivotMotor;
    private ElapsedTime elapsedTime;
    public Intake intake;
    public PIDFController armPid;
    public PIDFController pivotPid;

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
    public static final double MAX_ENCODER_EXTENSION = 1000; // Example max encoder value at full extension
    public static final double MIN_ENCODER_EXTENSION = -4000; // Minimum slide retraction
    public static final double SLIDE_AT_42_INCHES = -2191; // Encoder value for 42 inches when pivot is fully down
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

    private static final int PIVOT_DOWN_ENCODER = 259;  // Encoder value when pivot is down
    private static final int PIVOT_UP_ENCODER = -1053; // Encoder value when pivot is up
    private static final int MAX_SLIDE_ENCODER = 42 * 50; // 42 inches * encoder ticks per inch (adjust accordingly)

    public Arm2(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Initialize Intake
        intake = new Intake(hardwareMap, clawState);

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

    // Set the power for the slide motor directly (for manual control in TeleOp)
    public void setSlidePower(double power) {
        armMotor.setPower(power);
    }
    // Set the power for the pivot motor directly (for manual control in TeleOp)
    public void setPivotPower(double power) {
        pivotMotor.setPower(power);  // Set the power for the pivot motor (direct control)
    }

    // Optionally, update the slide motor power using PID control (if desired)
    public void updateSlide() {
        double currentPosition = armMotor.getCurrentPosition();
        double pidOutput = armPid.calculate(currentPosition);
        double motorPower = Math.max(-1.0, Math.min(1.0, pidOutput)); // Limit the power to [-1, 1]
        armMotor.setPower(motorPower);
    }

    // Optionally, update the pivot motor power using PID control (if desired)
    public void updatePivot() {
        double currentPosition = pivotMotor.getCurrentPosition();
        double pidOutput = pivotPid.calculate(currentPosition);
        double motorPower = Math.max(-1.0, Math.min(1.1, pidOutput)); // Limit the power to [-1, 1]
        pivotMotor.setPower(motorPower);
    }

    // Optionally, set new PID coefficients for fine-tuning the controllers
    public void setPIDCoefficients(double kP, double kI, double kD, double kF) {
        armPid.setPIDF(kP, kI, kD, kF);
        pivotPid.setPIDF(kP, kI, kD, kF);
    }

    // Optionally, get the current position of the slide motor
    public double getSlidePosition() {
        return armMotor.getCurrentPosition();
    }

    // Optionally, get the current position of the pivot motor
    public double getPivotPosition() {
        return pivotMotor.getCurrentPosition();
    }
}
