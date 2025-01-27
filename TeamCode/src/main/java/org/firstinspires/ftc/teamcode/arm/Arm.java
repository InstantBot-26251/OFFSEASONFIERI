package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;

import android.icu.text.CaseMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;
import org.firstinspires.ftc.teamcode.arm.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.util.hardware.InstantMotor;
import org.firstinspires.ftc.teamcode.util.subsystem.SubsystemIF;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Fieri;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

public class Arm extends SubsystemIF {
    Telemetry telemetry;

    Gamepad ishu;
    PIDController slidePid;
    PIDController pivotPid;


    InstantMotor slide;
    Pivot pivot;

    private static final Arm INSTANCE = new Arm();

    public boolean slideZeroing = false;

    ArmState state;
    ScoreType scoreType;

    public enum ArmState {
        STOW, COLLECTING_SAMPLE, COLLECTING_SPECIMEN, SCORING_SAMPLE, SCORING_SPECIMEN
    }

    public enum ScoreType {
        SPECIMEN, SAMPLE
    }

    public static Arm getInstance() {
        return INSTANCE;
    }

    public Arm() {
        slidePid = new PIDController(ArmConstants.SLIDE_KP, ArmConstants.SLIDE_KI, ArmConstants.SLIDE_KD);
        pivotPid = new PIDController(ArmConstants.PIVOT_KP, ArmConstants.PIVOT_KI, ArmConstants.PIVOT_KD);

        state = ArmState.STOW;
        scoreType = ScoreType.SAMPLE;
    }

    @Override
    public void onAutonomousInit() {
        telemetry = Fieri.getInstance().getTelemetry();
        setupMotors();
        configureHardware();
        resetSlideEncoder();

        pivotPid.setSetPoint(pivot.getPosition());
        slidePid.setSetPoint(getSlidePosition());
    }

    public double getSlidePosition() {
        return slide.getCurrentPosition();
    }

    @Override
    public void onTeleopInit() {
        telemetry = Fieri.getInstance().getTelemetry();
        configureHardware();
        setupMotors();

        pivotPid.setSetPoint(pivot.getPosition());
        slidePid.setSetPoint(getSlidePositon());

        Commands.sequence(
                Commands.waitUntil(RobotStatus::isEnabled),
                new MoveSlideCommand(() -> SLIDE_CHAMBER_POSITION),
                Commands.runOnce(() -> setState(ArmState.SCORING_SAMPLE))
                //wait until enabled, then zero
        ).schedule();
    }

    public double getSlidePositon() {
        return slide.getCurrentPosition();
    }

    public void setupMotors() {
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void configureHardware() {
        slide = new InstantMotor(RobotMap.getInstance().SLIDE);
        pivot = new Pivot();

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public double getSlideTarget() {
        return slidePid.getSetPoint();
    }

    public double getPivotTarget() {
        return pivotPid.getSetPoint();
    }

    public boolean slideAtSetPoint() {
        return Math.abs(getSlidePosition() - getSlideTarget()) < SLIDE_ERROR_TOLERANCE;
    }

    public boolean pivotAtSetPoint() {
        return Math.abs(pivot.getPosition() - getPivotTarget()) < PIVOT_ERROR_TOLERANCE;
    }

    public double getSlideVelocity() {
        return slide.getVelocity();
    }

    public ArmState getState() {
        return state;
    }

    public ScoreType getScoreType() {
        return scoreType;
    }

    public void setPivotPower(double power) {
        pivot.setPower(power);
    }

    public void setState(ArmState state) {
        this.state = state;
    }

    public void setScoreType(ScoreType scoreType) {
        this.scoreType = scoreType;
    }


    public void setSlidePosition(double pos) {
        slidePid.setSetPoint(pos);
    }

    public void setPivotPosition(double pos) {
        pivotPid.setSetPoint(pos);
    }

    public void setSlidePower(double power) {
        slide.setPower(power);
    }


    public void resetSlideEncoder() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void updatePid() {
        slidePid.setPID(SLIDE_KP, SLIDE_KI, SLIDE_KD);
        pivotPid.setPID(PIVOT_KP, PIVOT_KI, PIVOT_KD);
    }

    @Override
    public void periodic() {
        updatePid();
        telemetry.addLine();
        telemetry.addData("Arm State", state);
        telemetry.addData("Scoring Piece", scoreType);
        telemetry.addData("Pivot target", getPivotTarget());
        telemetry.addData("Pivot pos", pivot.getPosition());
        telemetry.addData("Slide target", getSlideTarget());
        telemetry.addData("Slide pos", getSlidePosition());

        double slideOutput = slidePid.calculate(getSlidePosition());

        double slideFeedforward = SLIDE_KF * Math.sin(Math.toRadians(getPivotTarget() - PIVOT_REST_POSITION));
        if (state == ArmState.STOW) slideFeedforward = 0;

        double pivotOutput = pivotPid.calculate(pivot.getPosition());
        double pivotFeedforward = PIVOT_KF * Math.cos(Math.toRadians(getPivotTarget() - PIVOT_REST_POSITION));

        if (!slideZeroing) setSlidePower(slideOutput + slideFeedforward);

        if (pivotAtSetPoint() && getPivotTarget() == PIVOT_REST_POSITION) pivot.setPower(0);
        else pivot.setPower(pivotOutput + pivotFeedforward);
    }
}
