package org.firstinspires.ftc.teamcode.claw;

import static org.firstinspires.ftc.teamcode.claw.ClawConstants.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Fieri;
import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.util.subsystem.SubsystemTemplate;

public class Claw extends SubsystemTemplate {
    Telemetry telemetry;

    Servo claw;
    CRServo wrist;

    ClawState state = new ClawState();

    private static final Claw INSTANCE = new Claw();

    public static Claw getInstance() {
        return INSTANCE;
    }

    private Claw() {
        state.clawPos = REST_STATE.clawPos;
        state.wristPos = REST_STATE.wristPos;
    }

    @Override
    public void onAutonomousInit() {
        telemetry = Fieri.getInstance().getTelemetry();
        configureHardware();
        claw.setPosition(CLAW_CLOSE_POSITION);
    }

    @Override
    public void onTeleopInit() {
        telemetry = Fieri.getInstance().getTelemetry();
        configureHardware();
        setState(REST_STATE);
    }

    public void configureHardware() {
        claw = RobotMap.getInstance().CLAW;
        wrist = RobotMap.getInstance().WRIST;
    }


    public ClawState getState() {
        return state;
    }


    public void setState(ClawState targetState) {
        state.clawPos = targetState.clawPos;
        state.wristPos = targetState.wristPos;
    }

    public void openClaw() {
        state.clawPos = CLAW_OPEN_POSITION;
    }

    public void closeClaw() {
        state.clawPos = CLAW_CLOSE_POSITION;
    }

    public void setWristPower(double power) {
        state.wristPos = power;
    }

    public void setWristRight() {
        setWristPower(-1);
    }

    public void setWristLeft() {
        setWristPower(1);
    }

    @Override
    public void periodic() {
        claw.setPosition(state.clawPos);
        wrist.setPower(state.wristPos);
    }
}