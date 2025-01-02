package org.firstinspires.ftc.teamcode.claw;

import static org.firstinspires.ftc.teamcode.claw.ClawConstants.*;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Fieri;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

public class Claw extends SubsystemIF {
    Telemetry telemetry;

    Servo claw;
    Servo wrist;
    Servo jointOne;
    Servo jointTwo;

    ClawState state = new ClawState();

    private static final Claw INSTANCE = new Claw();

    public static Claw getInstance() {
        return INSTANCE;
    }

    private Claw() {
        state.clawPos = REST_STATE.clawPos;
        state.wristPos = REST_STATE.wristPos;
    }

    // INIT

    @Override
    public void onAutonomousInit() {
        telemetry = Fieri.getInstance().getTelemetry();
        configureHardware();
        claw.setPosition(CLAW_CLOSE_POSITION);
        jointTwo.setPosition(0.6);

//        Commands.sequence(
//                Commands.waitUntil(RobotStatus::isEnabled),
//                Commands.runOnce(() -> jointTwo.getController().pwmEnable()),
//                Commands.runOnce(() -> jointTwo.setPosition(1))
//        ).schedule();
    }

    @Override
    public void onTeleopInit() {
        telemetry = Fieri.getInstance().getTelemetry();
        configureHardware();
        setState(REST_STATE);
    }

    // HARDWARE SETUP

    public void configureHardware() {
        claw = RobotMap.getInstance().CLAW;
        wrist = RobotMap.getInstance().WRIST;
    }

    // GETTERS

    public ClawState getState() {
        return state;
    }

    // SETTERS

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

    public void setWrist(double pos) {
        state.wristPos = pos;
    }

    // PERIODIC

    @Override
    public void periodic() {
        claw.setPosition(state.clawPos);
        wrist.setPosition(state.wristPos);
    }
}