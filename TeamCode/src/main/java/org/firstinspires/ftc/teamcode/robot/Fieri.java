package org.firstinspires.ftc.teamcode.robot;

import android.transition.Slide;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm.commands.ArmCommands;
import org.firstinspires.ftc.teamcode.arm.commands.*;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.util.subsystem.SubsystemIF;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.ArrayList;
import java.util.List;


public class Fieri extends Robot {
    private Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

    private final List<SubsystemIF> subsystems = new ArrayList<>();

    private static final Fieri INSTANCE = new Fieri();

    public static Fieri getInstance() {
        return INSTANCE;
    }

    private GamepadEx avy;
    private GamepadEx ishu;

    private static final double D_RESPONSE_CURVE = 1.5;
    private static final double M_RESPONSE_CURVE = 1.111111;
    private static final double ROTATIONAL_SENSITIVITY = 1.5;
    private static final double TRIGGER_DEADZONE = 0.1;

    private final ElapsedTime timer = new ElapsedTime();

    private Fieri() {
        reset();
        robotInit();
        Log.i("Fieri", "===============FIERI CREATED SKIBDI RIZZ MISSION PASSED===============");
    }

    // Resets the robot and command scheduler
    @Override
    public void reset() {
        RobotStatus.robotState = RobotStatus.RobotState.DISABLED;
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
        Log.i("Fieri", "===============COMMAND SCHEDULER CLEARED 10000+Aura ===============");
    }

    // Registers all subsystems to the command scheduler
    private void registerSubsystems() {
        for (SubsystemIF s : subsystems) {
            register(s);
        }
    }

    // Initializes all subsystems and adds them to the list. New subsystems should be added here.
    private void robotInit() {
        subsystems.clear();
        subsystems.add(Chassis.getInstance().initialize());
        subsystems.add(Arm.getInstance().initialize());
        subsystems.add(Claw.getInstance().initialize());
        registerSubsystems();
    }

    // Runs the process of disabling the robot
    public void disabledInit() {
        RobotStatus.robotState = RobotStatus.RobotState.DISABLED;
        telemetry = FtcDashboard.getInstance().getTelemetry();
        Log.i("Fieri", "===============ROBOT DISABLED===============");
    }

    // Runs autonomous initialization sequence
    public void autonomousInit(Telemetry telemetry, HardwareMap hardwareMap) {
        reset();
        registerSubsystems();
        RobotStatus.robotState = RobotStatus.RobotState.AUTONOMOUS_INIT;

        // Update telemetry and hardwareMap objects
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotMap.getInstance().init(hardwareMap);
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Run autonomous init
        subsystems.forEach(SubsystemIF::onAutonomousInit);
        Log.i("Fieri", "============INITIALIZED AUTONOMOUS============");
    }

    // Runs teleop initialization sequence and binds controls
    public void teleopInit(Telemetry telemetry, HardwareMap hardwareMap, Gamepad drive, Gamepad manip) {
        reset();
        registerSubsystems();
        RobotStatus.robotState = RobotStatus.RobotState.TELEOP_INIT;

        // Update telemetry and hardwareMap objects
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotMap.getInstance().init(hardwareMap);
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        avy = new GamepadEx(drive);
        ishu = new GamepadEx(manip);
        double SlidePower = applyResponseCurve(ishu.getLeftY(), M_RESPONSE_CURVE);
        double PivotPower = applyResponseCurve(ishu.getRightY(), M_RESPONSE_CURVE);

        subsystems.forEach(SubsystemIF::onTeleopInit);

        // Drive Controls
        Chassis.getInstance().setDefaultCommand(new TeleOpDriveCommand(
                () -> applyResponseCurve(avy.getLeftY(), D_RESPONSE_CURVE),
                () -> applyResponseCurve(avy.getLeftX(), D_RESPONSE_CURVE),
                () -> applyResponseCurve(avy.getRightX(), ROTATIONAL_SENSITIVITY)
        ));
        new Trigger(() -> avy.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenActive(Chassis.getInstance()::enableSlowMode)
                .whenInactive(Chassis.getInstance()::disableSlowMode);

        // Reset Heading (IMPORTANTIAL)
        avy.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(Chassis.getInstance()::resetHeading);

        // Turn Field Centric ON/OFF (IMPORTANTIAL)
        avy.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(Chassis.getInstance()::toggleRobotCentric);


        // Manip Controls

        // Scoring Position
        ishu.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(ArmCommands.TO_CHAMBER)
                .whenPressed(ArmCommands.TO_BASKET);

        // Stowed Position
        new Trigger(() -> ishu.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenInactive(Commands.either(
                        Commands.defer(ArmCommands.TO_STOW_S),
                        Commands.none(),
                        () -> Arm.getInstance().getState() == Arm.ArmState.SCORING_SAMPLE
                ));

        // Collecting Position
        ishu.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(ArmCommands.TO_COLLECT);

        // Switch From Sample/Specimen for Software (IMPORTANTIAL)
        ishu.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SAMPLE));
        ishu.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SPECIMEN));


        // MANUAL CONTROLS
        new SetSlidePowerMANUAL(SlidePower);
        new SetPivotPowerMANUAL(PivotPower);

        // Release Sample/Specimen
        ishu.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(Commands.sequence(
                        Commands.waitMillis(300),
                        Commands.runOnce(Claw.getInstance()::openClaw),
                        Commands.waitMillis(ClawConstants.GRAB_DELAY),
                        Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.REST_STATE))
                     ));
        // Collect Sample/Specimen
        ishu.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(Commands.sequence(
                        Commands.waitMillis(300),
                        Commands.runOnce(Claw.getInstance()::closeClaw),
                        Commands.waitMillis(ClawConstants.GRAB_DELAY),
                        Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.REST_STATE))
                ));
        // Rotate Wrist LEFT
        ishu.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(Commands.sequence(
                        Commands.waitMillis(300),
                        Commands.runOnce(Claw.getInstance()::setWristLeft),
                        Commands.waitMillis(ClawConstants.GRAB_DELAY),
                        Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.REST_STATE))
                ));
        // Rotate Wrist RIGHT
        ishu.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(Commands.sequence(
                        Commands.waitMillis(300),
                        Commands.runOnce(Claw.getInstance()::setWristRight),
                        Commands.waitMillis(ClawConstants.GRAB_DELAY),
                        Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.REST_STATE))
                ));
        Log.i("Fieri", "============INITIALIZED TELEOP 10000+aura MISSION SUCCESSFUL============");
    }


    private double applyResponseCurve(double input, double scale) {
        // Clamp input to -1, 1
        input = Math.max(-1, Math.min(1, input));

        // Apply response curve
        double output = Math.signum(input) * Math.pow(Math.abs(input), scale);

        return output;
    }


    public Telemetry getTelemetry() {
        return telemetry;
    }



    public void periodic() {
        if (!RobotStatus.isEnabled() && RobotStatus.isTeleop()) RobotStatus.robotState = RobotStatus.RobotState.TELEOP_ENABLED;
        if (!RobotStatus.isEnabled() && !RobotStatus.isTeleop()) RobotStatus.robotState = RobotStatus.RobotState.AUTONOMOUS_ENABLED;

        // Random stuff i learned from FTC 18079's code
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.clearBulkCache();
        }

        run();

        telemetry.addLine();
        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.addData("Status", RobotStatus.robotState);
        telemetry.update();
        timer.reset();
    }
}