package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;
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

    private GamepadEx driveController;
    private GamepadEx manipController;

    private static final double DRIVE_SENSITIVITY = 1.5;
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

        // Update gamepad objects
        driveController = new GamepadEx(drive);
        manipController = new GamepadEx(manip);

        // Run teleop init
        subsystems.forEach(SubsystemIF::onTeleopInit);

        // Chassis driving
        Chassis.getInstance().setDefaultCommand(new TeleOpDriveCommand(
                () -> applyResponseCurve(driveController.getLeftY(), DRIVE_SENSITIVITY),
                () -> applyResponseCurve(driveController.getLeftX(), DRIVE_SENSITIVITY),
                () -> applyResponseCurve(driveController.getRightX(), ROTATIONAL_SENSITIVITY)
        ));
        new Trigger(() -> driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenActive(Chassis.getInstance()::enableSlowMode)
                .whenInactive(Chassis.getInstance()::disableSlowMode);

        // Reset chassis heading
        driveController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(Commands.runOnce(() -> rumbleInTheKamabal(300, driveController)))
                .whenPressed(Chassis.getInstance()::resetHeading);

        // Toggle field centric
        driveController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(Chassis.getInstance()::toggleRobotCentric);

        // Arm to scoring position
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(ArmCommands.TO_CHAMBER)
                .whenPressed(ArmCommands.TO_BASKET);

        // Arm action
        new Trigger(() -> manipController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenInactive(Commands.either(
                        Commands.defer(ArmCommands.BASKET_TO_STOW),
                        Commands.none(),
                        () -> Arm.getInstance().getState() == Arm.ArmState.SCORING_SAMPLE
                ));

        manipController.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(Commands.runOnce(Arm.getInstance()::resetPivotEncoder));

        // Switch game pieces
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(Commands.runOnce(() -> rumbleInTheKamabal(300, manipController)))
                .whenPressed(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SAMPLE));
        manipController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(Commands.runOnce(() -> rumbleInTheKamabal(300, manipController)))
                .whenPressed(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SPECIMEN));


        // Release sample
        manipController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(Commands.sequence(
                        Commands.waitMillis(300),
                        Commands.runOnce(Claw.getInstance()::openClaw),
                        Commands.waitMillis(ClawConstants.GRAB_DELAY),
                        Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.REST_STATE))
                ));
        Log.i("Fieri", "============INITIALIZED TELEOP 10000+aura MISSION SUCCESSFUL============");
    }


    private double applyResponseCurve(double input, double scale) {
        // Clamp input to the range [-1, 1]
        input = Math.max(-1, Math.min(1, input));

        // Apply exponential response curve
        double output = Math.signum(input) * Math.pow(Math.abs(input), scale);

        return output;
    }


    private void rumbleInTheKamabal(int ms, GamepadEx... gamepads) {
        for(GamepadEx g : gamepads) {
            g.gamepad.rumble(1, 1, ms);
        }
    }


    public Telemetry getTelemetry() {
        return telemetry;
    }



    public void periodic() {
        if (!RobotStatus.isEnabled() && RobotStatus.isTeleop()) RobotStatus.robotState = RobotStatus.RobotState.TELEOP_ENABLED;
        if (!RobotStatus.isEnabled() && !RobotStatus.isTeleop()) RobotStatus.robotState = RobotStatus.RobotState.AUTONOMOUS_ENABLED;

        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.clearBulkCache();
        }

        run();

        telemetry.addLine();
        telemetry.addData("Status", RobotStatus.robotState);
        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.update();
        timer.reset();
    }
}