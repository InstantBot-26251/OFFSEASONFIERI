package org.firstinspires.ftc.teamcode.arm.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.Arm.ArmState;
import org.firstinspires.ftc.teamcode.arm.Arm.ScoreType;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.function.Supplier;

public class ArmCommands extends CommandBase {
    // TO STOW
    public static final Supplier<Command> BASKET_TO_STOW;
    public static final Supplier<Command> CHAMBER_TO_STOW;
    public static final Supplier<Command> SAMPLE_COLLECT_TO_STOW;
    public static final Supplier<Command> SPECIMEN_COLLECT_TO_STOW;

    // TO BASKET
    public static final Supplier<Command> STOW_TO_BASKET;

    // TO CHAMBER
    public static final Supplier<Command> STOW_TO_CHAMBER;
    public static final Supplier<Command> SPECIMEN_COLLECT_TO_CHAMBER;

    // TO SAMPLE COLLECT
    public static final Supplier<Command> STOW_TO_SAMPLE_COLLECT = null;

    // TO SPECIMEN COLLECT
    public static final Supplier<Command> STOW_TO_SPECIMEN_COLLECT = null;
    public static final Supplier<Command> COLLECT_SAMPLE;
    public static final Supplier<Command> CHAMBER_TO_SPECIMEN_COLLECT;

    // ACTIONS
    public static final Supplier<Command> GRAB;
    public static final Supplier<Command> RELEASE;
    public static final Supplier<Command> SCORE_SPECIMEN;

    // MOVEMENT COMMANDS
    public static Command TO_STOW;
    public static Command TO_BASKET;
    public static Command TO_CHAMBER;
    public static Command TO_COLLECT;
    public static Command SLIDE_ACTION;

    static {
        Supplier<Arm> arm = Arm::getInstance;
        Supplier<Claw> claw = Claw::getInstance;

        Gamepad gamepad2 = getGamepad2();

        BASKET_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(ArmState.STOW)),
                Commands.waitMillis(100),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                Commands.runOnce(() -> {
                    if (gamepad2.x) claw.get().closeClaw();
                    else if (gamepad2.b) claw.get().openClaw();
                }),
                new MovePivotCommand(() -> ArmConstants.PIVOT_REST_POSITION)
        );

        CHAMBER_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(ArmState.STOW)),
                Commands.runOnce(() -> {
                    if (gamepad2.x) claw.get().closeClaw();
                    else if (gamepad2.b) claw.get().openClaw();
                }),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                new MovePivotCommand(() -> ArmConstants.PIVOT_REST_POSITION)
        );

        SAMPLE_COLLECT_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(ArmState.STOW)),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_COLLECT_STATE)),
                Commands.runOnce(() -> {
                    if (gamepad2.x) claw.get().closeClaw();
                }),
                Commands.waitMillis(175),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                Commands.runOnce(() -> {
                    if (gamepad2.b) claw.get().openClaw();
                }),
                Commands.waitMillis(150),
                new SlideZeroCommand()
        );

        SPECIMEN_COLLECT_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(ArmState.STOW)),
                Commands.runOnce(() -> {
                    if (gamepad2.b) claw.get().openClaw();
                }),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                new SlideZeroCommand()
        );

        STOW_TO_BASKET = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setState(ArmState.SCORING_SAMPLE)),
                Commands.waitMillis(200),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_BASKET_POSITION),
                Commands.runOnce(() -> {
                    if (gamepad2.right_bumper) claw.get().setWrist(ClawConstants.SAMPLE_SCORING_STATE.wristPos + 1);
                    else if (gamepad2.left_bumper) claw.get().setWrist(ClawConstants.SAMPLE_SCORING_STATE.wristPos - 1);
                }),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_BASKET_POSITION),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SAMPLE_SCORING_STATE)),
                Commands.waitMillis(200)
        );

        SPECIMEN_COLLECT_TO_CHAMBER = () -> Commands.sequence(
                Commands.waitMillis(100),
                Commands.runOnce(() -> arm.get().setState(ArmState.SCORING_SPECIMEN)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                Commands.runOnce(() -> {
                    if (gamepad2.right_bumper) claw.get().setWrist(ClawConstants.SPECIMEN_SCORING_STATE.wristPos + 1);
                    else if (gamepad2.left_bumper) claw.get().setWrist(ClawConstants.SPECIMEN_SCORING_STATE.wristPos - 1);
                }),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_SPECIMEN_POSITION),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_CHAMBER_POSITION)
        );

        STOW_TO_CHAMBER = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_SCORING_STATE)),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_SPECIMEN_POSITION),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_CHAMBER_POSITION)
        );

        GRAB = () -> Commands.sequence(
                Commands.runOnce(claw.get()::closeClaw),
                Commands.waitMillis(ClawConstants.GRAB_DELAY)
        );

        RELEASE = () -> Commands.sequence(
                Commands.runOnce(claw.get()::openClaw),
                Commands.waitMillis(ClawConstants.GRAB_DELAY)
        );

        SCORE_SPECIMEN = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_SCORE_STATE)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_CHAMBER_POSITION - ArmConstants.SLIDE_CHAMBER_SCORE_OFFSET),
                Commands.waitMillis(200),
                Commands.runOnce(claw.get()::openClaw),
                Commands.waitMillis(ClawConstants.GRAB_DELAY)
        );
        CHAMBER_TO_SPECIMEN_COLLECT   = () -> Commands.sequence(
                new MoveSlideCommand(() -> ArmConstants.SLIDE_REST_POSITION),
                new MovePivotCommand(() -> ArmConstants.PIVOT_REST_POSITION),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_COLLECT_STATE)),
                Commands.runOnce(() -> arm.get().setState(ArmState.COLLECTING_SPECIMEN)),
                new MoveSlideCommand(() -> ArmConstants.SLIDE_SPECIMEN_COLLECT_POSITION)
        );

        COLLECT_SAMPLE = () -> Commands.sequence(
                Commands.runOnce(() -> arm.get().setSlidePosition(arm.get().getSlidePosition())),
                Commands.runOnce(() -> arm.get().setState(ArmState.STOW)),
                Commands.waitMillis(175),
                Commands.runOnce(() -> claw.get().closeClaw()),
                Commands.waitMillis(ClawConstants.GRAB_DELAY)
        );

    }

    static {
        Supplier<Arm> arm = Arm::getInstance;
        Supplier<Claw> claw = Claw::getInstance;

        TO_STOW = Commands.deferredProxy(() -> {
            switch (arm.get().getState()) {
                case COLLECTING_SAMPLE:
                    return Commands.defer(SAMPLE_COLLECT_TO_STOW, arm.get());
                case SCORING_SPECIMEN:
                    return Commands.defer(CHAMBER_TO_STOW, arm.get());
                case SCORING_SAMPLE:
                    return Commands.defer(BASKET_TO_STOW, arm.get());
                case COLLECTING_SPECIMEN:
                    return Commands.defer(SPECIMEN_COLLECT_TO_STOW, arm.get());
                default:
                    return Commands.none();
            }
        }).andThen(new SlideZeroCommand());

        TO_BASKET = Commands.deferredProxy(() -> {
            if (arm.get().getScoreType() == ScoreType.SPECIMEN) return Commands.none();
            switch (arm.get().getState()) {
                case STOW:
                    return Commands.defer(STOW_TO_BASKET, arm.get());
                default:
                    return Commands.none();
            }
        });

        TO_CHAMBER = Commands.deferredProxy(() -> {
            if (arm.get().getScoreType() == ScoreType.SAMPLE) return Commands.none();
            switch (arm.get().getState()) {
                case COLLECTING_SPECIMEN:
                    return Commands.defer(SPECIMEN_COLLECT_TO_CHAMBER, arm.get());
                default:
                    return Commands.none();
            }
        });

        TO_COLLECT = Commands.deferredProxy(() -> {
            if (arm.get().getState() == ArmState.STOW) {
                if (arm.get().getScoreType() == ScoreType.SPECIMEN) {
                    return Commands.defer(STOW_TO_SPECIMEN_COLLECT, arm.get());
                } else {
                    return Commands.defer(STOW_TO_SAMPLE_COLLECT, arm.get());
                }
            } else return Commands.none();
        });

        SLIDE_ACTION = Commands.deferredProxy(() -> {
            switch (arm.get().getState()) {
                case COLLECTING_SAMPLE:
                    return Commands.sequence(
                            Commands.defer(COLLECT_SAMPLE, claw.get()),
                            Commands.defer(GRAB, claw.get())
                    );
                case COLLECTING_SPECIMEN:
                    return Commands.sequence(
                            Commands.defer(GRAB, claw.get()),
                            Commands.defer(SPECIMEN_COLLECT_TO_CHAMBER, arm.get())
                    );
                case SCORING_SAMPLE:
                    return Commands.sequence(
                            Commands.defer(RELEASE, claw.get())
                    );
                case SCORING_SPECIMEN:
                    return Commands.sequence(
                            Commands.defer(SCORE_SPECIMEN, arm.get(), claw.get()),
                            Commands.defer(CHAMBER_TO_STOW, arm.get())
                    );
                case STOW:
                    return Commands.either(
                            Commands.defer(STOW_TO_SAMPLE_COLLECT),
                            Commands.defer(STOW_TO_SPECIMEN_COLLECT),
                            () -> arm.get().getScoreType() == ScoreType.SAMPLE
                    );
                default:
                    return Commands.none();
            }
        });
    }
    private static Gamepad getGamepad2() {
        return Arm.getInstance().getGamepad();
    }
}