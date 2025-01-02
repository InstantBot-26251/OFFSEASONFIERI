package org.firstinspires.ftc.teamcode.arm.commands;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.Arm.ArmState;
import org.firstinspires.ftc.teamcode.arm.Arm.ScoreType;
import org.firstinspires.ftc.teamcode.arm.commands.MovePivotCommand;
import org.firstinspires.ftc.teamcode.arm.commands.MoveSlideCommand;
import org.firstinspires.ftc.teamcode.arm.commands.SlideZeroCommand;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

import java.util.function.Supplier;

public class ArmCommands {
    // TO STOW
    public static final Supplier<Command> BASKET_TO_STOW;
    public static final Supplier<Command> CHAMBER_TO_STOW;
    public static final Supplier<Command> SAMPLE_COLLECT_TO_STOW;
    public static final Supplier<Command> SPECIMEN_COLLECT_TO_STOW;

    // TO BASKET
    public static final Supplier<Command> STOW_TO_BASKET;
    public static final Supplier<Command> CHAMBER_TO_BASKET;

    // TO CHAMBER
    public static final Supplier<Command> STOW_TO_CHAMBER;
    public static final Supplier<Command> SPECIMEN_COLLECT_TO_CHAMBER;
    public static final Supplier<Command> BASKET_TO_CHAMBER;

    // TO SAMPLE COLLECT
    public static final Supplier<Command> STOW_TO_SAMPLE_COLLECT;
    public static final Supplier<Command> COLLECT_SAMPLE;
    public static final Supplier<Command> MISSED_SEQUENCE;

    // TO SPECIMEN COLLECT
    public static final Supplier<Command> STOW_TO_SPECIMEN_COLLECT;
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
    public static Command Arm_ACTION;

    static {
        Supplier<Arm> Arm = Arm::getInstance;
        Supplier<Claw> claw = Claw::getInstance;

        BASKET_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> Arm.get().setState(ArmState.STOW)),
                Commands.waitMillis(100),
                new MoveSlideCommand(() -> ArmConstants.Arm_REST_POSITION),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.REST_STATE)),
                new MovePivotCommand(() -> ArmConstants.PIVOT_REST_POSITION)
        );
        CHAMBER_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> Arm.get().setState(ArmState.STOW)),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.REST_STATE)),
                new MoveSlideCommand(() -> ArmConstants.Arm_REST_POSITION),
                new MovePivotCommand(() -> ArmConstants.PIVOT_REST_POSITION)
        );
        SAMPLE_COLLECT_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> Arm.get().setState(ArmState.STOW)),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_COLLECT_STATE)),
                Commands.runOnce(claw.get()::closeClaw),
                Commands.waitMillis(175),
                new MoveSlideCommand(() -> ArmConstants.Arm_REST_POSITION),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.REST_STATE)),
                Commands.waitMillis(150),
                new SlideZeroCommand()
        );
        SPECIMEN_COLLECT_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> Arm.get().setState(ArmState.STOW)),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.REST_STATE)),
                new MoveSlideCommand(() -> ArmConstants.Arm_REST_POSITION),
                new SlideZeroCommand()
        );

        STOW_TO_BASKET = () -> Commands.sequence(
                Commands.runOnce(() -> Arm.get().setState(ArmState.SCORING_SAMPLE)),
                Commands.waitMillis(200),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION),
                Commands.runOnce(() -> claw.get().setWrist(ClawConstants.SAMPLE_SCORING_STATE.wristPos)),
                new MoveSlideCommand(() -> ArmConstants.Arm_BASKET_POSITION),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SAMPLE_SCORING_STATE)),
                Commands.waitMillis(200)
        );
        CHAMBER_TO_BASKET = () -> Commands.sequence(
                Commands.runOnce(() -> Arm.get().setState(ArmState.SCORING_SAMPLE)),
                new MoveSlideCommand(() -> ArmConstants.Arm_BASKET_POSITION)
        );

        SPECIMEN_COLLECT_TO_CHAMBER = () -> Commands.sequence(
                Commands.waitMillis(100),
                Commands.runOnce(() -> Arm.get().setState(ArmState.SCORING_SPECIMEN)),
                new MoveSlideCommand(() -> ArmConstants.Arm_REST_POSITION),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_SCORING_STATE)),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION),
                new MoveSlideCommand(() -> ArmConstants.Arm_CHAMBER_POSITION)
        );


        BASKET_TO_CHAMBER = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_SCORING_STATE)),
                Commands.runOnce(() -> Arm.get().setState(Arm.ArmState.SCORING_SPECIMEN)),
                new MoveArmCommand(() -> ArmConstants.Arm_CHAMBER_POSITION)
        );

        STOW_TO_SAMPLE_COLLECT = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SAMPLE_COLLECTING_STATE)),
                new MoveArmCommand(() -> ArmConstants.Arm_SAMPLE_COLLECT_POSITION),
                Commands.runOnce(() -> Arm.get().setState(Arm.ArmState.COLLECTING_SAMPLE))
        );

        STOW_TO_SPECIMEN_COLLECT = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_COLLECT_STATE)),
                new MoveArmCommand(() -> ArmConstants.Arm_SPECIMEN_COLLECT_POSITION),
                Commands.runOnce(() -> Arm.get().setState(Arm.ArmState.COLLECTING_SPECIMEN))
        );

        CHAMBER_TO_SPECIMEN_COLLECT = () -> Commands.sequence(
                new MoveArmCommand(() -> ArmConstants.Arm_REST_POSITION),
                new MovePivotCommand(() -> ArmConstants.PIVOT_REST_POSITION),
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_COLLECT_STATE)),
                Commands.runOnce(() -> Arm.get().setState(Arm.ArmState.COLLECTING_SPECIMEN)),
                new MoveArmCommand(() -> ArmConstants.Arm_SPECIMEN_COLLECT_POSITION)
        );

        STOW_TO_CHAMBER = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SPECIMEN_SCORING_STATE)),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION),
                new MoveArmCommand(() -> ArmConstants.Arm_CHAMBER_POSITION)
        );

        BASKET_TO_AUTO_ASCENT = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setState(ClawConstants.REST_STATE)),
                new MovePivotCommand(() -> ArmConstants.PIVOT_SCORE_POSITION),
                new MoveArmCommand(() -> 900)
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
                new MoveArmCommand(() -> ArmConstants.Arm_CHAMBER_POSITION - ArmConstants.Arm_CHAMBER_SCORE_OFFSET),
                Commands.waitMillis(200),
                Commands.runOnce(claw.get()::openClaw),
                Commands.waitMillis(ClawConstants.GRAB_DELAY)
        );

        COLLECT_SAMPLE = () -> Commands.sequence(
                Commands.runOnce(() -> Arm.get().setArmPos(Arm.get().getArmPos())),
                Commands.runOnce(() -> Arm.get().setState(Arm.ArmState.STOW)),
                Commands.runOnce(() -> claw.get().setWrist(limelight.get().getServoPos())),
                Commands.waitMillis(175),
                Commands.runOnce(() -> claw.get().setJointTwo(ClawConstants.SAMPLE_COLLECT_JOINT_TWO_POS)),
                Commands.waitMillis(ClawConstants.PIVOT_DELAY),
                Commands.runOnce(() -> claw.get().setJointOne(ClawConstants.SAMPLE_COLLECT_JOINT_ONE_POS)),
                Commands.waitMillis(ClawConstants.GRAB_DELAY)
        );

        MISSED_SEQUENCE = () -> Commands.sequence(
                Commands.runOnce(() -> claw.get().setState(ClawConstants.SAMPLE_COLLECTING_STATE), claw.get()),
                Commands.waitMillis(175),
                Commands.runOnce(() -> Arm.get().setState(Arm.ArmState.COLLECTING_SAMPLE), Arm.get())
        );

    }

    static {
        Supplier<Arm> Arm = Arm::getInstance;
        Supplier<Claw> claw = Claw::getInstance;
        Supplier<LLVision> llVision = LLVision::getInstance;

        TO_STOW = Commands.deferredProxy(() -> {
            switch (Arm.get().getState()) {
                case COLLECTING_SAMPLE:
                    return Commands.defer(SAMPLE_COLLECT_TO_STOW, Arm.get());
                case SCORING_SPECIMEN:
                    return Commands.defer(CHAMBER_TO_STOW, Arm.get());
                case SCORING_SAMPLE:
                    return Commands.defer(BASKET_TO_STOW, Arm.get());
                case COLLECTING_SPECIMEN:
                    return Commands.defer(SPECIMEN_COLLECT_TO_STOW, Arm.get());
                default:
                    return Commands.none();
            }
        }).andThen(new ArmZeroCommand());

        TO_BASKET = Commands.deferredProxy(() -> {
            if (Arm.get().getScoreType() == Arm.ScoreType.SPECIMEN) return Commands.none();
            switch (Arm.get().getState()) {
                case STOW:
                    return Commands.defer(STOW_TO_BASKET, Arm.get());
                case SCORING_SPECIMEN:
                    return Commands.defer(CHAMBER_TO_BASKET, Arm.get());
                default:
                    return Commands.none();
            }
        });

        TO_CHAMBER = Commands.deferredProxy(() -> {
            if (Arm.get().getScoreType() == Arm.ScoreType.SAMPLE) return Commands.none();
            switch (Arm.get().getState()) {
                case COLLECTING_SPECIMEN:
                    return Commands.defer(SPECIMEN_COLLECT_TO_CHAMBER, Arm.get());
                case SCORING_SAMPLE:
                    return Commands.defer(BASKET_TO_CHAMBER, Arm.get());
                default:
                    return Commands.none();
            }
        });

        TO_COLLECT = Commands.deferredProxy(() -> {
            if (Arm.get().getState() == Arm.ArmState.STOW) {
                if (Arm.get().getScoreType() == Arm.ScoreType.SPECIMEN) {
                    return Commands.defer(STOW_TO_SPECIMEN_COLLECT, Arm.get());
                } else {
                    return Commands.defer(STOW_TO_SAMPLE_COLLECT, Arm.get());
                }
            } else return Commands.none();
        });

        Arm_ACTION = Commands.deferredProxy(() -> {
            switch (Arm.get().getState()) {
                case COLLECTING_SAMPLE:
                    return Commands.sequence(
                            Commands.defer(COLLECT_SAMPLE, claw.get()),
                            Commands.defer(GRAB, claw.get()),
                            Commands.waitMillis(100),
                            Commands.either(
                                    // If claw is in sight (missed collection)
                                    Commands.defer(MISSED_SEQUENCE, claw.get()),
                                    // If claw is out of sight (collected)
                                    Commands.defer(SAMPLE_COLLECT_TO_STOW, Arm.get()),
                                    // Condition
                            ),
                    );
                case COLLECTING_SPECIMEN:
                    return Commands.sequence(
                            Commands.defer(GRAB, claw.get()),
                            Commands.defer(SPECIMEN_COLLECT_TO_CHAMBER, Arm.get())
                    );
                case SCORING_SAMPLE:
                    return Commands.sequence(
                            Commands.defer(RELEASE, claw.get())
//                            Commands.defer(BASKET_TO_STOW, Arm.get())
                    );
                case SCORING_SPECIMEN:
                    return Commands.sequence(
                            Commands.defer(SCORE_SPECIMEN, Arm.get(), claw.get()),
                            Commands.defer(CHAMBER_TO_STOW, Arm.get())
                    );
                case STOW:
                    return Commands.either(
                            Commands.defer(STOW_TO_SAMPLE_COLLECT),
                            Commands.defer(STOW_TO_SPECIMEN_COLLECT),
                            () -> Arm.get().getScoreType() == Arm.ScoreType.SAMPLE
                    );
                default:
                    return Commands.none();
            }
        });
    }
}