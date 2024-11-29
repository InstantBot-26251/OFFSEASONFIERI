package org.firstinspires.ftc.teamcode.subsystems.arm.chassis.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.arm.chassis.Chassis;

public class ToggleRobotCentricCommand extends CommandBase {
    private final Chassis chassis;

    public ToggleRobotCentricCommand(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        chassis.toggleRobotCentric();
    }
}
