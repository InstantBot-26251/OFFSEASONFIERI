/*** package org.firstinspires.ftc.teamcode.chassis.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.chassis.Chassis;

import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {
    Chassis chassis;
    DoubleSupplier fwd, str, rot;

    public TeleopDriveCommand(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot) {
        this.fwd = fwd;
        this.str = str;
        this.rot = rot;

        addRequirements(chassis);
    }

    // @Override
    public void execute() {
        chassis.setTeleopMovementVectors(fwd.getAsDouble(), str.getAsDouble(), rot.getAsDouble());
    }
}
***/