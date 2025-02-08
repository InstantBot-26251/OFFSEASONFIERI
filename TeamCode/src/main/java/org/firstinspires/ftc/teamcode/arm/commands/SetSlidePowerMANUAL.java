package org.firstinspires.ftc.teamcode.arm.commands;

import org.firstinspires.ftc.teamcode.arm.Arm;
import com.arcrobotics.ftclib.command.CommandBase;
import android.util.Log;

public class SetSlidePowerMANUAL extends CommandBase {
    private final Arm arm;
    public final double power;

    public SetSlidePowerMANUAL(double power) {
        this.power = power;
        this.arm = Arm.getInstance();
        addRequirements(arm);
    }
    @Override
    public void initialize() {
        Log.i("SetSlidePowerCommand", "===============COMMAND INITIALIZED===============" );
    }

    @Override
    public void execute() {
        arm.setSlidePower(power);
        Log.i("SetSlidePowerCommand", "Slide power set to: " + power);
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until explicitly interrupted.
        return false;
    }

    public double getPower() {
        return power;
    }
}
