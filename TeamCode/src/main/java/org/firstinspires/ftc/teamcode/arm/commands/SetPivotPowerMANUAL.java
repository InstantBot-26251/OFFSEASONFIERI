package org.firstinspires.ftc.teamcode.arm.commands;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.Pivot;
import com.arcrobotics.ftclib.command.CommandBase;
import android.util.Log;

public class SetPivotPowerMANUAL extends CommandBase {
    private final Pivot pivot;
    private final Arm arm;
    public final double power;

    public SetPivotPowerMANUAL(double power) {
        this.arm = Arm.getInstance()    ;
        this.power = power;
        this.pivot = new Pivot();
        addRequirements(arm);
    }
    @Override
    public void initialize() {
        Log.i("SetSlidePowerCommand", "===============COMMAND INITIALIZED===============" );
    }

    @Override
    public void execute() {
        arm.setPivotPower(power);
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
