package org.firstinspires.ftc.teamcode.arm.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmConstants;
import org.firstinspires.ftc.teamcode.arm.Pivot;

import java.util.function.DoubleSupplier;

public class MovePivotCommand extends CommandBase {
    private final Pivot pivot;
    private final Arm arm;
    private final double targetPos;
    private final ElapsedTime timer;
    boolean exceededTime = false;

    public MovePivotCommand(DoubleSupplier targetPos) {
        this.arm = Arm.getInstance();
        pivot = new Pivot();
        this.targetPos = targetPos.getAsDouble();
        timer = new ElapsedTime();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.reset();
        arm.setPivotPosition(targetPos);
    }

    @Override
    public void execute() {
        exceededTime = timer.milliseconds() > ArmConstants.PIVOT_TIMEOUT;
    }

    @Override
    public boolean isFinished() {
        return arm.pivotAtSetPoint() || exceededTime;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Log.i("MovePivotCommand", "===============COMMAND INTERRUPTED===============");
        } else if (exceededTime) {
            arm.setPivotPosition(pivot.getPosition());
            Log.i("MovePivotCommand", "===============COMMAND TIMED OUT AT " + timer.milliseconds() + " MILLISECONDS===============");
        } else {
            Log.i("MovePivotCommand", "===============COMMAND ENDED SAFELY===============");
        }
    }
}