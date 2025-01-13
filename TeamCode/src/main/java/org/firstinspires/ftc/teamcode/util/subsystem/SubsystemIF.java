package org.firstinspires.ftc.teamcode.util.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class SubsystemIF extends SubsystemBase {
    public SubsystemIF initialize() { return this; }
    public abstract void onAutonomousInit();
    public abstract void onTeleopInit();
}