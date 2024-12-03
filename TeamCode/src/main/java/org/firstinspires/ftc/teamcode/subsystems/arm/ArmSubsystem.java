package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile.State;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotorEx armMotor;
    private final PIDFController armPid;
    private TrapezoidProfile PivotMotionProfile;
    private State goalPivotState;
    private State currentPivotState;



    public ArmSubsystem(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armPid = new PIDFController(
                ArmConstants.ARM_KP,
                ArmConstants.ARM_KI,
                ArmConstants.ARM_KD,
                ArmConstants.ARM_KF
        );

        goalPivotState = new State(0, 0);
        currentPivotState = new State(0, 0);
    }

    public void setPower(double input) {
        armMotor.setPower(input);
    }

    public void toPoint(double position) {
        goalPivotState = new State(position, 0);
        PivotMotionProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION),
                goalPivotState,
                currentPivotState
        );
    }

    public void update() {
        double dt = 0.02; // Loop time
        currentPivotState = PivotMotionProfile.calculate(dt);

        double feedforward = ArmConstants.ARM_KF * currentPivotState.velocity;
        double feedback = armPid.calculate(armMotor.getCurrentPosition(), currentPivotState.position);

        armMotor.setPower(feedforward + feedback);
    }

    public void limitCheck() {

    }

    public void stop() {
        armMotor.setPower(0);
    }

    public double getEncoderValue() {
        return armMotor.getCurrentPosition();
    }
}
