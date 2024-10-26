package org.firstinspires.ftc.teamcode.arm;

import static org.firstinspires.ftc.teamcode.arm.ArmConstants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {
    Telemetry telemetry;

    DcMotorEx armMotor;
    DcMotorEx pivotMotor;

    PIDController armPid;
    PIDController pivotPid;

    double armPower = 0;
    double pivotPower = 0;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        pivotMotor = hardwareMap.get(DcMotorEx.class, "rotationMotor");

        armPid = new PIDController(armP, armI, armD);
        pivotPid = new PIDController(pivotP, pivotI, pivotD);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        armMotor.setPower(armPower);
        pivotMotor.setPower(pivotPower);
    }
}
