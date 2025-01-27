package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.util.hardware.InstantMotor;

public class Pivot {
    InstantMotor pivot;
    public Pivot(){
        pivot = new InstantMotor(RobotMap.getInstance().PIVOT);
    }
    public void setPower(double power) {
        pivot.setPower(power);
    }
    public void resetEncoder() {
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double getPosition() {
        return pivot.getCurrentPosition();
    }
}
