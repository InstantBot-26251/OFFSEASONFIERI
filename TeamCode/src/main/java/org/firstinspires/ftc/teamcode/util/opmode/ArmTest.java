package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.Pivot;

@Config
@TeleOp(name = "Arm Test", group = "Tests")
public class ArmTest extends OpMode {
    Pivot pivot;
    Arm arm;
    public static double pivotTarget = 0;
    public static double slideTarget = 0;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        arm = Arm.getInstance();
        arm.onTeleopInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        telemetry.addData("Pivot", pivot.getPosition());
        telemetry.addData("Target pos", arm.getPivotTarget());
        telemetry.addData("Target angle", arm.getPivotTarget());
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            arm.updatePid();
        }

        arm.setPivotPower(-gamepad2.left_stick_y);

        telemetry.addData("Slide Pos", arm.getSlidePosition());
        telemetry.addData("Pivot Pos", pivot.getPosition());
        telemetry.addData("Gamepad 2 (reversed)", gamepad2.left_stick_y);
        telemetry.update();
    }
}