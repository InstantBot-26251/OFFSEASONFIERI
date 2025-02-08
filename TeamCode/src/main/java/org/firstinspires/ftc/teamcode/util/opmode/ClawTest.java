package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RobotMap;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawState;

@Config
@TeleOp(name = "Claw Test", group = "Test")
public class ClawTest extends OpMode {
    Claw claw;
    ClawState state;
    public static double clawTarget = 0.0;
    public static double wristTarget = 0.0;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        claw = Claw.getInstance();
        claw.onTeleopInit();
        state = new ClawState();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        telemetry.addData("Claw State:", claw.getState());
        telemetry.update();
    }

    @Override
    public void loop() {
        state.clawPos = clawTarget;
        state.wristPos = wristTarget;

        telemetry.addData("Claw State:", claw.getState());

        claw.setState(state);
        claw.periodic();
    }
}