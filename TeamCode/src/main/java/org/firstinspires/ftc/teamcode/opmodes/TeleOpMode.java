package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.RobotStatus.Alliance.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Fieri;
import org.firstinspires.ftc.teamcode.robot.RobotStatus;

@Config
@TeleOp(name = "Period of Manual Control")
public class TeleOpMode extends OpMode {
    private final Fieri fieri = Fieri.getInstance();

    @Override
    public void init() {
        fieri.teleopInit(telemetry, hardwareMap, gamepad1, gamepad2);
        telemetry.addData("Alliance", RobotStatus.alliance);
        telemetry.addData("Status", RobotStatus.robotState);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.right_bumper) {
            switch (RobotStatus.alliance) {
                case RED:
                    RobotStatus.alliance = BLUE;
                    break;
                case BLUE:
                    RobotStatus.alliance = RED;
                    break;
            }
        }
    }
    @Override
    public void loop() {
        fieri.periodic();
    }
    @Override
    public void stop() {
        fieri.disabledInit();
    }
}