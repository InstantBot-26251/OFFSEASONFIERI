package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.*;

import java.util.ArrayList;
import java.util.List;

public class RobotMap {
    private HardwareMap hMap;
    private final List<HardwareDevice> devices = new ArrayList<>();

    // OTOS SIGMA RIZZ
    public SparkFunOTOS OTOS;


    // Drive motors
    public DcMotorEx MOTOR_FL;
    public DcMotorEx MOTOR_FR;
    public DcMotorEx MOTOR_BL;
    public DcMotorEx MOTOR_BR;

    // Slide
    public DcMotorEx SLIDE;

    // Pivot
    public DcMotorEx PIVOT;

    // Claw
    public Servo CLAW;
    public CRServo WRIST;


    private static RobotMap instance = null;

    // Returns an instance of this
    public static RobotMap getInstance() {
        if (instance == null) {
            instance = new RobotMap();
        }
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        devices.clear();

        this.hMap = hardwareMap;

        OTOS = hardwareMap.get(SparkFunOTOS.class, "otos");

        MOTOR_FL = hardwareMap.get(DcMotorEx.class, FConstants.MOTOR_FL_NAME);
        MOTOR_FR = hardwareMap.get(DcMotorEx.class, FConstants.MOTOR_FR_NAME);
        MOTOR_BL = hardwareMap.get(DcMotorEx.class, FConstants.MOTOR_BL_NAME);
        MOTOR_BR = hardwareMap.get(DcMotorEx.class, FConstants.MOTOR_BR_NAME);

        SLIDE = hardwareMap.get(DcMotorEx.class, "slide");
        PIVOT = hardwareMap.get(DcMotorEx.class, "pivot");

        CLAW = hardwareMap.get(Servo.class, "claw");
        WRIST = hardwareMap.get(CRServo.class, "wrist");
        addDevices();
    }

    private void addDevices() {
        devices.add(getInstance().OTOS);
        devices.add(getInstance().MOTOR_FL);
        devices.add(getInstance().MOTOR_FR);
        devices.add(getInstance().MOTOR_BL);
        devices.add(getInstance().MOTOR_BR);
        devices.add(getInstance().SLIDE);
        devices.add(getInstance().PIVOT);
        devices.add(getInstance().CLAW);
        devices.add(getInstance().WRIST);
    }

    public List<HardwareDevice> getDevices() {
        return devices;
    }

    // Get hubs
    public List<LynxModule> getLynxModules() {
        return hMap.getAll(LynxModule.class);
    }

    // Get hardwareMap instance
    public HardwareMap getHardwareMap() {
        return this.hMap;
    }
}