package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

@Config
public class ArmExtensionSubsystem extends SubsystemBase {
    Telemetry telemetry;
    public MotorGroup extensions;

    public static double P = 0, I = 0, D = 0;
    public static double kSpring = 0;
    public static int pHome = 0, pIntake = 0, pSpecimen = 0, pBucket = 0, pStart = 0;
    public static int pSpecimen_offset = 0;

    public static int target = 0;

    public PIDController pidController;
    private VoltageSensor voltageSensor;
    private States.ArmExtension currentState;


    public ArmExtensionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        currentState = States.ArmExtension.home;

        MotorEx leftExtension = new MotorEx(hardwareMap, "leftExtension");
        MotorEx rightExtension = new MotorEx(hardwareMap, "rightExtension");
        rightExtension.setInverted(true);
        extensions = new MotorGroup(leftExtension, rightExtension);
        extensions.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extensions.stopAndResetEncoder();

        pidController = new PIDController(P, I, D);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    @Override
    public void periodic() {
        telemetry.addData("Extension Target: ", target);
        telemetry.addData("Extension Pos: ", extensions.getCurrentPosition());
    }

    public void moveTo(int target){
        ArmExtensionSubsystem.target = target;
        holdPosition();
    }

    public void holdPosition() {
        extensions.set(calculate());
    }
    public void manual(double power) {
        extensions.set(calculate() + power);
        target = extensions.getCurrentPosition();
    }

    public States.ArmExtension getCurrentState() {
        return currentState;
    }

    public void setState(States.ArmExtension state) {
        currentState = state;
        switch (currentState) {
            case bucket:
                moveTo(pBucket);
                break;
            case intake:
                moveTo(pIntake);
                break;
            case specimen_1:
                moveTo(pSpecimen);
                break;
            case specimen_2:
                moveTo(pSpecimen + pSpecimen_offset);
            case home:
                moveTo(pHome);
                break;
            case start:
                moveTo(pStart);
                break;
        }
    }

    private double calculate() {
        pidController.setPID(P,I,D);
        int current = extensions.getCurrentPosition();

        double power = kSpring - pidController.calculate(current, target);
        // we are subtracting the PID since the springs are constantly trying to extend the arm
        power /= voltageSensor.getVoltage();

        telemetry.addData("Extension Power:", power);
        return power;
    }
}
