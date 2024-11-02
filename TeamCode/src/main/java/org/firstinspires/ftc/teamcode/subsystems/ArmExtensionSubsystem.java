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

@Config
public class ArmExtensionSubsystem extends SubsystemBase {
    Telemetry telemetry;
    public MotorGroup extensions;

    public static double P = 0, I = 0, D = 0;
    public static double kSpring = 0;

    public int target = 0;

    private PIDController pidController;
    private VoltageSensor voltageSensor;


    public ArmExtensionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
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
        this.target = target;
        holdPosition();
    }

    public void holdPosition() {
        extensions.set(calculate());
    }
    public void manual(double power) {
        extensions.set(calculate() + power);
        target = extensions.getCurrentPosition();
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
