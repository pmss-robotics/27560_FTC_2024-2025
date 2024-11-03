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

import java.util.function.IntSupplier;



@Config
public class ArmPivotSubsystem extends SubsystemBase {
    Telemetry telemetry;
    MotorGroup pivots;
    IntSupplier extensionAmount;

    public static double P = 0, I = 0, D = 0;
    public static double kCos = 0,kExt;
    public static double ticksPerRev = 0;
    public static int pBucket = 0, pSpecimen = 0, pIntake = 0;

    public static int target = 0;
    private States.ArmPivot currentState;

    private PIDController pidController;
    private VoltageSensor voltageSensor;

    public ArmPivotSubsystem(HardwareMap hardwareMap, Telemetry telemetry, IntSupplier extensionAmount) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        this.extensionAmount = extensionAmount;
        currentState = States.ArmPivot.intake;

        MotorEx leftPivot = new MotorEx(hardwareMap, "leftPivot", Motor.GoBILDA.RPM_435);
        MotorEx rightPivot = new MotorEx(hardwareMap, "rightPivot");
        rightPivot.setInverted(true);
        pivots = new MotorGroup(leftPivot, rightPivot);
        pivots.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        pivots.stopAndResetEncoder();

        pidController = new PIDController(P, I, D);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    @Override
    public void periodic() {
        telemetry.addData("Pivot Target: ", target);
        telemetry.addData("Pivot Pos: ", pivots.getCurrentPosition());
    }

    public void moveTo(int target){
        this.target = target;
        holdPosition();
    }

    public void holdPosition() {
        pivots.set(calculate());
    }
    public void manual(double power) {
        pivots.set(calculate() + power);
        target = pivots.getCurrentPosition();
    }

    public States.ArmPivot getCurrentState() {
        return currentState;
    }

    public void setState(States.ArmPivot state) {
        currentState = state;
        switch (currentState) {
            case bucket:
                moveTo(pBucket);
                break;
            case intake:
                moveTo(pIntake);
                break;
            case specimen:
                moveTo(pSpecimen);
                break;
        }
    }

    private double calculate() {
        pidController.setPID(P,I,D);
        int current = pivots.getCurrentPosition();

        double power = pidController.calculate(current, target);
        power += kCos * Math.cos(target / ticksPerRev) + kExt * extensionAmount.getAsInt();
        power /= voltageSensor.getVoltage();

        telemetry.addData("Pivot Power:", power);
        return power;
    }
}
