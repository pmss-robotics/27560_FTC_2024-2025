package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.function.IntSupplier;



@Config
public class ArmPivotSubsystem extends SubsystemBase {
    Telemetry telemetry;
    // left pivot is the lead motor
    DcMotorEx leftPivot, rightPivot;
    IntSupplier extensionAmount;

    public static double P = 0, I = 0, D = 0;
    public static double kCos = 0,kExt;
    public static double ticksPerRev = 0;
    public static int pBucket = 0, pSpecimen = 0, pIntake = 0, pStart = 0;

    public static int target = 0;
    private States.ArmPivot currentState;

    public PIDController pidController;
    private VoltageSensor voltageSensor;

    public ArmPivotSubsystem(HardwareMap hardwareMap, Telemetry telemetry, IntSupplier extensionAmount) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        this.extensionAmount = extensionAmount;
        currentState = States.ArmPivot.intake;

        leftPivot = hardwareMap.get(DcMotorEx.class, "leftPivot");
        rightPivot = hardwareMap.get(DcMotorEx.class, "rightPivot");
        rightPivot.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        resetEncoder();

        pidController = new PIDController(P, I, D);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        target = leftPivot.getCurrentPosition();
    }
    @Override
    public void periodic() {
        telemetry.addData("Pivot Target: ", target);
        telemetry.addData("Pivot Pos: ", leftPivot.getCurrentPosition());
    }

    public void moveTo(int target){
        ArmPivotSubsystem.target = target;
        holdPosition();
    }

    public void holdPosition() {
        double power = calculate();
        leftPivot.setPower(power);
        rightPivot.setPower(power);
    }
    public void manual(double power) {
        double p = calculate();
        leftPivot.setPower(p + power);
        rightPivot.setPower(p + power);
        target = leftPivot.getCurrentPosition();
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
            case start:
                moveTo(pStart);
                break;
        }
    }

    private double calculate() {
        pidController.setPID(P,I,D);
        int current = leftPivot.getCurrentPosition();

        double power = pidController.calculate(current, target);
        power += kCos * Math.cos(target / ticksPerRev) + kExt * extensionAmount.getAsInt();
        power /= voltageSensor.getVoltage();

        telemetry.addData("Pivot Power:", power);
        return power;
    }

    public void resetEncoder() {
        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
