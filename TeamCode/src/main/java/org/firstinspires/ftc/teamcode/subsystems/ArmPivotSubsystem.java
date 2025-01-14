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
    public DcMotorEx leftPivot, rightPivot;
    IntSupplier extensionAmount;

    public static double P = 0.014, I = 0.00000, D = 0.0035;
    public static double kCos = 0.05, kExt = 0.0001, kHelp = 0;
    public static int ticksPerRev = 1772;
    public static int pHome = 0, pBucket = 465, pSpecimen = 185, pSpecimenIntake = 60, pIntake = 0, pStart = 200;
    public static double tolerance = 15;


    public static int target = 0;
    public static int max = 480;
    public static double manualPower = 0.2;
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

        leftPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightPivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        resetEncoder();

        pidController = new PIDController(P, I, D);
        pidController.setTolerance(tolerance);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        target = pStart;
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
    public void manual(boolean forward) {
        double power;
        if(forward) {
            power = manualPower;
        }else {
            power = manualPower * -1;
        }
        leftPivot.setPower(power);
        rightPivot.setPower(power);
        //rightExtension.setPower(p);
    }

    public void resetTarget() {
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
            case specimenIntake:
                moveTo(pSpecimenIntake);
                break;
            case start:
                moveTo(pStart);
                break;
            case home:
                moveTo(pHome);
                break;
        }
    }

    private double calculate() {
        pidController.setPID(P,I,D);
        pidController.setTolerance(tolerance);
        int current = leftPivot.getCurrentPosition();

        double power = pidController.calculate(current, target);
        double angle = (2 * Math.PI * current) / ticksPerRev;

        power += kCos * Math.cos(angle) + kExt * extensionAmount.getAsInt() + kHelp;
        power /= voltageSensor.getVoltage();

        telemetry.addData("Pivot Power:", "%.6f", power);
        telemetry.addData("Pivot Angle:", Math.toDegrees(angle));
        return power;
    }

    public void resetEncoder() {
        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}