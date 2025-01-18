package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

@Config
public class ArmExtensionSubsystem extends SubsystemBase {
    Telemetry telemetry;
    public DcMotorEx leftExtension, rightExtension;

    public static double P = -0.07, I = 0, D = 0;
    public static double kSpring = -0.008;
    //max 980
    public static int pHome = 5, pIntake = 1590, pSpecimen = 1400, pBucket = 955, pStart = 0;
    // 800
    public static int pSpecimen_offset = 0;

    public static int target = 0;
    public static double tolerance = 20;

    public static double manualPower = 0.5;

    public PIDController pidController;
    private VoltageSensor voltageSensor;
    private States.ArmExtension currentState;


    public ArmExtensionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        currentState = States.ArmExtension.home;

        leftExtension = hardwareMap.get(DcMotorEx.class, "leftExtension");
        //rightExtension = hardwareMap.get(DcMotorEx.class, "rightExtension");

        leftExtension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //rightExtension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        pidController = new PIDController(P, I, D);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        target = pStart;
    }
    @Override
    public void periodic() {
        telemetry.addData("Extension Target: ", target);
        telemetry.addData("Extension Pos: ", leftExtension.getCurrentPosition());
    }

    public void moveTo(int target){
        ArmExtensionSubsystem.target = target;
        holdPosition();
    }

    public void holdPosition() {
        double power = calculate();
        leftExtension.setPower(power);
        //rightExtension.setPower(power);
    }
    public void manual(boolean forward) {
        double power;
        if(forward) {
            power = manualPower;
        }else {
            power = manualPower * -1;
        }
        leftExtension.setPower(power);
        //rightExtension.setPower(p);
    }

    public void resetTarget() {
        target = leftExtension.getCurrentPosition();
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
        pidController.setTolerance(tolerance);
        int current = leftExtension.getCurrentPosition();

        double power = kSpring - pidController.calculate(current, target);
        // we are subtracting the PID since the springs are constantly trying to extend the arm
        power /= voltageSensor.getVoltage();

        telemetry.addData("Extension Power:", power);
        return power;
    }
    public void resetEncoder() {
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}