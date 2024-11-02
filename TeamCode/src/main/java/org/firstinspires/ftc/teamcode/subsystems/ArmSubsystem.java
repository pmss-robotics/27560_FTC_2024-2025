package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.util.MathUtils.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmSubsystem extends SubsystemBase {
    Telemetry telemetry;
    MotorGroup pivots;

    public static double kP = 1, kI = 0, kD = 0;
    public static double kS = 0, kV = 0, kA = 0;
    public double target = 0;

    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        MotorEx leftPivot = new MotorEx(hardwareMap, "leftPivot", Motor.GoBILDA.RPM_435);
        MotorEx rightPivot = new MotorEx(hardwareMap, "rightPivot");
        rightPivot.setInverted(true);
        pivots = new MotorGroup(leftPivot, rightPivot);
    }
    @Override
    public void periodic() {
        telemetry.addData("Pivot Target: ", target);
        telemetry.addData("Pivot Pos: ", pivots.getCurrentPosition());
    }

    public void moveTo(double target){

    }

    public void holdPosition() {

    }
    public void manual(double power) {

    }

    private double clampPower(double power) {
        return MathUtils.clamp(power, -1, 1);
    }
}
