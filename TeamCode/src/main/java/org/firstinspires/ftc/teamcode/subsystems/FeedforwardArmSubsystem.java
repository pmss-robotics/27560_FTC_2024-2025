package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// tuning guide: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#introduction-to-dc-motor-feedforward
@Config
public class FeedforwardArmSubsystem extends SubsystemBase {

    private MotorEx arm;
    private Telemetry telemetry;
    // to find kS, it is the amount of voltage before the arm starts moving.
    // I Have no freaking clue how to tune kCos and kV
    // ignore kA if the component has not much inertia
    public double kS = 0, kCos = 0, kV = 0, kA = 0;
    // to find the number of ticks in one rotation on a motor
    // multiply its internal gear ratio by the number of pulses per rev. at the encoder shaft
    // OR whatever the pulses per rev. is at the gearbox output shaft which is the same thing.
    public double ticksInRadians = 0 / (2 * Math.PI);
    ArmFeedforward feedforward = new ArmFeedforward(kS, kCos, kV, kA);
    public int target;
    public double targetArmVel = 2;

    public FeedforwardArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        arm = new MotorEx(hardwareMap, "ArmName");
        arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
        target = arm.getCurrentPosition();
    }
    @Override
    public void periodic() {
        telemetry.addData("Arm Target: ", target);
        telemetry.addData("Arm Pos: ", arm.getCurrentPosition());
        telemetry.update();
    }
    // this is probably so wrong
    public void holdPosition() {
        double power = feedforward.calculate(target / ticksInRadians, targetArmVel);
        arm.set(power);
    }
    
    public void move(double power) {
        arm.set(power);
        target = arm.getCurrentPosition();
    }
}
