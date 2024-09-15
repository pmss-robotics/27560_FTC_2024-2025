package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// tuning guide: https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
@Config
public class PIDFmotorSubsystem extends SubsystemBase {

    private MotorEx motor;
    private Telemetry telemetry;
    public double kP = 1, kI = 0, kD = 0, kF = 0;
    public PIDFController controller = new PIDFController(kP, kI, kD, kF);
    public int target;

    public PIDFmotorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = new MotorEx(hardwareMap, "MotorName");
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
        target = motor.getCurrentPosition();
    }
    @Override
    public void periodic() {
        telemetry.addData("Motor Target: ", target);
        telemetry.addData("Motor Pos: ", motor.getCurrentPosition());
        telemetry.update();
    }

    public void holdPosition() {
        int current = motor.getCurrentPosition();
        motor.setVelocity(controller.calculate(current, target));
    }

    public void move(double power) {
        motor.set(power);
        target = motor.getCurrentPosition();
    }
}
