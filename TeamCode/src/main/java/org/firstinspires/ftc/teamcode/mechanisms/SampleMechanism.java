package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SampleMechanism {

    /*
    avoid making mechanisms and actions at all costs, their purpose collides with subsystems and commands
    we will be aiming their sole use as incorporating roadrunner functionality into ftclib.
     */
    private DcMotorEx motor;

    public SampleMechanism(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "SampleMechanismMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public class SampleMechanismAction implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                motor.setPower(0.8);
                initialized = true;
            }
            double pos = motor.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 3000.0) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                motor.setPower(0);
                return false;
            }
            // overall, the action powers the lift until it surpasses
            // 3000 encoder ticks, then powers it off
        }
    }

    public Action doSampleMechanismAction() {
        return new SampleMechanismAction();
    }

}