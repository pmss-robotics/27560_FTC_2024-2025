package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class GenericPositionServoSubsystem extends SubsystemBase {
    Telemetry telemetry;
    Servo servo;
    String servoName;
    public double position;

    public GenericPositionServoSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String servoName, double startPos) {
        // initialize hardware here alongside other parameters
        this.servoName = servoName;
        this.servo = hardwareMap.get(Servo.class, servoName);
        this.telemetry = telemetry;
        this.position = startPos;
    }
    @Override
    public void periodic() {
        telemetry.addData(servoName+": " , servo.getPosition());
        telemetry.update();
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }
    public void incrementPosition(double increment) {

        position = MathUtils.clamp(position + increment, 0, 1);
        servo.setPosition(position);
    }


}
