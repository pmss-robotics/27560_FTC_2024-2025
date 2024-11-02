package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
@Config
public class ClawSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;
    ServoImplEx finger, hand, wrist; // in order of precedence
    // wrist moves hand and finger along an axis, wrist just moves fingers, etc.

    public static double F_target = 0, H_target = 0, W_target = 0; // in degrees

    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // initialize hardware here alongside other parameters
        this.telemetry = telemetry;
        finger = hardwareMap.get(ServoImplEx.class, "finger");
        hand = hardwareMap.get(ServoImplEx.class, "hand");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");

        // expand the range of the servo beyond the default for control/expansion hubs
        // test
        finger.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hand.setPwmRange(new PwmControl.PwmRange(500, 2500));
        wrist.setPwmRange(new PwmControl.PwmRange(500, 2500));

        finger.setPosition(scale(F_target));
        hand.setPosition(scale(H_target));
        wrist.setPosition(scale(W_target));
    }

    @Override
    public void periodic() {
        telemetry.addData("finger position", finger.getPosition());
        telemetry.addData("hand position", hand.getPosition());
        telemetry.addData("wrist position", wrist.getPosition());
    }

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }

}
