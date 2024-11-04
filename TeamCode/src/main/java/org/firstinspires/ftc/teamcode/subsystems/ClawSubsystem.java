package org.firstinspires.ftc.teamcode.subsystems;

import android.bluetooth.le.ScanSettings;

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
import org.firstinspires.ftc.teamcode.util.States;

import java.util.Objects;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
@Config
public class ClawSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;
    ServoImplEx finger, hand, wrist; // in order of precedence
    // wrist moves hand and finger along an axis, wrist just moves fingers, etc.

    public static double F_target = 0, H_target = 0, W_target = 0; // in degrees

    private States.Claw currentClawState;
    public static int wpHome, wpBucket = 0, wpSpecimen = 0, wpIntake = 0, wpStart = 0; // in degrees
    public static int hpHome, hpBucket = 0, hpSpecimen = 0, hpIntake = 0, hpStart = 0; // in degrees

    private States.Finger currentFingerState;
    public static int pClosed = 0, pOpen = 0; // in degrees

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

        currentClawState = States.Claw.home;
        currentFingerState = States.Finger.closed;
    }

    public States.Claw getCurrentClawState() {
        return currentClawState;
    }

    public boolean isOpen() {
        return currentFingerState == States.Finger.opened;
    }

    public void toggleFingerState() {
        switch (currentFingerState) {
            case opened:
                finger.setPosition(pClosed);
                currentFingerState = States.Finger.closed;
                break;
            case closed:
                finger.setPosition(pOpen);
                currentFingerState = States.Finger.opened;
                break;
        }
    }

    public void setFingerState(States.Finger state) {
        currentFingerState = state;
        switch (currentFingerState) {
            case opened:
                finger.setPosition(pClosed);
                break;
            case closed:
                finger.setPosition(pOpen);
                break;
        }
    }

    public void setClawState(States.Claw state) {
        currentClawState = state;
        switch (currentClawState) {
            case home:
                hand.setPosition(hpHome);
                wrist.setPosition(wpHome);
                break;
            case intake:
                hand.setPosition(hpIntake);
                wrist.setPosition(wpIntake);
                break;
            case specimen:
                hand.setPosition(hpSpecimen);
                wrist.setPosition(wpSpecimen);
                break;
            case bucket:
                hand.setPosition(hpBucket);
                wrist.setPosition(wpBucket);
                break;
            case start:

        }
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
