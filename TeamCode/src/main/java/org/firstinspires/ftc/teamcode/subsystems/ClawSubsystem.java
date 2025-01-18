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

import java.lang.annotation.Target;
import java.util.Objects;

// https://docs.ftclib.org/ftclib/command-base/command-system/subsystems
@Config
public class ClawSubsystem extends SubsystemBase {

    // declare hardware here
    Telemetry telemetry;
    public ServoImplEx finger, hand, wrist; // in order of precedence
    // wrist moves hand and finger along an axis, wrist just moves fingers, etc.

    public static double F_target = 0, H_target = 0, W_target = 0; // in degrees

    private States.Claw currentClawState;
    public static double wpHome = 50, wpBucket = 130, wpSpecimen = 110, wpSpecimenIntake = 100, wpIntake = 110, wpStart =  25, pSpecimen2 = 60;// in degrees
    // 100
    public static double hpHome = 165, hpBucket = 165, hpSpecimen = 165, hpIntake = 165, hpStart = 165; // in degrees

    private States.Finger currentFingerState;
    public static double pClosed = 65, pOpen = 155; // in degrees
    public static double offset = 0;

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

        F_target = pClosed;
        H_target = hpStart;
        W_target = wpStart;

        finger.setPosition(scale(F_target));
        hand.setPosition(scale(H_target));
        wrist.setPosition(scale(W_target));

        currentClawState = States.Claw.home;
        currentFingerState = States.Finger.closed;
        offset = 0;
    }

    public static double getF_target() {
        return F_target;
    }

    public static double getW_target() {
        return W_target + offset;
    }

    public static double getH_target() {
        return H_target;
    }

    public void holdPosition() {

        finger.setPosition(scale(F_target));
        hand.setPosition(scale(H_target));
        wrist.setPosition(scale(getW_target()));
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
                F_target = pClosed;
                currentFingerState = States.Finger.closed;
                break;
            case closed:
                F_target = pOpen;
                currentFingerState = States.Finger.opened;
                break;
        }
        fingerSetPosition(getF_target());
    }

    public void setFingerState(States.Finger state) {
        currentFingerState = state;
        switch (currentFingerState) {
            case opened:
                F_target = pOpen;
                break;
            case closed:
                F_target = pClosed;
                break;
        }
        fingerSetPosition(getF_target());
    }

    public void setClawState(States.Claw state) {
        currentClawState = state;
        switch (currentClawState) {
            case home:
                H_target = hpHome;
                W_target = wpHome;
                break;
            case intake:
                H_target = hpIntake;
                W_target = wpIntake;
                break;
            case specimen:
                H_target = hpSpecimen;
                W_target = wpSpecimen;
                break;
            case specimenIntake:
                H_target = hpSpecimen;
                W_target = wpSpecimenIntake;
                break;
            case bucket:
                H_target = hpBucket;
                W_target = wpBucket;
                break;
            case start:
        }
        handSetPosition(getH_target());
        wristSetPosition(getW_target());
    }

    @Override
    public void periodic() {
        telemetry.addData("finger position", finger.getPosition());
        telemetry.addData("hand position", hand.getPosition());
        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("offset", offset);
    }

    private double scale(double angle){
        // angle in degrees
        return Range.scale(angle, 0, 300, 0, 1);
    }

    public void wristSetPosition(double target) {
        W_target = target;
        wrist.setPosition(scale(getW_target()));
    }
    public void handSetPosition(double target) {
        H_target = target;
        hand.setPosition(scale(getH_target()));
    }
    public void fingerSetPosition(double target) {
        F_target = target;
        finger.setPosition(scale(getF_target()));
    }



}
