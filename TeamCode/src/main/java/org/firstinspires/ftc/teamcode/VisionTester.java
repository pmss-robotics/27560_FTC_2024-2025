package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class VisionTester extends CommandOpMode {
    VisionSubsystem vision;
    GamepadEx tools;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        tools = new GamepadEx(gamepad2);

        try {
            vision = new VisionSubsystem(hardwareMap, telemetry);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        new GamepadButton(tools, GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> {
                    vision.getSampleAngle();
                    telemetry.update();})
        );
    }
}
