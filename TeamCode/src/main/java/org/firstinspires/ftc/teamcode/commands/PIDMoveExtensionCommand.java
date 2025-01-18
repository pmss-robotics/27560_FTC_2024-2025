package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmPivotSubsystem;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.Objects;

public class PIDMoveExtensionCommand extends CommandBase {
    private ArmExtensionSubsystem extension;

    private int position;

    public PIDMoveExtensionCommand(ArmExtensionSubsystem extension, int position){
        this.extension = extension;
        this.position = position;
    }
    @Override
    public void initialize() {
            extension.moveTo(position);
    }

    @Override
    public boolean isFinished() {

        return extension.pidController.atSetPoint();
    }

}
