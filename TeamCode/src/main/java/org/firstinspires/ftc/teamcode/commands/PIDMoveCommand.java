package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.ArmExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SampleSubsystem;
import org.firstinspires.ftc.teamcode.util.States;

import java.util.Objects;

public class PIDMoveCommand extends CommandBase {
    private ArmPivotSubsystem pivot;
    private ArmExtensionSubsystem extension;
    private States.ArmPivot pivotState;
    private States.ArmExtension extensionState;

    private boolean controlPivot;

    public PIDMoveCommand(ArmPivotSubsystem pivot, States.ArmPivot state) {
        this.pivot = pivot;
        pivotState = state;
        addRequirements(pivot);
    }
    public PIDMoveCommand(ArmExtensionSubsystem extension, States.ArmExtension state){
        this.extension = extension;
        extensionState = state;
        addRequirements(extension);
    }
    @Override
    public void initialize() {
        controlPivot = Objects.nonNull(pivot);
    }

    @Override
    public void execute() {
        if(controlPivot) {
            pivot.setState(pivotState);
        } else {
            extension.setState(extensionState);
        }
    }

    @Override
    public boolean isFinished() {
        if (controlPivot) {
            return pivot.pidController.atSetPoint();
        } else {
            return extension.pidController.atSetPoint();
        }
    }

}
