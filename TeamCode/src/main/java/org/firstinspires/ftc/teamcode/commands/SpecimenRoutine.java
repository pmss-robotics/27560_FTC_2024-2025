package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.util.States;

public class SpecimenRoutine extends SequentialCommandGroup {
    public SpecimenRoutine(ClawSubsystem claw, ArmExtensionSubsystem extension) {
        addCommands(
                new PIDMoveCommand(extension, States.ArmExtension.specimen_2),
                new InstantCommand(() -> claw.setFingerState(States.Finger.opened), claw),
                new InstantCommand(() -> claw.setClawState(States.Claw.home), claw)
        );
        addRequirements(claw, extension);
    }
}