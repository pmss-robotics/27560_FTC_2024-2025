package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.PIDMoveCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class Methods {
    public static SequentialCommandGroup lowBucket(ArmExtensionSubsystem ext, ArmPivotSubsystem pivot, ClawSubsystem claw) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> claw.setClawState(States.Claw.home), claw),
                new InstantCommand(() -> claw.setFingerState(States.Finger.closed), claw),
                new PIDMoveCommand(pivot, States.ArmPivot.bucket),
                new PIDMoveCommand(ext, States.ArmExtension.bucket),
                new InstantCommand(() -> claw.setClawState(States.Claw.bucket), claw)
        );
    }
    public static SequentialCommandGroup specimen(ArmExtensionSubsystem ext, ArmPivotSubsystem pivot, ClawSubsystem claw) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> claw.setClawState(States.Claw.home), claw),
                new InstantCommand(() -> claw.setFingerState(States.Finger.closed), claw),
                new PIDMoveCommand(pivot, States.ArmPivot.specimen),
                new PIDMoveCommand(ext, States.ArmExtension.specimen_1)
        );
    }

}
