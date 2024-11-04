package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.util.States;

public class BucketRoutine extends SequentialCommandGroup {
    public static long dropTimeout = 0;
    public BucketRoutine(ClawSubsystem claw) {
        addCommands(
                new InstantCommand(() -> claw.setFingerState(States.Finger.opened), claw),
                new WaitCommand(dropTimeout),
                new InstantCommand(() -> claw.setClawState(States.Claw.home), claw)
        );
        addRequirements(claw);
    }
}