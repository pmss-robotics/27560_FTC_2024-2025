package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.commands.PIDMoveCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class Methods {
    public static PathChain linePath(DriveSubsystem drive, Pose start, Pose end) {
        return drive.follower.pathBuilder()
                .addBezierLine(new Point(start), new Point(end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    public static SequentialCommandGroup lowBucket(ArmExtensionSubsystem ext, ArmPivotSubsystem pivot, ClawSubsystem claw) {
        return new SequentialCommandGroup(
                new PIDMoveCommand(ext, States.ArmExtension.home),
                new InstantCommand(() -> claw.setClawState(States.Claw.home)),
                new InstantCommand(() -> claw.setFingerState(States.Finger.closed)),
                new PIDMoveCommand(pivot, States.ArmPivot.bucket),
                new PIDMoveCommand(ext, States.ArmExtension.bucket),
                new InstantCommand(() -> claw.setClawState(States.Claw.bucket))
        );
    }
    public static SequentialCommandGroup specimen(ArmExtensionSubsystem ext, ArmPivotSubsystem pivot, ClawSubsystem claw) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> claw.setClawState(States.Claw.specimen)),
                new InstantCommand(() -> claw.setFingerState(States.Finger.closed)),
                new PIDMoveCommand(pivot, States.ArmPivot.specimen),
                new PIDMoveCommand(ext, States.ArmExtension.specimen_1)
        );
    }

    public static long dropTimeout = 500;
    public static SequentialCommandGroup bucketDrop(ClawSubsystem claw) {
        return new SequentialCommandGroup(
                new WaitCommand(dropTimeout),
                new InstantCommand(() -> claw.setFingerState(States.Finger.opened)),
                new WaitCommand(dropTimeout),
                new InstantCommand(() -> claw.setClawState(States.Claw.home))
        );
    }

    public static SequentialCommandGroup specimenIntake(ArmExtensionSubsystem ext, ArmPivotSubsystem pivot, ClawSubsystem claw) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> claw.setClawState(States.Claw.specimenIntake)),
                new InstantCommand(() -> claw.setFingerState(States.Finger.opened)),
                new PIDMoveCommand(pivot, States.ArmPivot.specimenIntake),
                new PIDMoveCommand(ext, States.ArmExtension.home)
        );
    }

    public static SequentialCommandGroup intake(ArmExtensionSubsystem ext, ArmPivotSubsystem pivot, ClawSubsystem claw, States.ArmExtension position) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> claw.setClawState(States.Claw.intake)),
                new PIDMoveCommand(pivot, States.ArmPivot.intake),
                new PIDMoveCommand(ext, position),
                new InstantCommand(() -> claw.setClawState(States.Claw.home)),
                new InstantCommand(() -> claw.setFingerState(States.Finger.opened))
        );
    }

    public static SequentialCommandGroup returnHome(ArmExtensionSubsystem armExt, ArmPivotSubsystem armPivot, ClawSubsystem claw) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> claw.setFingerState(States.Finger.closed)),
                new PIDMoveCommand(armExt, States.ArmExtension.home),
                new PIDMoveCommand(armPivot, States.ArmPivot.home),
                new InstantCommand(() -> claw.setClawState(States.Claw.intake))
        );
    }

}
