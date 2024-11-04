package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.States;

public class AutoAlignRoutine extends SequentialCommandGroup {
    public AutoAlignRoutine(VisionSubsystem vision, ClawSubsystem claw, DriveSubsystem drive, ArmExtensionSubsystem extension) {
        addCommands(
                new InstantCommand(() -> vision.enableDetection(true)),

                // align / drive / extend

                new InstantCommand(() -> vision.enableDetection(false))

        );
        addRequirements(claw, vision, drive, extension);
    }
}