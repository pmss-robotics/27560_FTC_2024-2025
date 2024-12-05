package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Collections;
import java.util.Set;

public class PedroPathCommand extends CommandBase {

    DriveSubsystem drive;
    PathChain path;
    boolean holdEnd;

    public PedroPathCommand(DriveSubsystem drive, PathChain path, boolean holdEnd) {
        this.drive = drive;
        this.path = path;
        this.holdEnd = holdEnd;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.follower.update();
        drive.follower.followPath(path, holdEnd); // FIXME this might be unnecessary
    }

    @Override
    public boolean isFinished() {
        return !drive.follower.isBusy();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}

