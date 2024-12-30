package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

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
    public PedroPathCommand(DriveSubsystem drive, Path path) {
        this(drive, new PathChain(path), false);
    }
    public PedroPathCommand(DriveSubsystem drive, PathChain path) {
        this(drive, path, false);
    }

    @Override
    public void initialize(){
        drive.follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !drive.follower.isBusy();
    }

}

