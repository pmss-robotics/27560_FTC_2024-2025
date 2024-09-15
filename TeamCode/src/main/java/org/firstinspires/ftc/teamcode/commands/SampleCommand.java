package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.SampleSubsystem;

public class SampleCommand extends CommandBase {
    private final SampleSubsystem sampleSubsystem;
    SampleCommand(SampleSubsystem sampleSubsystem) {
        this.sampleSubsystem = sampleSubsystem;
        // instantiate other parameters if there are
        addRequirements(sampleSubsystem);
    }
    @Override
    public void initialize() {
        // runs once when the command is scheduled
    }

    @Override
    public void execute() {
        // runs repeatedly until the command is finished or interrupted
        sampleSubsystem.doThing();
    }

    @Override
    public boolean isFinished() {
        // runs right after execute(),  return true to end the command.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // runs when the command is ended, use the interrupted argument for more precise control flow
    }

}
