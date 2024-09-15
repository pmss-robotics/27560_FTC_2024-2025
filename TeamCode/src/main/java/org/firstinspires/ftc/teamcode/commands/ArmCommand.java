package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FeedforwardArmSubsystem;

import java.util.function.DoubleSupplier;

public class ArmCommand extends CommandBase {
    private final FeedforwardArmSubsystem feedforwardArmSubsystem;
    private final DoubleSupplier lt, rt;
    public ArmCommand(FeedforwardArmSubsystem feedforwardArmSubsystem, DoubleSupplier lt, DoubleSupplier rt) {
        this.feedforwardArmSubsystem = feedforwardArmSubsystem;
        this.lt = lt;
        this.rt = rt;
        addRequirements(feedforwardArmSubsystem);
    }
    @Override
    public void execute() {
        if (rt.getAsDouble() == 0 && lt.getAsDouble() == 0) {
            feedforwardArmSubsystem.holdPosition();
        } else {
            feedforwardArmSubsystem.move(rt.getAsDouble() - lt.getAsDouble());
        }
    }

}
