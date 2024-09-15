package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.lang.reflect.Method;
import java.util.function.DoubleSupplier;

/**
 * The default command of {@link DriveSubsystem}
 */
public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier lx, ly, rx;
    private final boolean isFieldCentric;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx, boolean isFieldCentric) {
        this.drive = drive;
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.isFieldCentric = isFieldCentric;
        addRequirements(drive);
    }
    @Override
    public void execute() {
        if(isFieldCentric) drive.fieldCentric(lx.getAsDouble(), ly.getAsDouble(), rx.getAsDouble());
        else drive.robotCentric(lx.getAsDouble(), ly.getAsDouble(), rx.getAsDouble());
    }
}
