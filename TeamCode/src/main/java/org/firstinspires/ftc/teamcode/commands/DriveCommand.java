package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.PIDFController;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * The default command of {@link DriveSubsystem}
 */
@Config
public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier lx, ly, rx;
    private final boolean isFieldCentric;
    public static double target; // radians
    PIDFController headingPIDF;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx, boolean isFieldCentric) {
        this.drive = drive;
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.isFieldCentric = isFieldCentric;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        target = drive.follower.getPose().getHeading();
        drive.follower.startTeleopDrive();
        headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients);
    }
    @Override
    public void execute() {
        headingPIDF.setCoefficients(FollowerConstants.headingPIDFCoefficients); //FIXME: remove me when done tuning
        Pose currentPose = drive.follower.getPose();
        if(rx.getAsDouble()!= 0) {
            target = drive.follower.getPose().getHeading();
            drive.follower.setTeleOpMovementVectors(ly.getAsDouble(), lx.getAsDouble(), rx.getAsDouble(), !isFieldCentric);
        } else {
            // possibly deadband?
            double headingError = MathFunctions.getTurnDirection(currentPose.getHeading(), target) * MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), target);
            headingPIDF.updateError(headingError);
            double heading = MathFunctions.clamp(headingPIDF.runPIDF() + FollowerConstants.headingPIDFFeedForward * MathFunctions.getTurnDirection(currentPose.getHeading(), target), -1, 1);
            drive.follower.setTeleOpMovementVectors(ly.getAsDouble(), lx.getAsDouble(), heading, !isFieldCentric);
        }

    }
}
