package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Methods.bucketDrop;
import static org.firstinspires.ftc.teamcode.util.Methods.intake;
import static org.firstinspires.ftc.teamcode.util.Methods.lowBucket;
import static org.firstinspires.ftc.teamcode.util.Methods.returnHome;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.PedroPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PoseTransfer;
import org.firstinspires.ftc.teamcode.util.States;


@Config
@Autonomous(name="Specimen Auto", group="Auto")
public class Specimen_Auto extends CommandOpMode {

    Pose start = new Pose(9.124016, 56.874016, Math.toRadians(180));
    // fix me
    Pose specimenEnd = new Pose(36, 64, Math.toRadians(180));
    Pose sample1 = new Pose(27, 12, Math.toRadians(-30));
    Pose sample2 = new Pose(25, 12, Math.toRadians(0));
    Pose sample3 = new Pose(27, 11, Math.toRadians(29));
    double sampleDeposit1 = Math.toRadians(90);
    double sampleDeposit2 = Math.toRadians(180);

    DriveSubsystem drive;
    ArmExtensionSubsystem armExt;
    ArmPivotSubsystem armPivot;
    ClawSubsystem claw;

    @Override
    public void initialize() {
        PoseTransfer.pose = start;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.setConstants(FConstants.class, LConstants.class);
        drive = new DriveSubsystem(new Follower(hardwareMap), start, telemetry);
        armExt = new ArmExtensionSubsystem(hardwareMap, telemetry);
        armExt.setDefaultCommand(new RunCommand(armExt::holdPosition, armExt));

        armPivot = new ArmPivotSubsystem(hardwareMap, telemetry, armExt.leftExtension::getCurrentPosition);
        armPivot.setDefaultCommand(new RunCommand(armPivot::holdPosition, armPivot));
        claw = new ClawSubsystem(hardwareMap, telemetry);
        claw.setDefaultCommand(new RunCommand(claw::holdPosition, claw));

        PathChain preload = drive.follower.pathBuilder()
                        .addBezierLine(new Point(start), new Point(specimenEnd))
                        .setLinearHeadingInterpolation(start.getHeading(), specimenEnd.getHeading())
                        .build();


        schedule(new RunCommand(() -> {
            telemetry.update();
        }));


        SequentialCommandGroup routine = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        returnHome(armExt, armPivot, claw),
                        new PedroPathCommand(drive, preload)
                )
        );
        schedule(routine);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        // retract to a position
        while (!isStarted()) {
            armExt.holdPosition();
            armPivot.holdPosition();
        }
        waitForStart();
        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
        PoseTransfer.pose = drive.getPose();
    }


}
