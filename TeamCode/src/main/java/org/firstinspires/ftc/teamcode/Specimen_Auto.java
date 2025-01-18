package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Methods.intake;
import static org.firstinspires.ftc.teamcode.util.Methods.returnHome;
import static org.firstinspires.ftc.teamcode.util.Methods.specimen;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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

    Pose start = new Pose(9.124016, 56.874016, Math.toRadians(90));
    Pose preload = new Pose(39, 70, Math.toRadians(0));
    Point preloadControl = new Point(7.4, 75.8);

    // fix me
    Pose specimenEnd = new Pose(36, 64, Math.toRadians(180));
    Pose bucket = new Pose(18, 126, Math.toRadians(315));

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


        PathChain preloadPath = drive.follower.pathBuilder()
                        .addBezierCurve(new Point(start), preloadControl, new Point(preload))
                        .setLinearHeadingInterpolation(start.getHeading(), preload.getHeading())
                        .build();


        schedule(new RunCommand(() -> {
            telemetry.update();
        }));


        SequentialCommandGroup routine = new SequentialCommandGroup(
            specimen(armExt, armPivot, claw),
            new PedroPathCommand(drive, preloadPath,true),
            new InstantCommand(() -> claw.wristSetPosition(ClawSubsystem.pSpecimen2)),
            new WaitCommand(400),
            new PedroPathCommand(drive, goBack(preload), true),
            new WaitCommand(400),
            new InstantCommand(() -> claw.setFingerState(States.Finger.opened))
        );
        schedule(routine);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        // retract to a position
        //claw.setClawState(States.Claw.start);
        while (!isStarted()) {
            armExt.holdPosition();
            armPivot.holdPosition();
            claw.holdPosition();
        }
        waitForStart();
        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
        PoseTransfer.pose = drive.getPose();
    }

    PathChain goBack(Pose start) {
        return drive.follower.pathBuilder()
                .addBezierLine(new Point(start), new Point(start.getX() - 3.5, start.getY()))
                .setLinearHeadingInterpolation(start.getHeading(), start.getHeading())
                .build();
    }


}
