package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Methods.bucketDrop;
import static org.firstinspires.ftc.teamcode.util.Methods.intake;
import static org.firstinspires.ftc.teamcode.util.Methods.lowBucket;

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
import org.firstinspires.ftc.teamcode.util.States;


@Config
@Autonomous(name="Bucket Auto", group="Auto")
public class Bucket_Auto extends CommandOpMode {

    public static long sampleWait = 3500;

    Pose start = new Pose(9.124016, 104.874016, Math.toRadians(-90));
    Pose bucket = new Pose(18, 126, Math.toRadians(315));
    Pose sample1 = new Pose(27, 132, Math.toRadians(-30));
    Pose sample2 = new Pose(25, 132, Math.toRadians(0));
    Pose sample3 = new Pose(27, 131, Math.toRadians(29));

    DriveSubsystem drive;
    ArmExtensionSubsystem armExt;
    ArmPivotSubsystem armPivot;
    ClawSubsystem claw;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.setConstants(FConstants.class, LConstants.class);
        drive = new DriveSubsystem(new Follower(hardwareMap), start, telemetry);
        armExt = new ArmExtensionSubsystem(hardwareMap, telemetry);
        armExt.setDefaultCommand(new RunCommand(armExt::holdPosition, armExt));

        armPivot = new ArmPivotSubsystem(hardwareMap, telemetry, armExt.leftExtension::getCurrentPosition);
        armPivot.setDefaultCommand(new RunCommand(armPivot::holdPosition, armPivot));
        claw = new ClawSubsystem(hardwareMap, telemetry);
        claw.setDefaultCommand(new RunCommand(claw::holdPosition, claw));

        schedule(new RunCommand(() -> {
            telemetry.update();
        }));



        SequentialCommandGroup routine = new SequentialCommandGroup(
                bucket()/*,
                bucketDrop(claw), // preload
                new ParallelCommandGroup(
                        intake(armExt, armPivot, claw, States.ArmExtension.intake, States.Claw.intake),
                        new ParallelRaceGroup(
                                new PedroPathCommand(drive, toSample(sample1), true),
                                new WaitCommand(sampleWait)
                        )
                ),
                new InstantCommand(() -> claw.handSetPosition(193)),
                new WaitCommand(200),
                new InstantCommand(() -> claw.setFingerState(States.Finger.closed)),
                new WaitCommand(300),
                bucket(),
                bucketDrop(claw), // sample 1
                new ParallelCommandGroup(
                        intake(armExt, armPivot, claw, States.ArmExtension.intake, States.Claw.intake),
                        new ParallelRaceGroup(
                                new PedroPathCommand(drive, toSample(sample2), true),
                                new WaitCommand(sampleWait)
                        )
                ),
                new InstantCommand(() -> claw.setFingerState(States.Finger.closed)),
                new WaitCommand(300),
                bucket(),
                bucketDrop(claw), // sample 2
                new ParallelCommandGroup(
                        intake(armExt, armPivot, claw, States.ArmExtension.intake, States.Claw.intake),
                        new ParallelRaceGroup(
                                new PedroPathCommand(drive, toSample(sample3), true),
                                new WaitCommand(sampleWait)
                        )
                ),
                new InstantCommand(() -> claw.handSetPosition(137)),
                new WaitCommand(200),
                new InstantCommand(() -> claw.setFingerState(States.Finger.closed)),
                new WaitCommand(300),
                bucket(),
                bucketDrop(claw) //sample 3*/
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
    }

    //FIXME we don't need these race groups. you can just set a timeout constraint on the path.
    public static long bucketWait = 3500;
    public ParallelCommandGroup bucket() {
        return new ParallelCommandGroup(
                new ParallelRaceGroup(
                        new PedroPathCommand(drive, toBucket(start), true),
                        new WaitCommand(bucketWait)
                ),
                lowBucket(armExt, armPivot, claw)
        );
    }


    public PathChain toBucket(Pose start){
        return drive.follower.pathBuilder()
                .addPath(new BezierLine(new Point(start), new Point(bucket)))
                .setLinearHeadingInterpolation(start.getHeading(), bucket.getHeading())
                .build();
    }
    public PathChain toSample(Pose end) {
        return drive.follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucket), new Point(end)))
                .setLinearHeadingInterpolation(bucket.getHeading(), end.getHeading())
                .build();
    }
}
