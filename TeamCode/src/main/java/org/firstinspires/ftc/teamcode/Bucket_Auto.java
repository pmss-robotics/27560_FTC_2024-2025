package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.PedroPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;



@Config
@Autonomous(name="Bucket Auto", group="Auto")
public class Bucket_Auto extends CommandOpMode {
    Pose start = new Pose(9.124016, 104.874016, Math.toRadians(-90));
    Pose bucket = new Pose(24, 120, Math.toRadians(315));
    Pose sample1 = new Pose(24, 120, Math.toRadians(0));
    Pose sample2 = new Pose(24, 120, Math.toRadians(30));
    Pose sample3 = new Pose(24, 120, Math.toRadians(45));

    DriveSubsystem drive;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new DriveSubsystem(new Follower(hardwareMap), start, telemetry);
        GamepadEx tools = new GamepadEx(gamepad2);
        schedule(new RunCommand(() -> {
            telemetry.update();
        }));

        schedule(new PedroPathCommand(drive, toBucket(start)));

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
