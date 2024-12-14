package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;

@Config
public class DriveSubsystem extends SubsystemBase {
    public Follower follower;
    private Telemetry telemetry;
    public DriveSubsystem(Follower follower, Pose startPose, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
        follower.setStartingPose(startPose);
    }

    @Override
    public void periodic() {
        follower.update();
        Drawing.drawPoseHistory(follower.getDashboardPoseTracker(), "#4CAF50");
        Drawing.drawRobot(getPose(), "#4CAF50");
        Drawing.sendPacket();

        Pose pose = getPose();
        telemetry.addData("x", pose.getX());
        telemetry.addData("y",pose.getY());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
    }

    public Pose getPose() {
        return follower.getPose();
    }
}
