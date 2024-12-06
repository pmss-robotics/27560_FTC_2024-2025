package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Config
public class DriveSubsystem extends SubsystemBase {
    public final Follower follower;
    private Telemetry telemetry;
    public DriveSubsystem(Follower follower, Pose startPose, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
        follower.setStartingPose(startPose);
    }

    public Pose getPose() {
        return follower.getPose();
    }
}
