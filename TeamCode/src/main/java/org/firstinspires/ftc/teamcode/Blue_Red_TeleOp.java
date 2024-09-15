package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.SampleMechanism;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeedforwardArmSubsystem;

import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

@Config
@TeleOp(name = "SAMPLE", group = "TeleOp")
public class Blue_Red_TeleOp extends CommandOpMode {
    @Override
    public void initialize() {
        // data sent to telemetry shows up on dashboard and driver station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        // GamepadEx wraps gamepad 1 or 2 for easier implementations of more complex key bindings
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        // The driveSubsystem wraps Roadrunner's MecanumDrive to combine with Commands.
        DriveSubsystem drive = new DriveSubsystem(new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)), telemetry);
        // The driveCommand uses methods defined in the DriveSubsystem to create behaviour.
        // we're passing in methods to get values instead of straight values because it avoids
        // disturbing the structure of the CommandOpMode. The aim is to define bindings in this
        // initialize() method through Commands and these will be looped and acted in the (hidden)
        // run() loop. TODO we (me at least) should learn more about lambda expressions / functions as parameters.
        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -gamepadEx.getLeftX(),
                () -> -gamepadEx.getLeftY(),
                () -> -gamepadEx.getRightX(),
                true);

        FeedforwardArmSubsystem arm = new FeedforwardArmSubsystem(hardwareMap, telemetry);
        ArmCommand armCommand = new ArmCommand(arm,
                () -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        arm.setDefaultCommand(armCommand);

        // sample for action and command synergy and binding
        // try to avoid this kind of usage as much as possible
        SampleMechanism sampleMechanism = new SampleMechanism(hardwareMap);
        Set<Subsystem> subsystemSet = Stream.of(drive).collect(Collectors.toSet());
        // the binding for whenPressed() is convenient since it only activates once even when A is held down.
        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ActionCommand(sampleMechanism.doSampleMechanismAction(), subsystemSet));

        //TODO: see if this runs perpetually
        // also we might not want to be creating a new packet in each loop
        schedule(new RunCommand(() -> {
            TelemetryPacket packet = new TelemetryPacket();
            Pose2d pose = drive.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y",pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();

            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }));
        schedule(driveCommand);

    }


}
