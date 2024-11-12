package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoAlignRoutine;
import org.firstinspires.ftc.teamcode.commands.BucketRoutine;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.PIDMoveCommand;
import org.firstinspires.ftc.teamcode.commands.SpecimenRoutine;
import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.drive.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.States;


//hello
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "EncoderTest", group = "TeleOp")
public class EncoderTest extends CommandOpMode {

    GamepadEx driver, tools;

    ArmPivotSubsystem armPivot;
    ArmExtensionSubsystem armExt;

    public static int bucketDropTimeout;

    @Override
    public void initialize() {
        // data sent to telemetry shows up on dashboard and driverGamepad station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        // GamepadEx wraps gamepad 1 or 2 for easier implementations of more complex key bindings
        // The driveSubsystem wraps Roadrunner's MecanumDrive to combine with Commands.

        armExt = new ArmExtensionSubsystem(hardwareMap, telemetry);
        armExt.setDefaultCommand(new RunCommand(armExt::holdPosition, armExt));


        armPivot = new ArmPivotSubsystem(hardwareMap, telemetry, armExt.leftExtension::getCurrentPosition);
        armPivot.setDefaultCommand(new RunCommand(armPivot::holdPosition, armPivot));
        armPivot.resetEncoder();




        schedule(new RunCommand(() -> {
            telemetry.update();
        }));
    }




}
