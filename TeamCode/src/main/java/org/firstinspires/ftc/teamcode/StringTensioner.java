package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmPivotSubsystem;


//hello
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "String Tensioner", group = "TeleOp")
public class StringTensioner extends CommandOpMode {

    GamepadEx gamepad;

    ArmExtensionSubsystem armExt;
    
    @Override
    public void initialize() {
        // data sent to telemetry shows up on dashboard and driverGamepad station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        gamepad = new GamepadEx(gamepad1);
        // GamepadEx wraps gamepad 1 or 2 for easier implementations of more complex key bindings
        // The driveSubsystem wraps Roadrunner's MecanumDrive to combine with Commands.

        armExt = new ArmExtensionSubsystem(hardwareMap, telemetry);
        //armExt.setDefaultCommand(new RunCommand(armExt::holdPosition, armExt));

        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whileHeld(new InstantCommand(() -> armExt.leftExtension.setPower(0.5), armExt))
                .whenReleased(new InstantCommand(() -> armExt.leftExtension.setPower(0), armExt));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(new InstantCommand(() -> armExt.leftExtension.setPower(-0.5), armExt))
                .whenReleased(new InstantCommand(() -> armExt.leftExtension.setPower(0), armExt));


        schedule(new RunCommand(() -> {
            telemetry.update();
        }));
    }




}
