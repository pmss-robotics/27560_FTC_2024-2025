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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP", group = "TeleOp")
public class TeleOp extends CommandOpMode {
    States.Global currentState = States.Global.home;

    GamepadEx driver, tools;
    DriveSubsystem drive;
    ArmExtensionSubsystem armExt;
    ArmPivotSubsystem armPivot;
    ClawSubsystem claw;
    VisionSubsystem vision;
    public static double tempRot1 = 0;
    public static double tempRot2 = 150;


    @Override
    public void initialize() {
        // data sent to telemetry shows up on dashboard and driverGamepad station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        // GamepadEx wraps gamepad 1 or 2 for easier implementations of more complex key bindings
        driver = new GamepadEx(gamepad1);
        tools = new GamepadEx(gamepad2);
        // The driveSubsystem wraps Roadrunner's MecanumDrive to combine with Commands.
        drive = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0)), telemetry);
        // The driveCommand uses methods defined in the DriveSubsystem to create behaviour.
        // we're passing in methods to get values instead of straight values because it avoids
        // disturbing the structure of the CommandOpMode. The aim is to define bindings in this
        // initialize() method through Commands and these will be looped and acted in the (hidden)
        // run() loop.

        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> -driver.getRightX(),
                true);

        armExt = new ArmExtensionSubsystem(hardwareMap, telemetry);
        armExt.setDefaultCommand(new RunCommand(armExt::holdPosition, armExt));

        armPivot = new ArmPivotSubsystem(hardwareMap, telemetry, armExt.leftExtension::getCurrentPosition);
        armPivot.setDefaultCommand(new RunCommand(armPivot::holdPosition, armPivot));

        claw = new ClawSubsystem(hardwareMap, telemetry);
        claw.setDefaultCommand(new RunCommand(claw::holdPosition, claw));
        /*
        try {
            vision = new VisionSubsystem(hardwareMap, telemetry);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

         */


        // far intake
        new GamepadButton(tools, GamepadKeys.Button.B).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(() -> claw.setClawState(States.Claw.home), claw),
                        new InstantCommand(() -> claw.setFingerState(States.Finger.opened), claw),
                        new PIDMoveCommand(armPivot, States.ArmPivot.intake),
                        new PIDMoveCommand(armExt, States.ArmExtension.intake),
                        new InstantCommand(() -> claw.setClawState(States.Claw.intake), claw),
                        swapState(States.Global.intake_far)
                ),
                returnHome(),
                () -> currentState != States.Global.intake_far
        ));
        // near intake
        new GamepadButton(tools, GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(() -> claw.setClawState(States.Claw.home), claw),
                        new InstantCommand(() -> claw.setFingerState(States.Finger.opened), claw),
                        new PIDMoveCommand(armPivot, States.ArmPivot.intake),
                        new PIDMoveCommand(armExt, States.ArmExtension.home),
                        new InstantCommand(() -> claw.setClawState(States.Claw.intake), claw),
                        swapState(States.Global.intake_near)
                ),
                returnHome(),
                () -> currentState != States.Global.intake_near
        ));
        // bucket
        new GamepadButton(tools, GamepadKeys.Button.X).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(() -> claw.setClawState(States.Claw.home), claw),
                        new InstantCommand(() -> claw.setFingerState(States.Finger.closed), claw),
                        new PIDMoveCommand(armPivot, States.ArmPivot.bucket),
                        new PIDMoveCommand(armExt, States.ArmExtension.bucket),
                        new InstantCommand(() -> claw.setClawState(States.Claw.bucket), claw),
                        swapState(States.Global.bucket)
                ),
                returnHome(),
                () -> currentState != States.Global.bucket
        ));
        // specimen
        new GamepadButton(tools, GamepadKeys.Button.Y).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(() -> claw.setClawState(States.Claw.home), claw),
                        new InstantCommand(() -> claw.setFingerState(States.Finger.closed), claw),
                        new PIDMoveCommand(armPivot, States.ArmPivot.specimen),
                        new PIDMoveCommand(armExt, States.ArmExtension.specimen_1),
                        swapState(States.Global.specimen)
                ),
                returnHome(),
                () -> currentState != States.Global.specimen
        ));

        new GamepadButton(tools, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SelectCommand(
            this::bumper
        ));

        new Trigger(() -> tools.getLeftY()!= 0)
                .whileActiveContinuous(new InstantCommand(() -> armPivot.manual(tools.getLeftY()), armPivot));
        new Trigger(() -> tools.getRightY()!= 0)
                .whileActiveContinuous(new InstantCommand(() -> armExt.manual(tools.getRightY()), armExt));

        //auto align function
        /*
        new GamepadButton(tools, GamepadKeys.Button.RIGHT_BUMPER)
                .and(new Trigger(() -> currentState == States.Global.intake_far || currentState == States.Global.intake_near))
                .whenActive(new AutoAlignRoutine(vision, claw, drive, armExt));

         */
        new GamepadButton(tools, GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(() -> ClawSubsystem.H_target = tempRot1),
                        new InstantCommand(() -> ClawSubsystem.H_target = tempRot2)
                );

        new GamepadButton(driver, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(armPivot::resetEncoder, armPivot));

        new GamepadButton(driver, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(armExt::resetEncoder, armPivot));





        /*
        control objectives:
        toggle - close intake <-> home
        toggle - far intake <-> home
        toggle - high? bucket <-> home
        toggle - low bucket <-> home
        toggle - auto align then pick up <-> claw disengaged

        toggle - specimen <-> home
        button - place specimen & reset (extend down, release claw, extend up)

        + each position should be reached by at most 1 motion.
        + i.e. we can go from close intake to low bucket without going thru home.
         */
        //TODO add reset keys like home-ing the arm and extension


        schedule(new RunCommand(() -> {
            TelemetryPacket packet = new TelemetryPacket();
            Pose2d pose = drive.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y",pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Current State:", currentState.name());
            telemetry.update();

            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }));
        schedule(driveCommand);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (!isStarted()) {
            armPivot.holdPosition();
            armExt.holdPosition();
        }
        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }

    public InstantCommand swapState(States.Global state) {
        return new InstantCommand(() -> currentState = state);
    }
    public Command bumper() {
        switch (currentState) {
            case intake_far:
            case intake_near:
                return new InstantCommand(claw::toggleFingerState, claw);
            case bucket:
                return new BucketRoutine(claw);
            case specimen:
                return new SpecimenRoutine(claw, armExt);
            default:
                return new InstantCommand();
        }
    }
    public SequentialCommandGroup returnHome() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> claw.setClawState(States.Claw.home), claw),
                new InstantCommand(() -> claw.setFingerState(States.Finger.closed), claw),
                new PIDMoveCommand(armExt, States.ArmExtension.home),
                new PIDMoveCommand(armPivot, States.ArmPivot.intake),
                swapState(States.Global.home)
        );
    }


}
