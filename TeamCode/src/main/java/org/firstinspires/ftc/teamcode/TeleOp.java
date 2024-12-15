package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.commands.BucketRoutine;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.PIDMoveCommand;
import org.firstinspires.ftc.teamcode.commands.SpecimenRoutine;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
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
    public static double tempRot1 = 75;
    public static double tempRot2 = 165;
    public static double pivotThreshold = 0.5;



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
        drive = new DriveSubsystem(new Follower(hardwareMap), new Pose(72,72,0), telemetry);
        // The driveCommand uses methods defined in the DriveSubsystem to create behaviour.
        // we're passing in methods to get values instead of straight values because it avoids
        // disturbing the structure of the CommandOpMode. The aim is to define bindings in this
        // initialize() method through Commands and these will be looped and acted in the (hidden)
        // run() loop.

        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver.getLeftX() * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> driver.getLeftY() * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> -driver.getRightX() * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
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
                        new InstantCommand(() -> claw.setClawState(States.Claw.intake), claw),
                        new InstantCommand(() -> claw.setFingerState(States.Finger.opened), claw),
                        new PIDMoveCommand(armPivot, States.ArmPivot.intake),
                        new PIDMoveCommand(armExt, States.ArmExtension.intake),
                        new InstantCommand(() -> claw.setClawState(States.Claw.home), claw),
                        swapState(States.Global.intake_far)
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> claw.setClawState(States.Claw.intake), claw),
                        returnHome()
                ),
                () -> currentState != States.Global.intake_far
        ));
        // near intake
        new GamepadButton(tools, GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(() -> claw.setClawState(States.Claw.intake), claw),
                        new InstantCommand(() -> claw.setFingerState(States.Finger.opened), claw),
                        new PIDMoveCommand(armPivot, States.ArmPivot.intake),
                        new PIDMoveCommand(armExt, States.ArmExtension.home),
                        new InstantCommand(() -> claw.setClawState(States.Claw.home), claw),
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

        new GamepadButton(tools, GamepadKeys.Button.DPAD_UP)
                .whenHeld(new InstantCommand(() -> armExt.manual(true) ,armExt))
                .whenReleased(
                   new InstantCommand(() -> {
                       armExt.leftExtension.setPower(0);
                       armExt.resetTarget();
                   }, armExt)
                );

        new GamepadButton(tools, GamepadKeys.Button.DPAD_DOWN)
                .whenHeld(new InstantCommand(() -> armExt.manual(false) ,armExt))
                .whenReleased(
                        new InstantCommand(() -> {
                            armExt.leftExtension.setPower(0);
                            armExt.resetTarget();
                        }, armExt)
                );


        new Trigger(() -> tools.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > pivotThreshold)
                .whileActiveContinuous(new InstantCommand(() -> armPivot.manual(false), armPivot))
                .whenInactive(
                        new InstantCommand(() -> {
                            armPivot.leftPivot.setPower(0);
                            armPivot.rightPivot.setPower(0);
                            armPivot.resetTarget();
                        }, armPivot)
                );

        new Trigger(() -> tools.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > pivotThreshold)
                .whileActiveContinuous(new InstantCommand(() -> armPivot.manual(true), armPivot))
                .whenInactive(
                        new InstantCommand(() -> {
                            armPivot.leftPivot.setPower(0);
                            armPivot.rightPivot.setPower(0);
                            armPivot.resetTarget();
                        }, armPivot)
                );


        /*
        new Trigger(() -> tools.getRightY()!= 0)
                .whileActiveContinuous(new InstantCommand(() -> armExt.manual(tools.getRightY()), armExt));

         */

        //auto align function
        /*
        new GamepadButton(tools, GamepadKeys.Button.LEFT_BUMPER)
                .and(new Trigger(() -> currentState == States.Global.intake_far || currentState == States.Global.intake_near))
                .whenActive(new AutoAlignRoutine(vision, claw, drive, armExt));

         */
        new GamepadButton(tools, GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new InstantCommand(() -> ClawSubsystem.H_target = tempRot1),
                        new InstantCommand(() -> ClawSubsystem.H_target = tempRot2)
                );

        new GamepadButton(driver, GamepadKeys.Button.A).whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> armPivot.setState(States.ArmPivot.home)),
                new InstantCommand(()-> armPivot.leftPivot.setPower(0)),
                new InstantCommand(()-> armPivot.rightPivot.setPower(0)),
                new InstantCommand(armPivot::resetEncoder, armPivot)));


        new GamepadButton(driver, GamepadKeys.Button.B).whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> ArmExtensionSubsystem.target = 0),
                new InstantCommand(()-> armExt.leftExtension.setPower(0)),
                new InstantCommand(armExt::resetEncoder, armExt)));


        schedule(new RunCommand(() -> {
            telemetry.addData("Current State:", currentState.name());
            telemetry.update();
        }));
        schedule(driveCommand);
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

    public InstantCommand swapState(States.Global state) {
        return new InstantCommand(() -> currentState = state);
    }
    public Command bumper() {
        switch (currentState) {
            case intake_far:
            case intake_near:
                return new SequentialCommandGroup(
                        new PIDMoveCommand(armPivot, States.ArmPivot.home),
                        new InstantCommand(claw::toggleFingerState)
                );
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
                new InstantCommand(() -> claw.setFingerState(States.Finger.closed), claw),
                new PIDMoveCommand(armExt, States.ArmExtension.home),
                new PIDMoveCommand(armPivot, States.ArmPivot.home),
                new InstantCommand(() -> claw.setClawState(States.Claw.home), claw),
                swapState(States.Global.home)
        );
    }


}
