package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Methods.intake;
import static org.firstinspires.ftc.teamcode.util.Methods.lowBucket;
import static org.firstinspires.ftc.teamcode.util.Methods.returnHome;
import static org.firstinspires.ftc.teamcode.util.Methods.specimen;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.SpecimenRoutine;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmPivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.PoseTransfer;
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
    public static double driveMult = 1;

    Pose startPose;
    @Override
    public void initialize() {
        startPose = PoseTransfer.pose;
        // data sent to telemetry shows up on dashboard and driverGamepad station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        // GamepadEx wraps gamepad 1 or 2 for easier implementations of more complex key bindings
        driver = new GamepadEx(gamepad1);
        tools = new GamepadEx(gamepad2);
        // The driveSubsystem wraps Roadrunner's MecanumDrive to combine with Commands.
        Constants.setConstants(FConstants.class, LConstants.class);
        drive = new DriveSubsystem(new Follower(hardwareMap), startPose, telemetry);
        // The driveCommand uses methods defined in the DriveSubsystem to create behaviour.
        // we're passing in methods to get values instead of straight values because it avoids
        // disturbing the structure of the CommandOpMode. The aim is to define bindings in this
        // initialize() method through Commands and these will be looped and acted in the (hidden)
        // run() loop.

        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver.getLeftX() * driveMult,
                () -> driver.getLeftY() * driveMult,
                () -> -driver.getRightX() * driveMult,
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

        Servo lights = hardwareMap.get(Servo.class, "lights");
        lights.setPosition(1.0);


        // far intake
        new GamepadButton(tools, GamepadKeys.Button.B).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        intake(armExt, armPivot, claw, States.ArmExtension.intake, States.Claw.intake),
                        swapState(States.Global.intake_far)
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> claw.setClawState(States.Claw.intake)),
                        returnHome(armExt, armPivot, claw),
                        swapState(States.Global.home)
                ),
                () -> currentState != States.Global.intake_far
        ));
        // near intake
        new GamepadButton(tools, GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        intake(armExt, armPivot, claw, States.ArmExtension.home, States.Claw.intake),
                        swapState(States.Global.intake_near)
                ),
                new SequentialCommandGroup(
                        returnHome(armExt, armPivot, claw),
                        swapState(States.Global.home)
                ),
                () -> currentState != States.Global.intake_near
        ));
        // specimen intake
        new GamepadButton(tools, GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new ConditionalCommand(
           new SequentialCommandGroup(
                   intake(armExt, armPivot, claw, States.ArmExtension.home, States.Claw.specimenIntake),
                   swapState(States.Global.specimenIntake)
                   ),
           new SequentialCommandGroup(
                   returnHome(armExt, armPivot, claw),
                   swapState(States.Global.home)
           ),
           () -> currentState != States.Global.specimenIntake
        ));

        // bucket
        new GamepadButton(tools, GamepadKeys.Button.X).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        lowBucket(armExt, armPivot, claw),
                        swapState(States.Global.bucket)
                ),
                new SequentialCommandGroup(
                        returnHome(armExt, armPivot, claw),
                        swapState(States.Global.home)
                ),
                () -> currentState != States.Global.bucket
        ));
        // specimen
        new GamepadButton(tools, GamepadKeys.Button.Y).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        specimen(armExt, armPivot, claw),
                        swapState(States.Global.specimen)
                ),
                new SequentialCommandGroup(
                        returnHome(armExt, armPivot, claw),
                        swapState(States.Global.home)
                ),
                () -> currentState != States.Global.specimen
        ));

        new GamepadButton(tools, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new SelectCommand(
            this::bumper
        ));


        new GamepadButton(tools, GamepadKeys.Button.DPAD_UP)
                .whileHeld(new InstantCommand(() -> armExt.manual(true), armExt))
                .whenReleased(
                   new InstantCommand(() -> {
                       armExt.leftExtension.setPower(0);
                       armExt.resetTarget();
                   }, armExt)
                );

        new GamepadButton(tools, GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(new InstantCommand(() -> armExt.manual(false) ,armExt))
                .whenReleased(
                        new InstantCommand(() -> {
                            armExt.leftExtension.setPower(0);
                            armExt.resetTarget();
                        }, armExt)
                );



        /*

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

         */


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

        new GamepadButton(driver, GamepadKeys.Button.A).whileHeld(new ParallelCommandGroup(
                new InstantCommand(() -> {
                    ArmPivotSubsystem.target = 0;
                    armPivot.leftPivot.setPower(0);
                    armPivot.rightPivot.setPower(0);
                    armPivot.resetEncoder();
                }, armPivot)
        ));


        new GamepadButton(driver, GamepadKeys.Button.B).whileHeld(new ParallelCommandGroup(
                new InstantCommand(() -> {
                    ArmExtensionSubsystem.target = 0;
                    armExt.leftExtension.setPower(0);
                    armExt.resetEncoder();
                }, armExt)));

        new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> driveMult = 0.5))
                .whenReleased(new InstantCommand(() -> driveMult = 1));

        new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                        new InstantCommand(() -> driveMult = 0.33))
                .whenReleased(new InstantCommand(() -> driveMult = 1));

        new GamepadButton(driver, GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> DriveCommand.target = startPose.getHeading())
        );
        new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> DriveCommand.target = startPose.getHeading() - Math.toRadians(90))
        );
        new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> DriveCommand.target = startPose.getHeading() + Math.toRadians(180))
        );
        new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> DriveCommand.target = startPose.getHeading() + Math.toRadians(90))
        );

        schedule(new RunCommand(() -> {
            telemetry.addData("Current State:", currentState.name());
            telemetry.addData("heading (deg) target: ", DriveCommand.target);
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
        PoseTransfer.pose = new Pose();
    }

    public InstantCommand swapState(States.Global state) {
        return new InstantCommand(() -> currentState = state);
    }
    public Command bumper() {
        switch (currentState) {
            case intake_far:
            case intake_near:
                return new SequentialCommandGroup(
                        new InstantCommand(claw::toggleFingerState)
                );
            case bucket:
                return new InstantCommand(() -> claw.setFingerState(States.Finger.opened));
            case specimen:
                return new SpecimenRoutine(claw, armExt);
            default:
                return new InstantCommand();
        }
    }

}
