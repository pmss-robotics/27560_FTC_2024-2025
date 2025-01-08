package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.KalmanFilterParameters;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftBack";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightBack";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.useBrakeModeInTeleOp = true;
        FollowerConstants.mass = 9.2; // in kg

        FollowerConstants.xMovement = 72.0105;
        FollowerConstants.yMovement = 73.7821;

        FollowerConstants.forwardZeroPowerAcceleration = -42.2;
        FollowerConstants.lateralZeroPowerAcceleration = -42.8;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.2, 0, 0.03, 0);
        FollowerConstants.translationalPIDFFeedForward = 0.015;
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(0.1, 0, 0.01, 0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(0.65, 0, 0.055, 0); //TODO: increase this.
        FollowerConstants.headingPIDFFeedForward = 0.01;
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(2, 0, 0.1, 0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.015, 0, 0.0007, 0.6, 0); // increased p from 0.009
        FollowerConstants.drivePIDFFeedForward = 0.01;
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.1, 0, 0, 0.6, 0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.driveKalmanFilterParameters = new KalmanFilterParameters(6, 1);

        FollowerConstants.zeroPowerAccelerationMultiplier = 2; // maybe set this to 2
        FollowerConstants.centripetalScaling = 0.0003;

        FollowerConstants.pathEndTimeoutConstraint = 500; // in ms
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 3; // TODO: apparently setting this to 3 can fix overshoot issues if we come across them.
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
