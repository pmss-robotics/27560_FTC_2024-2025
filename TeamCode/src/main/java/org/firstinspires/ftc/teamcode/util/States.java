package org.firstinspires.ftc.teamcode.util;

public class States {

    // related to each gamepad mapping
    public enum Global {
        home,
        intake_far,
        intake_near,
        bucket,
        specimen,
        specimenIntake,
    }

    // Arm Enums
    public enum ArmPivot {
        start,
        specimen,
        specimenIntake,
        bucket,
        intake,
        home,
    }

    public enum ArmExtension {
        start,
        home,
        intake, // is a free value, arm should extend to this but not hold it so driver can adjust
        bucket,
        specimen_1,
        specimen_2
    }

    // Claw Enums
    // Claw merges both wrist and hand states
    public enum Claw {
        start,
        home,
        intake,
        specimen,
        specimenIntake,
        bucket
    }

    public enum Finger {
        opened,
        closed
    }


}