package org.firstinspires.ftc.teamcode.util;

public class States {

    // Arm Enums
    public enum ArmPivot {
        specimen,
        bucket,
        intake
    }

    public enum ArmExtension {
        home,
        intake, // is a free value, arm should extend to this but not hold it so driver can adjust
        bucket,
        specimen
    }

    // Claw Enums
    // Claw merges both wrist and hand states
    public enum Claw {
        home,
        intake,
        specimen,
        bucket
    }

    public enum Finger {
        opened,
        closed
    }


}
