package com.team2383.diffy.subsystems.pinkArm.telescope;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class TelescopeConstants {
    // FF Values
    public static final double kS = 0.55714;
    public static final double kV = 3;
    public static final double kA = 0.013931;

    public static final double kP = 1;

    public static final double TELESCOPE_VELOCITY = .2;

    public static final TrapezoidProfile.Constraints TRAPEZOIDAL_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);

    public static final double UPPER_BOUND = 38;
    public static final double LOWER_BOUND = 0.0;
    public static final double SAFETY_BOUND = 1;

    public static final double MAX_CURRENT = 40.0;

    public static final int EXTENSION_LEFT_ID = 4;
    public static final int EXTENSION_RIGHT_ID = 5;

    public static final double ROTATION_CONVERSION = 20.0 / 324.0;

}