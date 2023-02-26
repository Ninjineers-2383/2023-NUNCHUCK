package com.team2383.diffy.subsystems.pinkArm.telescope;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class TelescopeConstants {

    public static final int EXTENSION_LEFT_ID = 4;
    public static final int EXTENSION_RIGHT_ID = 5;

    public static final int MAX_CURRENT = 20;
    public static final double ROTATION_CONVERSION = 0.36363636363636365 * 12 / 20; // TODO: Recalcuate conversion

    public static final double CURRENT_THRESHOLD = 10;

    public static final double UPPER_BOUND = 38;
    public static final double LOWER_BOUND = 0.0;
    public static final double SAFETY_BOUND = 1;

    public static final double kEXTENSION_BIAS = -0.15;

    public static final PIDController PID_CONTROLLER = new PIDController(4, 0, 0);
    public static final ArmFeedforward FEEDFORWARD_CONTROLLER = new ArmFeedforward(0.7, -2, 4.0, 0.01);
    public static final TrapezoidProfile.Constraints TRAPEZOIDAL_CONSTRAINTS = new TrapezoidProfile.Constraints(12,
            6);
    public static final LinearSystem<N1, N1, N1> SIMULATION_SUBSYSTEM = LinearSystemId
            .identifyVelocitySystem(TelescopeConstants.FEEDFORWARD_CONTROLLER.kv,
                    TelescopeConstants.FEEDFORWARD_CONTROLLER.ka);
}