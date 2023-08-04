package com.team2383.nunchuck.subsystems.pinkArm.telescope;

import com.team2383.lib.controller.TunableArmFeedforward;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class TelescopeConstants {

    public static final int EXTENSION_ID = 5;

    public static final int MAX_CURRENT = 40;
    public static final double ROTATION_CONVERSION = 19.0 / 90.0;

    public static final double CURRENT_THRESHOLD = 10;

    public static final double UPPER_BOUND = 38;
    public static final double LOWER_BOUND = 0.0;
    public static final double SAFETY_BOUND = 2;

    public static final double kEXTENSION_BIAS = -0.15;

    public static final double POSITION_THRESHOLD = 2;

    public static final PIDController PID_CONTROLLER = new PIDController(4, 0, 0);
    public static final TunableArmFeedforward FEEDFORWARD_CONTROLLER = new TunableArmFeedforward(0.25, 0, 0.7, 0.01);
    public static final TrapezoidProfile.Constraints TRAPEZOIDAL_CONSTRAINTS = new TrapezoidProfile.Constraints(40,
            50);
    public static final LinearSystem<N1, N1, N1> SIMULATION_SUBSYSTEM = LinearSystemId
            .identifyVelocitySystem(TelescopeConstants.FEEDFORWARD_CONTROLLER.kv,
                    TelescopeConstants.FEEDFORWARD_CONTROLLER.ka);
}