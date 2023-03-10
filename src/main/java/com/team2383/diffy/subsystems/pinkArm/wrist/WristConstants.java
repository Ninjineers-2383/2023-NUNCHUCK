package com.team2383.diffy.subsystems.pinkArm.wrist;

import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class WristConstants {

    public static final int kMotorID = 8;
    public static final int encoderOffset = 2970; // -2.502

    public static final int kMaxCurrent = 20;

    public static final Rotation2d POSITON_THRESHOLD = Rotation2d.fromDegrees(20);

    public static final Rotation2d kUpperBound = Rotation2d.fromDegrees(180);
    public static final Rotation2d kLowerBound = Rotation2d.fromDegrees(-180);
    public static final Rotation2d feedAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d outFeedAngle = Rotation2d.fromDegrees(0);

    public static final PIDController PID_CONTROLLER = new PIDController(9, 0, 0);
    public static final ArmFeedforward FEEDFORWARD_CONTROLLER = new ArmFeedforward(0.3, -1.15, 4, 0.10864); // ks:
                                                                                                            // 0.21143
    // public static final ArmFeedforward FEEDFORWARD_CONTROLLER = new
    // ArmFeedforward(0, 0, 0, 0);
    public static final TrapezoidProfile.Constraints TRAPEZOIDAL_CONSTRAINTS = new TrapezoidProfile.Constraints(4,
            4);
    public static final LinearSystem<N1, N1, N1> SIMULATION_SUBSYSTEM = LinearSystemId
            .identifyVelocitySystem(TelescopeConstants.FEEDFORWARD_CONTROLLER.kv,
                    TelescopeConstants.FEEDFORWARD_CONTROLLER.ka);
}