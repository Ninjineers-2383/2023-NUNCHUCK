package com.team2383.nunchuck.subsystems.pinkArm.pivot;

import com.team2383.nunchuck.helpers.TunableFeedforward;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class PivotConstants {
    public static final int BOTTOM_MOTOR_LEFT_ID = 2;
    public static final int BOTTOM_MOTOR_RIGHT_ID = 3;
    public static final int ABS_ENCODER_ID = 6;

    public static final int MAX_CURRENT = 40;
    public static final double ENCODER_OFFSET = 3.365 / (2 * Math.PI);

    public static final Rotation2d POSITION_THRESHOLD = Rotation2d.fromDegrees(15);

    public static final Rotation2d UPPER_BOUND = Rotation2d.fromDegrees(170);
    public static final Rotation2d LOWER_BOUND = Rotation2d.fromDegrees(-170);

    public static final Rotation2d UPPER_SAFETY = Rotation2d.fromDegrees(0);
    public static final Rotation2d LOWER_SAFETY = Rotation2d.fromDegrees(-0);

    public static final double kG = 0.5;

    public static final double kGOFFSET = 19;

    public static final PIDController PID_CONTROLLER = new PIDController(6, 0, 0);
    public static final TunableFeedforward FEEDFORWARD_CONTROLLER = new TunableFeedforward(0.2, 1.66, 0.001);

    public static final Rotation2d VELOCITY_CONVERSION_FACTOR = Rotation2d.fromDegrees(360 * 60);

    // Values will be in radians and seconds
    public static final TrapezoidProfile.Constraints TRAPEZOIDAL_CONSTRAINTS = new TrapezoidProfile.Constraints(17, 7);

    public static final LinearSystem<N1, N1, N1> SIMULATION_SUBSYSTEM = LinearSystemId.identifyVelocitySystem(
            PivotConstants.FEEDFORWARD_CONTROLLER.kv,
            PivotConstants.FEEDFORWARD_CONTROLLER.ka);
}