package com.team2383.nunchuck.subsystems.pinkArm.telescope;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.lib.math.Clip;
import com.team2383.lib.simulation.SparkMaxSimWrapper;
import com.team2383.nunchuck.Robot;
import com.team2383.nunchuck.helpers.TrapezoidalSubsystemBase;
import com.team2383.nunchuck.subsystems.pinkArm.pivot.PivotConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelescopeSubsystem extends TrapezoidalSubsystemBase {
    private final SparkMaxSimWrapper m_motor;

    private Supplier<Rotation2d> m_pivotAngle;

    public TelescopeSubsystem(Supplier<Rotation2d> pivotAngle) {
        super("Telescope", TelescopeConstants.TRAPEZOIDAL_CONSTRAINTS, TelescopeConstants.SIMULATION_SUBSYSTEM,
                TelescopeConstants.POSITION_THRESHOLD);
        m_pivotAngle = pivotAngle;

        m_motor = new SparkMaxSimWrapper(TelescopeConstants.EXTENSION_ID, MotorType.kBrushless);

        m_motor.restoreFactoryDefaults();

        resetPosition();

        m_motor.setSmartCurrentLimit(TelescopeConstants.MAX_CURRENT);

        m_motor.setInverted(true);

        SmartDashboard.putData("Telescope FF", TelescopeConstants.FEEDFORWARD_CONTROLLER);
        SmartDashboard.putData("Telescope PID", TelescopeConstants.PID_CONTROLLER);
    }

    public void setPivotAngle(Supplier<Rotation2d> pivotAngle) {
        m_pivotAngle = pivotAngle;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    /**
     * Uses trapezoidal motion-profiling to implement pseudo-positional control
     * Using this method disables velocity control
     * 
     * @param extension
     * @return boolean state to determine whether the input extension is safe
     */
    public boolean setGoal(double extension) {
        boolean isSafe = (m_pivotAngle != null ? m_pivotAngle.get().getRadians() : 0) < PivotConstants.LOWER_SAFETY
                .getRadians()
                || (m_pivotAngle != null ? m_pivotAngle.get().getRadians() : 0) > PivotConstants.UPPER_SAFETY
                        .getRadians();
        double adjustedExtension = Clip.clip(TelescopeConstants.LOWER_BOUND, extension,
                isSafe ? TelescopeConstants.UPPER_BOUND : TelescopeConstants.SAFETY_BOUND);
        super.setGoal(new TrapezoidProfile.State(adjustedExtension, 0));
        return adjustedExtension == extension;
    }

    public void forceGoal(double extension) {
        super.setGoal(new TrapezoidProfile.State(extension, 0));
    }

    /**
     * Set velocity of the extension using PID and feedforward control
     * If used externally, call disable() before using this method
     * Make sure to call enable() to resume positional control
     * 
     * @param desiredVelocity in inches per second
     */
    public void setVelocity(double desiredVelocity) {
        super.setVelocity(desiredVelocity);
    }

    /* Velocity measured in inches per second */
    public double getVelocity() {
        return m_motor.get() * TelescopeConstants.ROTATION_CONVERSION / 60.0;
    }

    public double getExtensionInches() {
        return m_motor.getPosition() * TelescopeConstants.ROTATION_CONVERSION;
    }

    protected TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getExtensionInches(), getVelocity());
    }

    /** Handles simulation */
    protected void setSimulatedMotors(Matrix<N1, N1> matrix) {
        double simVelocity = matrix.get(0, 0);
        m_motor.setSimVelocity(simVelocity);

        m_motor.setSimPosition(m_motor.getPosition() + simVelocity * 0.02);
    }

    private double getCosGravityAngle() {
        return m_pivotAngle != null ? m_pivotAngle.get().getRadians() : 0;
    }

    /**
     * PIDF calculations used by trapezoidal motion profiling
     * 
     * @param velocity in radians per second
     */
    @Override
    protected double calculateVoltage(double velocity, double position) {
        double voltage = TelescopeConstants.PID_CONTROLLER.calculate(getExtensionInches(), position);
        voltage += TelescopeConstants.FEEDFORWARD_CONTROLLER
                .calculate(Robot.isReal() ? getCosGravityAngle() : -Math.PI / 2, velocity);
        voltage += velocity < 0 ? TelescopeConstants.kEXTENSION_BIAS : 0;
        return voltage;
    }

    /**
     * Sets the voltage of the pivot motors
     * 
     * @param voltage
     */
    @Override
    protected void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    public void resetPosition() {
        DataLogManager.log("Telescope reset");
        m_motor.getEncoder().setPosition(0);
    }

    public double getCurrent() {
        return m_motor.getOutputCurrent();
    }

    public boolean getZeroState() {
        return getCurrent() > TelescopeConstants.CURRENT_THRESHOLD;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty("Pivot angle does the existy", () -> m_pivotAngle != null, null);

        builder.addDoubleProperty("Pivot Angle", () -> m_pivotAngle != null ? m_pivotAngle.get().getRadians() : 0,
                null);

        builder.addDoubleProperty("Current", this::getCurrent,
                null);

    }
}
