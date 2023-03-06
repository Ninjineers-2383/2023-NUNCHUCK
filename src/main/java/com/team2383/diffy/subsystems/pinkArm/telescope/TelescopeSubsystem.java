package com.team2383.diffy.subsystems.pinkArm.telescope;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Robot;
import com.team2383.diffy.helpers.Clip;
import com.team2383.diffy.helpers.SparkMaxSimWrapper;
import com.team2383.diffy.helpers.TrapezoidalSubsystemBase;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;

public class TelescopeSubsystem extends TrapezoidalSubsystemBase {
    private final SparkMaxSimWrapper m_rightMotor;
    private final SparkMaxSimWrapper m_leftMotor;

    private Supplier<Rotation2d> m_pivotAngle;

    public TelescopeSubsystem(Supplier<Rotation2d> pivotAngle) {
        super("Telescope", TelescopeConstants.TRAPEZOIDAL_CONSTRAINTS, TelescopeConstants.SIMULATION_SUBSYSTEM,
                TelescopeConstants.POSITION_THRESHOLD);
        m_pivotAngle = pivotAngle;

        m_rightMotor = new SparkMaxSimWrapper(TelescopeConstants.EXTENSION_RIGHT_ID, MotorType.kBrushless);
        m_leftMotor = new SparkMaxSimWrapper(TelescopeConstants.EXTENSION_LEFT_ID, MotorType.kBrushless);

        m_rightMotor.restoreFactoryDefaults();
        m_leftMotor.restoreFactoryDefaults();

        resetPosition();

        m_rightMotor.setSmartCurrentLimit(TelescopeConstants.MAX_CURRENT);
        m_leftMotor.setSmartCurrentLimit(TelescopeConstants.MAX_CURRENT);

        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);
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

    /* Velocity measured in inches per minute */
    public double getVelocity() {
        return (m_rightMotor.get() + m_leftMotor.get()) * TelescopeConstants.ROTATION_CONVERSION / 120.0;
    }

    public double getExtensionInches() {
        return ((m_rightMotor.getPosition()) + (m_leftMotor.getPosition())) * TelescopeConstants.ROTATION_CONVERSION
                / 2.0;
    }

    public double getCurrent() {
        return (m_rightMotor.getOutputCurrent() + m_leftMotor.getOutputCurrent()) / 2;
    }

    protected TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getExtensionInches(), getVelocity());
    }

    /** Handles simulation */
    protected void setSimulatedMotors(Matrix<N1, N1> matrix) {
        double simVelocity = matrix.get(0, 0);
        m_leftMotor.setSimVelocity(simVelocity);
        m_rightMotor.setSimVelocity(simVelocity);

        m_rightMotor.setSimPosition(m_rightMotor.getPosition() + simVelocity * 0.02);
        m_leftMotor.setSimPosition(m_leftMotor.getPosition() + simVelocity * 0.02);
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
        m_rightMotor.setVoltage(voltage);
        m_leftMotor.setVoltage(voltage);
    }

    public void resetPosition() {
        DataLogManager.log("Telescope reset");
        m_rightMotor.getEncoder().setPosition(0);
        m_leftMotor.getEncoder().setPosition(0);
    }

    private double getAmpDraw() {
        return (m_rightMotor.getOutputCurrent() + m_leftMotor.getOutputCurrent()) / 2;
    }

    public boolean getZeroState() {
        return getAmpDraw() > TelescopeConstants.CURRENT_THRESHOLD;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty("Pivot angle does the existy", () -> m_pivotAngle != null, null);

        builder.addDoubleProperty("Pivot Angle", () -> m_pivotAngle != null ? m_pivotAngle.get().getRadians() : 0,
                null);

        builder.addDoubleProperty("Current", this::getAmpDraw,
                null);

    }
}
