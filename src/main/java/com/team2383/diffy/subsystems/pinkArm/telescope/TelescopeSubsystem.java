package com.team2383.diffy.subsystems.pinkArm.telescope;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.helpers.Clip;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;
import com.team2383.diffy.helpers.TrapezoidalSubsystemBase;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TelescopeSubsystem extends TrapezoidalSubsystemBase {
    private final Ninja_CANSparkMax m_rightMotor;
    private final Ninja_CANSparkMax m_leftMotor;

    private double m_voltage;

    private Supplier<Rotation2d> m_pivotAngle;


    public TelescopeSubsystem(Supplier<Rotation2d> pivotAngle) {
        super("Telescope", TelescopeConstants.TRAPEZOIDAL_CONSTRAINTS, TelescopeConstants.SIMULATION_SUBSYSTEM);
        m_pivotAngle = pivotAngle;

        m_rightMotor = new Ninja_CANSparkMax(TelescopeConstants.EXTENSION_RIGHT_ID, MotorType.kBrushless);
        m_leftMotor = new Ninja_CANSparkMax(TelescopeConstants.EXTENSION_LEFT_ID, MotorType.kBrushless);

        m_rightMotor.getEncoder().setPosition(0);
        m_leftMotor.getEncoder().setPosition(0);

        m_rightMotor.setSmartCurrentLimit(30);
        m_leftMotor.setSmartCurrentLimit(30);
        
        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);


        m_rightMotor.getEncoder().setPositionConversionFactor(-TelescopeConstants.ROTATION_CONVERSION);
        m_leftMotor.getEncoder().setPositionConversionFactor(-TelescopeConstants.ROTATION_CONVERSION);


        m_rightMotor.getEncoder().setVelocityConversionFactor(-TelescopeConstants.ROTATION_CONVERSION / 60);
        m_leftMotor.getEncoder().setVelocityConversionFactor(-TelescopeConstants.ROTATION_CONVERSION / 60);
    }


    /**
     * Uses trapezoidal motion-profiling to implement pseudo-positional control
     * Using this method disables velocity control
     * @param extension
     * @return boolean state to determine whether the input extension is safe
     */
    public boolean setGoal(double extension) {
        boolean isSafe = m_pivotAngle.get().getRadians() < PivotConstants.LOWER_SAFETY.getRadians() || m_pivotAngle.get().getRadians() > PivotConstants.UPPER_SAFETY.getRadians();
        double adjustedExtension = Clip.clip( TelescopeConstants.LOWER_BOUND, extension, isSafe ? TelescopeConstants.UPPER_BOUND: TelescopeConstants.SAFETY_BOUND);
        super.setGoal(new TrapezoidProfile.State(adjustedExtension, 0));
        return adjustedExtension == extension;
    }

    /**
     * Set velocity of the extension using PID and feedforward control
     * If used externally, call disable() before using this method
     * Make sure to call enable() to resume positional control
     * @param desiredVelocity in inches per second
     */
    public void setVelocity(double desiredVelocity) {
        super.setVelocity(desiredVelocity);
    }

    /* Velocity measured in inches per minute*/
    public double getVelocity() {
        return (m_rightMotor.get() + m_leftMotor.get()) / 2.0;
    }

    public double getExtensionInches() {
        return ((m_rightMotor.getPosition()) + (m_leftMotor.getPosition())) / 2.0;
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

    /** PIDF calculations used by trapezoidal motion profiling
     *  @param velocity in radians per second
     */
    protected double calculateVoltage(double velocity) {
        double voltage = TelescopeConstants.PID_CONTROLLER.calculate(getVelocity(), velocity);
        voltage += TelescopeConstants.FEEDFORWARD_CONTROLLER.calculate(m_pivotAngle.get().getRadians() + 90, velocity);
        return voltage;
    }

    /**
     * Sets the voltage of the pivot motors
     * @param voltage
     */
    protected void setVoltage(double voltage) {
        m_rightMotor.setVoltage(m_voltage);
        m_leftMotor.setVoltage(m_voltage);
    }
}
    