package com.team2383.diffy.subsystems.pinkArm.telescope;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.helpers.Clip;
import com.team2383.diffy.helpers.LinearVelocityWrapper;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class TelescopeSubsystem extends TrapezoidProfileSubsystem {
    private final Ninja_CANSparkMax m_rightMotor;
    private final Ninja_CANSparkMax m_leftMotor;

    private final PIDController m_PIDController = new PIDController(TelescopeConstants.kP, 0, 0);

    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId
            .identifyVelocitySystem(TelescopeConstants.kV, TelescopeConstants.kA);

    private double m_voltage;

    private double m_extensionInches;

    private double m_velocity = 0;
    private double m_desiredExtension;
    private Supplier<Rotation2d> m_pivotAngle;
    private double m_simVelocity = 0;


    public TelescopeSubsystem(Supplier<Rotation2d> pivotAngle) {
        super(TelescopeConstants.TRAPEZOIDAL_CONSTRAINTS);
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

    public void periodic() {
        m_extensionInches = getExtensionInches();
        m_velocity = getVelocity();
    }

    @Override
    public void simulationPeriodic() {
        m_simVelocity = m_motorSim.calculateX(VecBuilder.fill(m_simVelocity), VecBuilder.fill(m_voltage), 0.02)
                .get(0, 0);

        m_rightMotor.setSimVelocity(m_simVelocity);
        m_leftMotor.setSimVelocity(m_simVelocity);

        m_rightMotor.setSimPosition(m_rightMotor.getPosition() + m_simVelocity * 0.02);
        m_leftMotor.setSimPosition(m_leftMotor.getPosition() + m_simVelocity * 0.02);
    }

    @Override
    protected void useState(State state) {
       setVelocity(state.velocity);
    }

    public boolean setGoalSafely(double extension) {
        boolean isSafe = m_pivotAngle.get().getRadians() < PivotConstants.LOWER_SAFETY.getRadians() || m_pivotAngle.get().getRadians() > PivotConstants.UPPER_SAFETY.getRadians();
        double adjustedExtension = Clip.clip( TelescopeConstants.LOWER_BOUND, extension, isSafe ? TelescopeConstants.UPPER_BOUND: TelescopeConstants.SAFETY_BOUND);
        super.setGoal(adjustedExtension);
        return adjustedExtension == extension;
    }

    /** Sets desired extension in inches */
    @Override
    public void setGoal(double extension) {
        setGoalSafely(extension);
    }

    /**
     * Set velocity of the extension using PID and feedforward control
     * If used externally, call disable() before using this method
     * Make sure to call enable() to resume positional control
     * @param desiredVelocity in inches per second
     */
    public void setVelocity(double desiredVelocity) {
        m_voltage = m_PIDController.calculate(m_velocity, desiredVelocity);
        m_voltage += Math.signum(m_voltage) * TelescopeConstants.kS;
        m_rightMotor.setVoltage(m_voltage);
        m_leftMotor.setVoltage(m_voltage);
    }


    /* Velocity measured in inches per minute*/
    public double getVelocity() {
        return (m_rightMotor.get() + m_leftMotor.get()) / 2.0;
    }

    public double getExtensionInches() {
        return ((m_rightMotor.getPosition()) + (m_leftMotor.getPosition())) / 2.0;
    }

    public boolean isAtPosition() {
        return Math.abs(m_desiredExtension - m_extensionInches) < 0.1;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");

        builder.addDoubleProperty("Extension (Raw)", () -> {
            return m_rightMotor.getEncoder().getPosition();
        }, null);
        
        builder.addDoubleProperty("Extension (Inches)", () -> {
            return m_extensionInches;
        }, null);

        builder.addDoubleProperty("Desired Extension (Inches)", () -> {
            return m_desiredExtension;
        }, null);

        builder.addDoubleProperty("Velocity (Inches per Second)", () -> {
            return m_velocity;
        }, null);

        builder.addDoubleProperty("Right Current (Amperage)", () -> {
            return m_rightMotor.getOutputCurrent();
        }, null);

        builder.addDoubleProperty("Left Current (Amperage)", () -> {
            return m_leftMotor.getOutputCurrent();
        }, null);

        builder.addDoubleProperty("Voltage (Volts)", () -> {
            return m_voltage;
        }, null);
    }

    
}
    