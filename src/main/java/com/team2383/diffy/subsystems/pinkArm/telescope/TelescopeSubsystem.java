package com.team2383.diffy.subsystems.pinkArm.telescope;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopeSubsystem extends SubsystemBase {
    private final Ninja_CANSparkMax m_rightMotor;
    private final Ninja_CANSparkMax m_leftMotor;

    private final PIDController m_PIDController = new PIDController(TelescopeConstants.kP, 0, 0);

    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId
            .identifyVelocitySystem(TelescopeConstants.kV, TelescopeConstants.kA);

    private double m_voltage;

    private double m_extensionInches;

    private double m_velocityInches;

    private double m_desiredExtension;

    double m_simVelocity = 0;

    private final Mechanism2d m_mechanism2d;
    private final MechanismRoot2d m_mechanismRoot2d;
    private final MechanismLigament2d m_telescopeLigament;

    public TelescopeSubsystem() {
        m_rightMotor = new Ninja_CANSparkMax(TelescopeConstants.kExtensionRightID, MotorType.kBrushless);
        m_leftMotor = new Ninja_CANSparkMax(TelescopeConstants.kExtensionLeftID, MotorType.kBrushless);

        m_rightMotor.getEncoder().setPosition(0);
        m_leftMotor.getEncoder().setPosition(0);

        m_rightMotor.setSmartCurrentLimit(30);
        m_leftMotor.setSmartCurrentLimit(30);

        m_rightMotor.getEncoder().setPositionConversionFactor(TelescopeConstants.kRotToInches);
        m_leftMotor.getEncoder().setPositionConversionFactor(TelescopeConstants.kRotToInches);

        m_rightMotor.getEncoder().setVelocityConversionFactor(TelescopeConstants.kRotToInches / 60);
        m_leftMotor.getEncoder().setVelocityConversionFactor(TelescopeConstants.kRotToInches / 60);

        m_mechanism2d = new Mechanism2d(3, 3);
        m_mechanismRoot2d = m_mechanism2d.getRoot("Bottom Pivot", 1.5, 1.5);

        m_telescopeLigament = m_mechanismRoot2d.append(new MechanismLigament2d("Telescope Ligament", 0.5, 0.5));
    }

    public void periodic() {
        m_velocityInches = getVelocity();
        m_extensionInches = getExtensionInches();
        
        m_telescopeLigament.setLength(m_extensionInches);

        SmartDashboard.putData("Telescope Mechanism", m_mechanism2d);
        calculateVoltage();
    }

    @Override
    public void simulationPeriodic() {
        m_simVelocity = m_motorSim.calculateX(VecBuilder.fill(m_simVelocity), VecBuilder.fill(m_voltage), 0.02)
                .get(0, 0);

        m_rightMotor.set(m_simVelocity);
        m_leftMotor.set(m_simVelocity);

        m_rightMotor.setPosition(m_rightMotor.getPosition() + m_simVelocity * 0.02);
        m_leftMotor.setPosition(m_leftMotor.getPosition() + m_simVelocity * 0.02);

        SmartDashboard.putNumber("Simulated Telescope Motor Velocity",
                m_rightMotor.get() / 2 + m_leftMotor.get() / 2);

        SmartDashboard.putNumber("Simulated Telescope Motor Position",
                m_rightMotor.getPosition() / 2 + m_leftMotor.getPosition() / 2);

        SmartDashboard.putNumber("Simulated Extension", getExtensionInches());
    }

    public void setExtension(double extension) {
        if (m_desiredExtension > TelescopeConstants.kUpperBound) {
            m_desiredExtension = TelescopeConstants.kUpperBound;
        } else if (m_desiredExtension < TelescopeConstants.kLowerBound) {
            m_desiredExtension = TelescopeConstants.kLowerBound;
        } else {
            m_desiredExtension = extension;
        }
    }

    public void setVelocity(double desiredSpeed) {
        m_desiredExtension += desiredSpeed * 0.02;
        setExtension(m_desiredExtension);
    }

    private void calculateVoltage() {
        m_voltage = m_PIDController.calculate(m_extensionInches, m_desiredExtension);
        m_voltage += Math.signum(m_voltage) * TelescopeConstants.kS;

        setVoltage();
    }

    /* Velocity measured in inches per minute*/
    public double getVelocity() {
        return (m_rightMotor.get() + m_leftMotor.get()) / 2.0;
    }

    public double getExtensionInches() {
        return (m_rightMotor.getPosition() + m_leftMotor.getPosition()) / 2.0;
    }

    private void setVoltage() {
        m_rightMotor.setVoltage(m_voltage);
        m_leftMotor.setVoltage(m_voltage);
    }

    public boolean isAtPosition() {
        return Math.abs(m_desiredExtension - m_extensionInches) < 0.1;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");
        
        builder.addDoubleProperty("Extension (Inches)", () -> {
            return m_extensionInches;
        }, null);

        builder.addDoubleProperty("Desired Extension (Inches)", () -> {
            return m_desiredExtension;
        }, null);

        builder.addDoubleProperty("Velocity (Inches per Second)", () -> {
            return m_velocityInches;
        }, null);

        builder.addDoubleProperty("Voltage (Volts)", () -> {
            return m_voltage;
        }, null);
    }
}
