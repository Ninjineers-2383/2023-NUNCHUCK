package com.team2383.diffy.subsystems.pinkArm.telescope;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopeSubsystem extends SubsystemBase {
    private final Ninja_CANSparkMax m_rightMotor;
    private final Ninja_CANSparkMax m_leftMotor;

    private final PIDController m_fb = new PIDController(TelescopeConstants.kP, 0, 0);
    private final SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(TelescopeConstants.kS,
            TelescopeConstants.kV, TelescopeConstants.kA);

    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId
            .identifyVelocitySystem(TelescopeConstants.kV, TelescopeConstants.kA);

    private double m_voltage;

    private double m_speed;
    private double m_extension;

    private double m_desiredExtension;

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    double m_simVelocity = 0;

    public TelescopeSubsystem() {
        m_rightMotor = new Ninja_CANSparkMax(TelescopeConstants.kExtensionRightID, MotorType.kBrushless);
        m_leftMotor = new Ninja_CANSparkMax(TelescopeConstants.kExtensionLeftID, MotorType.kBrushless);

        m_rightMotor.getEncoder().setPosition(0);
        m_leftMotor.getEncoder().setPosition(0);

        m_rightMotor.setSmartCurrentLimit(30);
        m_leftMotor.setSmartCurrentLimit(30);

        m_rightMotor.getEncoder().setPositionConversionFactor(TelescopeConstants.kRotToInches);
        m_leftMotor.getEncoder().setPositionConversionFactor(TelescopeConstants.kRotToInches);

        m_rightMotor.getEncoder().setVelocityConversionFactor(TelescopeConstants.kRotToInches);
        m_leftMotor.getEncoder().setVelocityConversionFactor(TelescopeConstants.kRotToInches);
    }

    public void periodic() {
        m_speed = m_rightMotor.get() / 2.0 + m_leftMotor.get() / 2.0;

        m_extension = getExtension();
    }

    public void simulate() {
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

        SmartDashboard.putNumber("Simulated Extension", getExtension());
    }

    public void setVelocity(double desiredSpeed) {
        m_desiredExtension += desiredSpeed * 0.02;

        setExtension(m_desiredExtension);
    }

    public void setExtension(double extension) {
        if (m_desiredExtension > TelescopeConstants.kUpperBound) {
            m_desiredExtension = TelescopeConstants.kUpperBound;
        } else if (m_desiredExtension < TelescopeConstants.kLowerBound) {
            m_desiredExtension = TelescopeConstants.kLowerBound;
        } else {
            m_desiredExtension = extension;
        }

        goal = new TrapezoidProfile.State(m_desiredExtension, 0);

        double ff = m_ff.calculate(m_speed, 0.02);
        double fb = m_fb.calculate(m_extension, goal.position);

        m_voltage = ff + fb;

        setVoltage();
        
    }

    public double getExtension() {
        return m_rightMotor.getPosition() / 2.0 + m_leftMotor.getPosition() / 2.0;
    }

    private void setVoltage() {
        m_rightMotor.setVoltage(m_voltage);
        m_leftMotor.setVoltage(m_voltage);
    }
}
