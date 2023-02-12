package com.team2383.diffy.subsystems.PinkArm;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Constants;
import com.team2383.diffy.Constants.TelescopeConstants;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelescopeModule implements Sendable {
    private final Ninja_CANSparkMax m_rightMotor;
    private final Ninja_CANSparkMax m_leftMotor;

    private final PIDController m_fb = new PIDController(Constants.TelescopeConstants.kP, 0, 0);
    private final SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(Constants.TelescopeConstants.kS,
            Constants.TelescopeConstants.kV, Constants.TelescopeConstants.kA);

    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId
            .identifyVelocitySystem(Constants.TelescopeConstants.kV, Constants.TelescopeConstants.kA);

    private double m_voltageLeft;
    private double m_voltageRight;

    private double m_desiredExtension;
    private double m_desiredSpeed;

    private double m_speed;
    private double m_extension;

    private final DataLog m_log;

    private final DoubleLogEntry m_motorCurrent;

    private final DoubleLogEntry m_motorVel;

    private final DoubleLogEntry m_moduleExtensionLog;

    private final DoubleLogEntry m_expectedSpeed;
    private final DoubleLogEntry m_expectedExtension;

    private double m_desired;

    public TelescopeModule(DataLog log) {
        m_rightMotor = new Ninja_CANSparkMax(TelescopeConstants.kExtensionRightID, MotorType.kBrushless);
        m_leftMotor = new Ninja_CANSparkMax(TelescopeConstants.kExtensionLeftID, MotorType.kBrushless);

        m_rightMotor.setPositionConversionFactor(2.0 * Math.PI);
        m_leftMotor.setPositionConversionFactor(2.0 * Math.PI);

        m_rightMotor.setSmartCurrentLimit(30);
        m_leftMotor.setSmartCurrentLimit(30);

        m_rightMotor.getEncoder().setVelocityConversionFactor(1.0 / 16.5);
        m_leftMotor.getEncoder().setVelocityConversionFactor(1.0 / 16.5);

        m_log = log;

        m_motorCurrent = new DoubleLogEntry(m_log, "/motorCurrent");

        m_motorVel = new DoubleLogEntry(m_log, "/motorVel");

        m_moduleExtensionLog = new DoubleLogEntry(m_log, "/moduleExtension");

        m_expectedSpeed = new DoubleLogEntry(m_log, "/expectedSpeed");
        m_expectedExtension = new DoubleLogEntry(m_log, "/expectedExtension");
    }

    public void periodic() {
        m_speed = m_rightMotor.get() / 2.0 + m_leftMotor.get() / 2.0;

        m_extension = getExtension();

        m_motorCurrent.append(m_rightMotor.getOutputCurrent());

        m_motorVel.append(m_speed);

        m_moduleExtensionLog.append(m_extension);

        m_expectedSpeed.append(m_desiredSpeed);

        m_expectedExtension.append(m_desiredExtension);

        m_voltageLeft = m_ff.calculate(m_desired) + m_fb.calculate(m_speed, m_desiredSpeed);
        m_voltageRight = m_voltageLeft;

        setVoltage();

    }

    public void simulate() {
        double vel = m_motorSim.calculateX(VecBuilder.fill(m_speed), VecBuilder.fill(m_voltageLeft), 0.02).get(0, 0);

        m_rightMotor.set(vel);
        m_leftMotor.set(vel);

        m_rightMotor.setPosition(m_rightMotor.getPosition() + vel * 0.02);
        m_leftMotor.setPosition(m_leftMotor.getPosition() + vel * 0.02);

        SmartDashboard.putNumber("Simulated Telescope Motor Velocity",
                m_rightMotor.get() / 2 + m_leftMotor.get() / 2);

        SmartDashboard.putNumber("Simulated Telescope Motor Position",
                m_rightMotor.getPosition() / 2 + m_leftMotor.getPosition() / 2);

        SmartDashboard.putNumber("Simulated Extension", getExtension());
    }

    public void setVelocity(double desiredSpeed) {
        m_desiredExtension += desiredSpeed * 0.02;

        m_desired = desiredSpeed;
    }

    public void setExtension(double extension) {
        if (m_desiredExtension > TelescopeConstants.kUpperBound) {
            m_desiredExtension = TelescopeConstants.kUpperBound;
        } else if (m_desiredExtension < TelescopeConstants.kLowerBound) {
            m_desiredExtension = TelescopeConstants.kLowerBound;
        } else {
            m_desiredExtension = extension;
        }

        m_speed = m_rightMotor.get() / 2 + m_leftMotor.get() / 2;

        m_extension = getExtension();

        setVoltage();
    }

    public double getExtension() {
        return m_rightMotor.getPosition() / 2.0 + m_leftMotor.getPosition() / 2.0;
    }

    private void setVoltage() {
        m_rightMotor.setVoltage(m_voltageRight);
        m_leftMotor.setVoltage(m_voltageLeft);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Telescope");

        builder.addDoubleProperty("Desired Speed", () -> {
            return m_desiredSpeed;
        }, null);

        builder.addDoubleProperty("Desired Extension (Inches)", () -> {
            return m_desiredExtension;
        }, null);

        builder.addDoubleProperty("Speed", () -> {
            return m_speed;
        }, null);

        builder.addDoubleProperty("Extension", () -> {
            return m_extension;
        }, null);

        builder.addDoubleProperty("Voltage", () -> {
            return m_voltageLeft;
        }, null);

    }

}
