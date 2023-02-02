package com.team2383.diffy.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Constants.TelescopeConstants;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelescopeModule implements Sendable {
    // TODO: Comment
    private final Ninja_CANSparkMax m_rightMotor;
    private final Ninja_CANSparkMax m_leftMotor;

    private final LinearSystem<N3, N2, N3> m_telescopePlant;
    private final LinearQuadraticRegulator<N3, N2, N3> m_controller;
    private final KalmanFilter<N3, N2, N3> m_observer;
    private final LinearSystemLoop<N3, N2, N3> m_systemLoop;

    private double m_voltageLeft;
    private double m_voltageRight;

    private double m_desiredExtension;
    private double m_desiredSpeed;

    private double m_speed;
    private double m_position;
    private double m_extension;

    private final DataLog m_log;

    private final DoubleLogEntry m_motorCurrent;

    private final DoubleLogEntry m_motorVel;

    private final DoubleLogEntry m_moduleExtensionLog;

    private final DoubleLogEntry m_expectedSpeed;
    private final DoubleLogEntry m_expectedExtension;

    public TelescopeModule(DataLog log) {
        m_rightMotor = new Ninja_CANSparkMax(TelescopeConstants.kExtensionRightID, MotorType.kBrushless);
        m_leftMotor = new Ninja_CANSparkMax(TelescopeConstants.kExtensionLeftID, MotorType.kBrushless);

        m_rightMotor.setVelocityConversionFactor(2.0 * Math.PI * 60);
        m_leftMotor.setVelocityConversionFactor(2.0 * Math.PI * 60);

        m_rightMotor.setPositionConversionFactor(2.0 * Math.PI);
        m_leftMotor.setPositionConversionFactor(2.0 * Math.PI);

        m_telescopePlant = new LinearSystem<N3, N2, N3>(
                Matrix.mat(Nat.N3(), Nat.N3()).fill(
                        -TelescopeConstants.kV / TelescopeConstants.kA, 0, 0,
                        0, -TelescopeConstants.kV / TelescopeConstants.kA, 0,
                        TelescopeConstants.kRadPerSecToInchesPerSec / 2.0,
                        TelescopeConstants.kRadPerSecToInchesPerSec / 2.0, 0),
                Matrix.mat(Nat.N3(), Nat.N2()).fill(
                        1 / TelescopeConstants.kA, 0,
                        0, 1 / TelescopeConstants.kA,
                        0, 0),
                Matrix.mat(Nat.N3(), Nat.N3()).fill(
                        1, 0, 0,
                        0, 1, 0,
                        0, 0, 1),
                Matrix.mat(Nat.N3(), Nat.N2()).fill(
                        0, 0,
                        0, 0,
                        0, 0));

        m_controller = new LinearQuadraticRegulator<>(m_telescopePlant,
                VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 1), VecBuilder.fill(12, 12), 0.02);

        m_observer = new KalmanFilter<>(Nat.N3(), Nat.N3(), m_telescopePlant,
                VecBuilder.fill(0.01, 0.01, 0.01), VecBuilder.fill(0.1, 0.1, 0.1), 0.02);

        m_systemLoop = new LinearSystemLoop<>(m_telescopePlant, m_controller,
                m_observer, 12.0, 0.02);

        m_rightMotor.setSmartCurrentLimit(40);
        m_leftMotor.setSmartCurrentLimit(40);

        m_log = log;

        m_motorCurrent = new DoubleLogEntry(m_log, "/motorCurrent");

        m_motorVel = new DoubleLogEntry(m_log, "/motorVel");

        m_moduleExtensionLog = new DoubleLogEntry(m_log, "/moduleExtension");

        m_expectedSpeed = new DoubleLogEntry(m_log, "/expectedSpeed");
        m_expectedExtension = new DoubleLogEntry(m_log, "/expectedExtension");

        m_position = 0;
    }

    public void periodic() {
        m_speed = m_rightMotor.get() / 2.0 + m_leftMotor.get() / 2.0;

        m_extension = getExtension();

        m_motorCurrent.append(m_rightMotor.getOutputCurrent());

        m_motorVel.append(m_speed);

        m_moduleExtensionLog.append(m_extension);

        m_expectedSpeed.append(m_desiredSpeed);

        m_expectedExtension.append(m_desiredExtension);
    }

    public void simulate() {
        double velocity = m_systemLoop.getXHat(0);
        m_position = (m_systemLoop.getXHat(2));

        m_rightMotor.set(velocity);
        m_leftMotor.set(velocity);

        m_rightMotor.setPosition(m_position);
        m_leftMotor.setPosition(m_position);

        SmartDashboard.putNumber("Simulated Telescope Motor Velocity",
                m_rightMotor.get() / 2 + m_leftMotor.get() / 2);

        SmartDashboard.putNumber("Simulated Telescope Motor Position",
                m_rightMotor.getPosition() / 2 + m_leftMotor.getPosition() / 2);

        SmartDashboard.putNumber("Simulated Extension", getExtension());
    }

    public void setVelocity(double desiredSpeed) {
        m_desiredExtension += desiredSpeed * 0.02;

        if (m_desiredExtension > TelescopeConstants.kUpperBound
                || m_desiredExtension < TelescopeConstants.kLowerBound) {
            m_desiredExtension -= desiredSpeed * 0.02;
            desiredSpeed = 0;
        }

        m_speed = m_rightMotor.get() / 2 + m_leftMotor.get() / 2;

        m_extension = getExtension();

        m_systemLoop.setNextR(VecBuilder.fill(desiredSpeed, desiredSpeed, m_desiredExtension));

        m_systemLoop.correct(VecBuilder.fill(m_speed, m_speed, m_extension));

        m_systemLoop.predict(0.020);

        m_voltageLeft = m_systemLoop.getU(0);
        m_voltageRight = m_systemLoop.getU(1);

        setVoltage();
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

        m_systemLoop.setNextR(VecBuilder.fill(0, 0, m_desiredExtension));

        m_systemLoop.correct(VecBuilder.fill(m_speed, m_speed, m_extension));

        m_systemLoop.predict(0.020);

        m_voltageLeft = m_systemLoop.getU(0);
        m_voltageRight = m_systemLoop.getU(1);

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

        builder.addDoubleProperty("Estimated Velocity (x hat)", () -> {
            return m_systemLoop.getXHat(0) / 2 + m_systemLoop.getXHat(1) / 2;
        }, null);

        builder.addDoubleProperty("Estimated Extension (x hat)", () -> {
            return m_systemLoop.getXHat(2);
        }, null);
    }

}
