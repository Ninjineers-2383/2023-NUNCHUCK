package com.team2383.diffy.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2383.diffy.Constants.TelescopeConstants;

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
    //TODO: Comment
    private final WPI_TalonFX m_telescopeMotor;

    private final TalonFXSimCollection m_telescopeMotorSim;

    private final LinearSystem<N2, N1, N2> m_telescopePlant;
    private final LinearQuadraticRegulator<N2, N1, N2> m_controller;
    private final KalmanFilter<N2, N1, N2> m_observer;
    private final LinearSystemLoop<N2, N1, N2> m_systemLoop;

    private double m_voltage;

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
        m_telescopeMotor = new WPI_TalonFX(TelescopeConstants.kExtensionLeftID);

        m_telescopeMotorSim = m_telescopeMotor.getSimCollection();

        m_telescopePlant = new LinearSystem<N2, N1, N2>(
            Matrix.mat(Nat.N2(), Nat.N2()).fill(
                -TelescopeConstants.kV / TelescopeConstants.kA, 0,
                TelescopeConstants.kge, 0), 
            Matrix.mat(Nat.N2(), Nat.N1()).fill(
                1 / TelescopeConstants.kA,
                0),
            Matrix.mat(Nat.N2(), Nat.N2()).fill(
                1, 0,
                0, 1), 
            Matrix.mat(Nat.N2(), Nat.N1()).fill(
                0,
                0)
        );
        
        m_controller = new LinearQuadraticRegulator<>(m_telescopePlant, 
            VecBuilder.fill(Double.POSITIVE_INFINITY, 1), VecBuilder.fill(12), 0.02);

        m_observer = new KalmanFilter<>(Nat.N2(), Nat.N2(), m_telescopePlant, 
            VecBuilder.fill(0.01, 0.01), VecBuilder.fill(0.1, 0.1), 0.02);

        m_systemLoop = new LinearSystemLoop<>(m_telescopePlant, m_controller, 
            m_observer, 12.0, 0.02);

        SupplyCurrentLimitConfiguration supply = new SupplyCurrentLimitConfiguration(
            true,
            TelescopeConstants.kMaxCurrent,
            TelescopeConstants.kMaxCurrent, 10);
    
        m_telescopeMotor.configSupplyCurrentLimit(supply);

        m_log = log;

        m_motorCurrent = new DoubleLogEntry(m_log, "/motorCurrent");

        m_motorVel = new DoubleLogEntry(m_log, "/motorVel");

        m_moduleExtensionLog = new DoubleLogEntry(m_log, "/moduleExtension");

        m_expectedSpeed = new DoubleLogEntry(m_log, "/expectedSpeed");
        m_expectedExtension = new DoubleLogEntry(m_log, "/expectedExtension");

        m_position = 0;
    }

    public void periodic() {
        m_speed = sensorVelocityToRadiansPerSecond(m_telescopeMotor.getSelectedSensorVelocity());

        m_extension = getExtension();

        m_motorCurrent.append(m_telescopeMotor.getStatorCurrent());

        m_motorVel.append(m_speed);

        m_moduleExtensionLog.append(m_extension);

        m_expectedSpeed.append(m_desiredSpeed);
        m_expectedExtension.append(m_desiredExtension);
    }

    public void simulate() {
        double velocity = (m_systemLoop.getXHat(0) / (2 * Math.PI)) * 2048 / 10.0;
        m_position += velocity * 0.02 * 10.0;

        m_telescopeMotorSim.setIntegratedSensorVelocity((int) ((m_systemLoop.getXHat(0) / (2 * Math.PI)) * 2048 / 10.0));
        m_telescopeMotorSim.setIntegratedSensorRawPosition((int) m_position);

        SmartDashboard.putNumber("Simulated Telescope Motor Velocity", 
            m_telescopeMotor.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Simulated Telescope Motor Position", 
            m_telescopeMotor.getSelectedSensorPosition());

        SmartDashboard.putNumber("Simulated Extension", getExtension());
    }

    public void setExtension(double desiredSpeed) {
        m_desiredExtension += desiredSpeed;

        m_speed = sensorVelocityToRadiansPerSecond(m_telescopeMotor.getSelectedSensorVelocity());

        m_extension = getExtension();

        m_systemLoop.setNextR(VecBuilder.fill(0, m_desiredExtension));

        m_systemLoop.correct(VecBuilder.fill(m_speed, m_extension));

        m_systemLoop.predict(0.020);

        m_voltage = m_systemLoop.getU(0);

        setVoltage();
    }

    public double getExtension() {
        return sensorPositionToRadians(m_telescopeMotor.getSelectedSensorPosition());
    }

    private void setVoltage() {
        m_telescopeMotor.setVoltage(m_voltage);
    }

    private double sensorVelocityToRadiansPerSecond(double sensorVelocity) {
        return sensorVelocity * (10.0 / 2048.0) * (2 * Math.PI);
    }

    private double sensorPositionToRadians(double sensorPosition) {
        return (sensorPosition * 2 * Math.PI) / 2048.0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Telescope");

        builder.addDoubleProperty("Desired Speed", () -> {
            return m_desiredSpeed;
        }, null);

        builder.addDoubleProperty("Desired Extension (Pick a Unit)", () -> {
            return m_desiredExtension;
        }, null);

        builder.addDoubleProperty("Speed", () -> {
            return m_speed;
        }, null);

        builder.addDoubleProperty("Extension", () -> {
            return m_extension;
        }, null);

        builder.addDoubleProperty("Voltage", () -> {
            return m_voltage;
        }, null);

        builder.addDoubleProperty("Estimated Velocity (x hat)", () -> {
            return m_systemLoop.getXHat(0);
        }, null);

        builder.addDoubleProperty("Estimated Extension (x hat)", () -> {
            return m_systemLoop.getXHat(1);
        }, null);
    }
    
}
