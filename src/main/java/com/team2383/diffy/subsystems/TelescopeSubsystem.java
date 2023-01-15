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

public class TelescopeSubsystem implements Sendable {
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

    private final DataLog m_log;

    private final DoubleLogEntry m_motorCurrent;

    private final DoubleLogEntry m_motorVel;

    private final DoubleLogEntry m_moduleExtensionLog;

    private final DoubleLogEntry m_expectedSpeed;
    private final DoubleLogEntry m_expectedExtension;

    public TelescopeSubsystem(DataLog log) {
        m_telescopeMotor = new WPI_TalonFX(TelescopeConstants.kExtensionID);

        m_telescopeMotorSim = new TalonFXSimCollection(m_telescopeMotor);

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
            VecBuilder.fill(1, 1), VecBuilder.fill(12), 0.02);

        m_observer = new KalmanFilter<>(Nat.N2(), Nat.N2(), m_telescopePlant, 
            VecBuilder.fill(0.1, 0.1), VecBuilder.fill(0.1, 0.1), 0.02);

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
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        
    }
    
}
