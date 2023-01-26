package com.team2383.diffy.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2383.diffy.Constants.TopPivotConstants;
import com.team2383.diffy.helpers.DoubleEncoder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TopPivotModule implements Sendable{
    //TODO: Comment
    private final WPI_TalonFX m_pivotMotor;
    private final TalonFXSimCollection m_pivotMotorSim;

    private final DoubleEncoder m_topAngleEncoder;

    private final LinearSystem<N2, N1, N2> m_topPivotPlant;
    private final LinearQuadraticRegulator<N2, N1, N2> m_controller;
    private final KalmanFilter<N2, N1, N2> m_observer;
    private final LinearSystemLoop<N2, N1, N2> m_systemLoop;

    private double m_voltage;

    private double m_desiredAngle;
    private double m_desiredSpeed;

    private double m_speed;
    private double m_angle;

    private final DataLog m_log;

    private final DoubleLogEntry m_motorCurrent;

    private final DoubleLogEntry m_motorVel;

    private final DoubleLogEntry m_moduleAngleLog;

    private final DoubleLogEntry m_expectedSpeed;
    private final DoubleLogEntry m_expectedAngle;

    public TopPivotModule(DataLog log) {
        m_pivotMotor = new WPI_TalonFX(TopPivotConstants.kMotorID);
        m_pivotMotorSim = m_pivotMotor.getSimCollection();

        m_topAngleEncoder = new DoubleEncoder(TopPivotConstants.kEncoderPortA, 
            TopPivotConstants.kEncoderPortB, TopPivotConstants.kEncoderPortAbs);

        m_topPivotPlant = new LinearSystem<N2, N1, N2>(
            Matrix.mat(Nat.N2(), Nat.N2()).fill(
                -TopPivotConstants.kV / TopPivotConstants.kA, 0,
                TopPivotConstants.kgt, 0), 
            Matrix.mat(Nat.N2(), Nat.N1()).fill(
                1 / TopPivotConstants.kA,
                0), 
            Matrix.mat(Nat.N2(), Nat.N2()).fill(
                1, 0,
                0, 1), 
            Matrix.mat(Nat.N2(), Nat.N1()).fill(
                0,
                0));

        m_controller = new LinearQuadraticRegulator<>(m_topPivotPlant, 
            VecBuilder.fill(Double.POSITIVE_INFINITY, 0.1), VecBuilder.fill(12), 0.02);
    
        m_observer = new KalmanFilter<>(Nat.N2(), Nat.N2(), m_topPivotPlant, 
            VecBuilder.fill(0.1, 0.1), VecBuilder.fill(0.1, 0.1), 0.02);
    
        m_systemLoop = new LinearSystemLoop<>(m_topPivotPlant, m_controller, 
            m_observer, 12.0, 0.02);

        m_log = log;

        m_motorCurrent = new DoubleLogEntry(m_log, "/motorCurrent");

        m_motorVel = new DoubleLogEntry(m_log, "/motorVel");

        m_moduleAngleLog = new DoubleLogEntry(m_log, "/moduleAngle");

        m_expectedSpeed = new DoubleLogEntry(m_log, "/expectedSpeed");
        m_expectedAngle = new DoubleLogEntry(m_log, "/expectedAngle");
    }

    public void periodic() {
        m_speed = sensorVelocityToRadiansPerSecond(m_pivotMotor.getSelectedSensorVelocity());
        m_angle = getAngle();

        m_motorCurrent.append(m_pivotMotor.getStatorCurrent());

        m_motorVel.append(m_speed);

        m_moduleAngleLog.append(m_angle);

        m_expectedSpeed.append(m_desiredSpeed);
        m_expectedAngle.append(m_desiredAngle);
    }

    public void simulate() {
        m_pivotMotorSim.setIntegratedSensorVelocity((int) (m_systemLoop.getXHat(0) / (2 * Math.PI) * 2048 / 10.0));

        SmartDashboard.putNumber("Simulated Top Pivot Motor Output Velocity",
        m_pivotMotor.getSelectedSensorVelocity());

        m_topAngleEncoder.simulate(new Rotation2d(m_systemLoop.getXHat(1)).getDegrees());

        SmartDashboard.putNumber("Simulated Encoder Rotation", getAngle());
    }

    public void setAngle(double desiredSpeed) {
        m_desiredAngle += desiredSpeed;

        m_speed = sensorVelocityToRadiansPerSecond(m_pivotMotor.getSelectedSensorVelocity());
        m_angle = getAngle();

        m_systemLoop.setNextR(VecBuilder.fill(m_desiredSpeed, Math.toRadians(m_desiredAngle)));

        m_systemLoop.correct(VecBuilder.fill(m_speed, Math.toRadians(m_angle)));

        m_systemLoop.predict(0.02);

        m_voltage = m_systemLoop.getU(0);

        setVoltage();
    }

    private double sensorVelocityToRadiansPerSecond(double sensorVelocity) {
        return sensorVelocity * (10.0 / 2048.0) * (2 * Math.PI);
    }

    public double getAngle() {
        return m_topAngleEncoder.get();
    }

    public void setVoltage() {
        m_pivotMotor.setVoltage(m_voltage);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Top Pivot");

        builder.addDoubleProperty("Desired Speed", () -> {
            return m_desiredSpeed;
        }, null);

        builder.addDoubleProperty("Desired Angle (Degrees)", () -> {
            return m_desiredAngle;
        }, null);

        builder.addDoubleProperty("Speed", () -> {
            return m_speed;
        }, null);

        builder.addDoubleProperty("Angle (Degrees)", () -> {
            return m_angle;
        }, null);

        builder.addDoubleProperty("Estimated Velocity (x hat)", () -> {
            return m_systemLoop.getXHat(0);
        }, null);

        builder.addDoubleProperty("Estimated Angle (x hat)", () -> {
            return Math.toDegrees(m_systemLoop.getXHat(1));
        }, null);

        builder.addDoubleProperty("Voltage", () -> {
            return m_voltage;
        }, null);
    }
}
