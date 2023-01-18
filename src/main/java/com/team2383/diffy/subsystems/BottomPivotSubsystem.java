package com.team2383.diffy.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2383.diffy.Constants.BottomPivotConstants;
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

public class BottomPivotSubsystem implements Sendable {
    // TODO: Comment
    private final WPI_TalonFX m_leftMotor;
    private final TalonFXSimCollection m_leftMotorSim;

    private final WPI_TalonFX m_rightMotor;
    private final TalonFXSimCollection m_rightMotorSim;

    private final DoubleEncoder m_bottomAngleEncoder;

    private final LinearSystemLoop<N3, N2, N3> m_systemLoop;

    private double m_leftVoltage;
    private double m_rightVoltage;

    private double m_desiredAngle;
    private double m_desiredSpeed;

    private double m_leftSpeed;
    private double m_rightSpeed;
    private double m_angle;

    private final DataLog m_log;

    private final DoubleLogEntry m_leftMotorCurrent;
    private final DoubleLogEntry m_rightMotorCurrent;

    private final DoubleLogEntry m_leftMotorVel;
    private final DoubleLogEntry m_rightMotorVel;

    private final DoubleLogEntry m_moduleAngleLog;

    private final DoubleLogEntry m_expectedSpeed;
    private final DoubleLogEntry m_expectedAngle;

    public BottomPivotSubsystem(DataLog log) {
        m_leftMotor = new WPI_TalonFX(BottomPivotConstants.kBottomMotorLeftId);
        m_leftMotorSim = new TalonFXSimCollection(m_leftMotor);

        m_rightMotor = new WPI_TalonFX(BottomPivotConstants.kBottomMotorRightId);
        m_rightMotorSim = new TalonFXSimCollection(m_rightMotor);

        m_bottomAngleEncoder = new DoubleEncoder(BottomPivotConstants.kEncoderPortA,
                BottomPivotConstants.kEncoderPortB, BottomPivotConstants.kEncoderPortAbs);

        LinearSystem<N3, N2, N3> m_bottomPivotPlant = new LinearSystem<>(
                Matrix.mat(Nat.N3(), Nat.N3()).fill(
                        -BottomPivotConstants.kV / BottomPivotConstants.kA, 0, 0,
                        0, -BottomPivotConstants.kV / BottomPivotConstants.kA, 0,
                        BottomPivotConstants.kgb / 2, BottomPivotConstants.kgb / 2, 0),

                Matrix.mat(Nat.N3(), Nat.N2()).fill(
                        1 / BottomPivotConstants.kA, 0,
                        0, 1 / BottomPivotConstants.kA,
                        0, 0),

                Matrix.mat(Nat.N3(), Nat.N3()).fill(
                        1, 0, 0,
                        0, 1, 0,
                        0, 0, 1),

                Matrix.mat(Nat.N3(), Nat.N2()).fill(
                        0, 0,
                        0, 0,
                        0, 0));

        m_log = log;

        LinearQuadraticRegulator<N3, N2, N3> m_controller = new LinearQuadraticRegulator<>(m_bottomPivotPlant,
                VecBuilder.fill(1, 1, 0.001), VecBuilder.fill(12, 12), 0.02);

        KalmanFilter<N3, N2, N3> m_observer = new KalmanFilter<>(Nat.N3(), Nat.N3(), m_bottomPivotPlant,
                VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.01, 0.01, 0.01), 0.02);

        m_systemLoop = new LinearSystemLoop<>(m_bottomPivotPlant, m_controller,
                m_observer, 12.0, 0.02);

        SupplyCurrentLimitConfiguration supply = new SupplyCurrentLimitConfiguration(
                true,
                BottomPivotConstants.kMaxCurrent,
                BottomPivotConstants.kMaxCurrent, 10);

        m_leftMotor.configSupplyCurrentLimit(supply);
        m_rightMotor.configSupplyCurrentLimit(supply);

        m_leftMotorCurrent = new DoubleLogEntry(m_log, "/topMotorCurrent");
        m_rightMotorCurrent = new DoubleLogEntry(m_log, "/bottomMotorCurrent");

        m_leftMotorVel = new DoubleLogEntry(m_log, "/topMotorVel");
        m_rightMotorVel = new DoubleLogEntry(m_log, "/bottomMotorVel");

        m_moduleAngleLog = new DoubleLogEntry(m_log, "/moduleAngle");

        m_expectedSpeed = new DoubleLogEntry(m_log, "/expectedSpeed");
        m_expectedAngle = new DoubleLogEntry(m_log, "/expectedAngle");

    }

    public void periodic() {
        m_leftMotorCurrent.append(m_leftMotor.getStatorCurrent());
        m_rightMotorCurrent.append(m_rightMotor.getStatorCurrent());

        m_leftMotorVel.append(m_leftSpeed);
        m_rightMotorVel.append(m_rightSpeed);

        m_moduleAngleLog.append(m_angle);

        m_expectedSpeed.append(m_desiredSpeed);
        m_expectedAngle.append(m_desiredAngle);
    }

    public void simulate() {
        m_leftMotorSim.setIntegratedSensorVelocity((int) radiansPerSecondToSensorVelocity(m_systemLoop.getXHat(0)));
        m_rightMotorSim.setIntegratedSensorVelocity((int) radiansPerSecondToSensorVelocity(m_systemLoop.getXHat(1)));

        SmartDashboard.putNumber("Simulated Left Motor Output Velocity",
                m_leftMotor.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Simulated Right Motor Output Velocity",
                m_rightMotor.getSelectedSensorVelocity());

        m_bottomAngleEncoder.simulate(new Rotation2d(m_systemLoop.getXHat(2)).getDegrees());

        SmartDashboard.putNumber("Simulated Encoder Rotation", getAngle());
    }

    public void setAngle(double desiredAngle, double desiredSpeed, double extension, double pivotAngle) {
        m_desiredAngle = desiredAngle;
        m_desiredSpeed = desiredSpeed;

        m_leftSpeed = sensorVelocityToRadiansPerSecond(m_leftMotor.getSelectedSensorVelocity());
        m_rightSpeed = sensorVelocityToRadiansPerSecond(m_rightMotor.getSelectedSensorVelocity());
        m_angle = getAngle();

        // TODO Reverse motors if necessary
        m_systemLoop.setNextR(VecBuilder.fill(desiredSpeed, desiredSpeed, Math.toRadians(desiredAngle)));

        m_systemLoop.correct(VecBuilder.fill(m_leftSpeed, m_rightSpeed, Math.toRadians(m_angle)));

        m_systemLoop.predict(0.020);

        m_leftVoltage = m_systemLoop.getU(0);

        m_leftVoltage += Math.signum(m_leftVoltage) * BottomPivotConstants.kS;

        m_leftVoltage += Math.signum(m_leftVoltage) * ((extension / 2) - BottomPivotConstants.pivotLength) * 
            BottomPivotConstants.armMass * 9.8 * Math.cos(Math.toRadians(pivotAngle));

        m_rightVoltage = m_systemLoop.getU(1);

        m_rightVoltage += Math.signum(m_rightVoltage) * BottomPivotConstants.kS;

        m_rightVoltage += Math.signum(m_rightVoltage) * ((extension / 2) - BottomPivotConstants.pivotLength) * 
            BottomPivotConstants.armMass * 9.8 * Math.cos(Math.toRadians(pivotAngle));

        setVoltage();
    }

    public double getAngle() {
        return m_bottomAngleEncoder.get();
    }

    private double sensorVelocityToRadiansPerSecond(double sensorVelocity) {
        return sensorVelocity * (10.0 / 2048.0) * (2 * Math.PI);
    }

    private double radiansPerSecondToSensorVelocity(double radiansPerSecond) {
        return radiansPerSecond / ((2 * Math.PI * 2048) / 10.0);
    }

    public void setVoltage() {
        m_leftMotor.setVoltage(m_leftVoltage);
        m_rightMotor.setVoltage(m_rightVoltage);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Bottom Pivot");

        builder.addDoubleProperty("Desired Speed", () -> {
            return m_desiredSpeed;
        }, null);

        builder.addDoubleProperty("Desired Angle (Degrees)", () -> {
            return m_desiredAngle;
        }, null);

        builder.addDoubleProperty("Left Speed", () -> {
            return m_leftSpeed;
        }, null);

        builder.addDoubleProperty("Right Speed", () -> {
            return m_rightSpeed;
        }, null);

        builder.addDoubleProperty("Left Voltage", () -> {
            return m_leftVoltage;
        }, null);
        builder.addDoubleProperty("Right Voltage", () -> {
            return m_rightVoltage;
        }, null);
    }
}
