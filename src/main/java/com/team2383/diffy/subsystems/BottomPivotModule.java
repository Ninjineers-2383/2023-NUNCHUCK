package com.team2383.diffy.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Robot;
import com.team2383.diffy.Constants.BottomPivotConstants;
import com.team2383.diffy.helpers.DoubleEncoder;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;
import com.team2383.diffy.helpers.SparkMaxSimCollection;

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
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BottomPivotModule implements Sendable {
    // TODO: Comment
    // private final WPI_TalonFX m_rightMotor;
    // private final WPI_TalonFX m_leftMotor;

    private final CANSparkMax m_rightMotor;
    private final CANSparkMax m_leftMotor;

    private final SparkMaxSimCollection m_leftMotorSim;
    private final SparkMaxSimCollection m_rightMotorSim;

    // private final TalonFXSimCollection m_leftMotorSim;
    // private final TalonFXSimCollection m_rightMotorSim;

    private final DoubleEncoder m_bottomAngleEncoder;

    private final LinearSystemLoop<N3, N2, N3> m_systemLoop;

    private double m_leftVoltage;
    private double m_rightVoltage;

    private double m_desiredAngle;

    private double m_leftSpeed;
    private double m_rightSpeed;
    private double m_angle;

    private final DataLog m_log;

    private final DoubleLogEntry m_leftMotorCurrent;
    private final DoubleLogEntry m_rightMotorCurrent;

    private final DoubleLogEntry m_leftMotorVel;
    private final DoubleLogEntry m_rightMotorVel;

    private final DoubleLogEntry m_moduleAngleLog;

    private final DoubleLogEntry m_expectedAngle;

    private int reset_counter = 0;

    public BottomPivotModule(DataLog log) {
        m_leftMotor = new CANSparkMax(BottomPivotConstants.kBottomMotorLeftId, MotorType.kBrushless);
 
        m_leftMotorSim = new SparkMaxSimCollection(m_leftMotor);

        m_rightMotor = new CANSparkMax(BottomPivotConstants.kBottomMotorRightId, MotorType.kBrushless);
        m_rightMotorSim = new SparkMaxSimCollection(m_rightMotor);

        m_bottomAngleEncoder = new DoubleEncoder(BottomPivotConstants.kEncoderPortA,
                BottomPivotConstants.kEncoderPortB, BottomPivotConstants.kEncoderPortAbs);

        m_log = log;

        // SupplyCurrentLimitConfiguration supply = new SupplyCurrentLimitConfiguration(
        //         true,
        //         BottomPivotConstants.kMaxCurrent,
        //         BottomPivotConstants.kMaxCurrent, 10);

        m_leftMotor.setSmartCurrentLimit(40);
        m_rightMotor.setSmartCurrentLimit(40);

        m_leftMotorCurrent = new DoubleLogEntry(m_log, "/topMotorCurrent");
        m_rightMotorCurrent = new DoubleLogEntry(m_log, "/bottomMotorCurrent");

        m_leftMotorVel = new DoubleLogEntry(m_log, "/topMotorVel");
        m_rightMotorVel = new DoubleLogEntry(m_log, "/bottomMotorVel");

        m_moduleAngleLog = new DoubleLogEntry(m_log, "/moduleAngle");

        m_expectedAngle = new DoubleLogEntry(m_log, "/expectedAngle");

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

        LinearQuadraticRegulator<N3, N2, N3> m_controller = new LinearQuadraticRegulator<>(m_bottomPivotPlant,
                VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.001), VecBuilder.fill(12, 12), 0.02);

        KalmanFilter<N3, N2, N3> m_observer = new KalmanFilter<>(Nat.N3(), Nat.N3(), m_bottomPivotPlant,
                VecBuilder.fill(0.01, 0.01, 0.01), VecBuilder.fill(0.01, 0.01, 0.01), 0.02);

        m_systemLoop = new LinearSystemLoop<>(m_bottomPivotPlant, m_controller,
                m_observer, 12.0, 0.02);
    }

    public void periodic() {
        m_leftSpeed = sensorVelocityToRadiansPerSecond(m_leftMotor.get());
        m_rightSpeed = sensorVelocityToRadiansPerSecond(m_rightMotor.get());

        m_angle = getAngle();

        m_leftMotorCurrent.append(m_leftMotor.getOutputCurrent());
        m_rightMotorCurrent.append(m_rightMotor.getOutputCurrent());

        m_leftMotorVel.append(m_leftSpeed);
        m_rightMotorVel.append(m_rightSpeed);

        m_moduleAngleLog.append(m_angle);

        m_expectedAngle.append(m_desiredAngle);

        if (reset_counter < 200) {
            if (reset_counter == 199) {
                m_bottomAngleEncoder.reset();
                m_systemLoop.setXHat(VecBuilder.fill(0, 0, Math.toRadians(getAngle())));
            }
            reset_counter++;
        }
    }

    public void simulate() {
        m_leftMotorSim.setVelocity((int) ((m_systemLoop.getXHat(0) / (2 * Math.PI)) * 2048 / 10.0));
        m_rightMotorSim.setVelocity((int) ((m_systemLoop.getXHat(1) / (2 * Math.PI)) * 2048 / 10.0));

        SmartDashboard.putNumber("Simulated Left Motor Output Velocity",
                m_leftMotor.get());

        SmartDashboard.putNumber("Simulated Right Motor Output Velocity",
                m_rightMotor.get());

        m_bottomAngleEncoder.simulate(new Rotation2d(m_systemLoop.getXHat(2)).getDegrees());

        SmartDashboard.putNumber("Simulated Encoder Rotation", getAngle());
    }

    public void setAngle(double angularVelocity, double extension) {
        m_desiredAngle += angularVelocity;

        m_systemLoop.setNextR(VecBuilder.fill(0, 0, Math.toRadians(m_desiredAngle)));

        m_systemLoop.correct(VecBuilder.fill(m_leftSpeed, m_rightSpeed, Math.toRadians(m_angle)));

        m_systemLoop.predict(0.020);

        m_leftVoltage = m_systemLoop.getU(0);

        m_leftVoltage += Math.signum(m_leftVoltage) * BottomPivotConstants.kS;
        
        m_leftVoltage += Math.signum(m_leftVoltage) * ((extension / 2) - BottomPivotConstants.pivotLength) * 
            BottomPivotConstants.armMass * 9.8 * Math.cos(Math.toRadians(getAngle()));

        m_rightVoltage = m_systemLoop.getU(1);

        m_rightVoltage += Math.signum(m_rightVoltage) * BottomPivotConstants.kS;

        m_rightVoltage += Math.signum(m_rightVoltage) * ((extension / 2) - BottomPivotConstants.pivotLength) * 
            BottomPivotConstants.armMass * 9.8 * Math.cos(Math.toRadians(getAngle()));

        setVoltage();
    }

    public double getAngle() {
        return m_bottomAngleEncoder.get();
    }

    private double sensorVelocityToRadiansPerSecond(double sensorVelocity) {
        return sensorVelocity * (10.0 / 2048.0) * (2 * Math.PI);
    }

    public void setVoltage() {
        m_leftMotor.setVoltage(m_leftVoltage);
        m_rightMotor.setVoltage(m_rightVoltage);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Bottom Pivot");

        builder.addDoubleProperty("Desired Angle (Degrees)", () -> {
            return m_desiredAngle;
        }, null);

        builder.addDoubleProperty("Angle", () -> {
            return m_angle;
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

        builder.addDoubleProperty("Estimated Module Angle (x hat)", () -> {
            return Math.toDegrees(m_systemLoop.getXHat(2));
        }, null);

        builder.addDoubleProperty("Estimated Left Velocity (x hat)", () -> {
            return m_systemLoop.getXHat(0);
        }, null);

        builder.addDoubleProperty("Estimated Right Velocity (x hat)", () -> {
            return m_systemLoop.getXHat(1);
        }, null);
    }
}
