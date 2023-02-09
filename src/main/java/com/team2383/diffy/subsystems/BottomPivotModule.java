package com.team2383.diffy.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Constants;
import com.team2383.diffy.Constants.BottomPivotConstants;
import com.team2383.diffy.helpers.DoubleEncoder;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BottomPivotModule implements Sendable {
    // TODO: Comment

    private final Ninja_CANSparkMax m_rightMotor;
    private final Ninja_CANSparkMax m_leftMotor;

    private final DutyCycleEncoder m_bottomAngleEncoder;
    private final DutyCycleEncoderSim m_bottomAngleEncoderSim;

    private final SimpleMotorFeedforward m_ff;
    private final PIDController m_fb;

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

    private double m_currentVelocity = 0;
    private double m_prevAngle = 0;

    private double setVelocity = 0;

    public BottomPivotModule(DataLog log) {
        m_leftMotor = new Ninja_CANSparkMax(BottomPivotConstants.kBottomMotorLeftId, MotorType.kBrushless);
        m_rightMotor = new Ninja_CANSparkMax(BottomPivotConstants.kBottomMotorRightId, MotorType.kBrushless);

        m_rightMotor.setVelocityConversionFactor(2.0 * Math.PI * 60);
        m_leftMotor.setVelocityConversionFactor(2.0 * Math.PI * 60);

        m_bottomAngleEncoder = new DutyCycleEncoder(BottomPivotConstants.kEncoderPortAbs);
        m_bottomAngleEncoderSim = new DutyCycleEncoderSim(m_bottomAngleEncoder);

        m_log = log;

        m_leftMotor.setSmartCurrentLimit(40);
        m_rightMotor.setSmartCurrentLimit(40);

        m_leftMotorCurrent = new DoubleLogEntry(m_log, "/topMotorCurrent");
        m_rightMotorCurrent = new DoubleLogEntry(m_log, "/bottomMotorCurrent");

        m_leftMotorVel = new DoubleLogEntry(m_log, "/topMotorVel");
        m_rightMotorVel = new DoubleLogEntry(m_log, "/bottomMotorVel");

        m_moduleAngleLog = new DoubleLogEntry(m_log, "/moduleAngle");

        m_expectedAngle = new DoubleLogEntry(m_log, "/expectedAngle");

        m_ff = new SimpleMotorFeedforward(Constants.BottomPivotConstants.kS, Constants.BottomPivotConstants.kV,
                Constants.BottomPivotConstants.kA);
        m_fb = new PIDController(Constants.BottomPivotConstants.kP, 0, 0);
    }

    public void periodic() {
        m_leftSpeed = m_leftMotor.get();
        m_rightSpeed = m_rightMotor.get();

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
            }
            reset_counter++;
        }

        double m_currentAngle = Units.degreesToRadians(m_bottomAngleEncoder.getDistance() * 360);

        m_currentVelocity = (m_currentAngle - m_prevAngle) / 0.02;

        m_prevAngle = m_currentAngle;
    }

    public void simulate() {
        SmartDashboard.putNumber("Simulated Left Motor Output Velocity",
                m_leftMotor.get());

        SmartDashboard.putNumber("Simulated Right Motor Output Velocity",
                m_rightMotor.get());

        // m_bottomAngleEncoder.simulate(new
        // Rotation2d(m_systemLoop.getXHat(2)).getDegrees());

        SmartDashboard.putNumber("Simulated Encoder Rotation", getAngle());
    }

    public void setAngle(double angle, double extension) {
        // System.out.println("Bottom Angle: " + angle);
        if (m_desiredAngle > BottomPivotConstants.kUpperBound) {
            m_desiredAngle = BottomPivotConstants.kUpperBound;
        } else if (m_desiredAngle < BottomPivotConstants.kLowerBound) {
            m_desiredAngle = BottomPivotConstants.kLowerBound;
        } else {
            m_desiredAngle = angle;
        }

        // m_systemLoop.setNextR(VecBuilder.fill(0, 0, Math.toRadians(m_desiredAngle)));

        // m_systemLoop.correct(VecBuilder.fill(m_leftSpeed, m_rightSpeed,
        // Math.toRadians(m_angle)));

        // m_systemLoop.predict(0.020);

        // m_leftVoltage = m_systemLoop.getU(0);

        // m_leftVoltage += Math.signum(m_leftVoltage) * BottomPivotConstants.kS;

        // m_leftVoltage += Math.signum(m_leftVoltage) * ((extension / 2) -
        // BottomPivotConstants.pivotLength) *
        // BottomPivotConstants.armMass * 9.8 * Math.cos(Math.toRadians(getAngle()));

        // m_rightVoltage = m_systemLoop.getU(1);

        // m_rightVoltage += Math.signum(m_rightVoltage) * BottomPivotConstants.kS;

        // m_rightVoltage += Math.signum(m_rightVoltage) * ((extension / 2) -
        // BottomPivotConstants.pivotLength) *
        // BottomPivotConstants.armMass * 9.8 * Math.cos(Math.toRadians(getAngle()));

        setVoltage();
    }

    public void setVelocity(double angularVelocity, double extension) {
        // m_desiredAngle += angularVelocity * 0.02;

        // if (m_desiredAngle > BottomPivotConstants.kUpperBound || m_desiredAngle <
        // BottomPivotConstants.kLowerBound) {
        // m_desiredAngle -= angularVelocity * 0.02;
        // }

        m_leftVoltage = m_ff.calculate(angularVelocity) + m_fb.calculate(m_currentVelocity, setVelocity);

        // m_leftVoltage += Math.signum(m_leftVoltage) * BottomPivotConstants.kS;

        // m_leftVoltage += Math.signum(m_leftVoltage) * ((extension / 2) -
        // BottomPivotConstants.pivotLength) *
        // BottomPivotConstants.armMass * 9.8 * Math.cos(Math.toRadians(getAngle()));

        m_rightVoltage = m_leftVoltage;

        setVelocity = angularVelocity;

        // m_rightVoltage += Math.signum(m_rightVoltage) * BottomPivotConstants.kS;

        // m_rightVoltage += Math.signum(m_rightVoltage) * ((extension / 2) -
        // BottomPivotConstants.pivotLength) *
        // BottomPivotConstants.armMass * 9.8 * Math.cos(Math.toRadians(getAngle()));

        setVoltage();
    }

    public double getAngle() {
        return m_bottomAngleEncoder.getDistance() * 360;
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

        builder.addDoubleProperty("Set Velocity", () -> {
            return setVelocity;
        }, null);

        builder.addDoubleProperty("Current Velocity", () -> {
            return m_currentVelocity;
        }, null);

        builder.addDoubleProperty("PID", () -> {
            return m_fb.calculate(m_currentVelocity, setVelocity);
        }, null);
        // builder.addDoubleProperty("Estimated Module Angle (x hat)", () -> {
        // return Math.toDegrees(m_systemLoop.getXHat(2));
        // }, null);

        // builder.addDoubleProperty("Estimated Left Velocity (x hat)", () -> {
        // return m_systemLoop.getXHat(0);
        // }, null);

        // builder.addDoubleProperty("Estimated Right Velocity (x hat)", () -> {
        // return m_systemLoop.getXHat(1);
        // }, null);
    }
}
