package com.team2383.diffy.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Constants;
import com.team2383.diffy.Constants.TopPivotConstants;
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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TopPivotModule implements Sendable {
    // TODO: Comment
    private final Ninja_CANSparkMax m_pivotMotor;

    private final DutyCycleEncoder m_topAngleEncoder;
    private final DutyCycleEncoderSim m_topAngleEncoderSim;

    private final SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(Constants.TopPivotConstants.kS,
            Constants.TopPivotConstants.kV, Constants.TopPivotConstants.kA);
    private final PIDController m_fb = new PIDController(Constants.TopPivotConstants.kP, 0, 0);

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

    private double m_prevAngle = 0;
    private double m_currentVelocity = 0;

    public TopPivotModule(DataLog log) {
        m_pivotMotor = new Ninja_CANSparkMax(TopPivotConstants.kMotorID, MotorType.kBrushless);

        m_pivotMotor.setVelocityConversionFactor(2.0 * Math.PI * 60);

        m_topAngleEncoder = new DutyCycleEncoder(Constants.TopPivotConstants.kEncoderPortAbs);

        m_topAngleEncoderSim = new DutyCycleEncoderSim(m_topAngleEncoder);

        m_log = log;

        m_motorCurrent = new DoubleLogEntry(m_log, "/motorCurrent");

        m_motorVel = new DoubleLogEntry(m_log, "/motorVel");

        m_moduleAngleLog = new DoubleLogEntry(m_log, "/moduleAngle");

        m_expectedSpeed = new DoubleLogEntry(m_log, "/expectedSpeed");
        m_expectedAngle = new DoubleLogEntry(m_log, "/expectedAngle");
    }

    public void periodic() {
        m_speed = m_pivotMotor.get();
        m_angle = getAngle();

        m_motorCurrent.append(m_pivotMotor.getOutputCurrent());

        m_motorVel.append(m_speed);

        m_moduleAngleLog.append(m_angle);

        m_expectedSpeed.append(m_desiredSpeed);
        m_expectedAngle.append(m_desiredAngle);

        double m_currentAngle = Units.degreesToRadians(m_topAngleEncoder.get() * 360);
        m_currentVelocity = (m_currentAngle - m_prevAngle) / 0.02;
        m_prevAngle = m_currentAngle;
    }

    public void simulate() {
        SmartDashboard.putNumber("Simulated Top Pivot Motor Output Velocity",
                m_pivotMotor.get());

        SmartDashboard.putNumber("Simulated Encoder Rotation", getAngle());
    }

    public void setAngle(double desiredAngle) {
        // System.out.println("Top Angle: " + desiredAngle);
        m_desiredAngle = desiredAngle;

        // if (m_desiredAngle > TopPivotConstants.kUpperBound || m_desiredAngle <
        // TopPivotConstants.kLowerBound) {
        // m_desiredAngle = m_desiredAngle > TopPivotConstants.kUpperBound ?
        // TopPivotConstants.kUpperBound
        // : TopPivotConstants.kLowerBound;
        // }

        setVoltage();
    }

    public void setVelocity(double desiredSpeed) {
        m_desiredAngle += desiredSpeed * 0.02;

        if (m_desiredAngle > TopPivotConstants.kUpperBound || m_desiredAngle < TopPivotConstants.kLowerBound) {
            m_desiredAngle -= desiredSpeed * 0.02;
            desiredSpeed = 0;
        }

        m_voltage = m_ff.calculate(desiredSpeed) + m_fb.calculate(m_currentVelocity, desiredSpeed);

        setVoltage();
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

        // builder.addDoubleProperty("Estimated Velocity (x hat)", () -> {
        // return m_systemLoop.getXHat(0);
        // }, null);

        // builder.addDoubleProperty("Estimated Angle (x hat)", () -> {
        // return Math.toDegrees(m_systemLoop.getXHat(1));
        // }, null);

        builder.addDoubleProperty("Voltage", () -> {
            return m_voltage;
        }, null);
    }
}
