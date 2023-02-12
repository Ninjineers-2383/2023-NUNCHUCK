package com.team2383.diffy.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Constants;
import com.team2383.diffy.Robot;
import com.team2383.diffy.Constants.WristConstants;
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
import edu.wpi.first.math.system.plant.LinearSystemId;
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

    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId
            .identifyVelocitySystem(Constants.WristConstants.kV, Constants.WristConstants.kA);

    private final DutyCycleEncoder m_topAngleEncoder;
    private final DutyCycleEncoderSim m_topAngleEncoderSim;

    private final SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(Constants.WristConstants.kS,
            Constants.WristConstants.kV, Constants.WristConstants.kA);
    private final PIDController m_fb = new PIDController(Constants.WristConstants.kP, 0, 0);

    private double m_voltage;

    private double m_desiredAngle;
    private double m_desiredSpeed;

    private double m_bottomAngle;

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

    private double m_velocitySetpoint = 0;

    public TopPivotModule(DataLog log) {
        m_pivotMotor = new Ninja_CANSparkMax(WristConstants.kMotorID, MotorType.kBrushless);

        m_pivotMotor.setVelocityConversionFactor(2.0 * Math.PI * 60);

        m_topAngleEncoder = new DutyCycleEncoder(Constants.WristConstants.kEncoderPortAbs);

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

        m_voltage = m_ff.calculate(m_velocitySetpoint) + m_fb.calculate(m_currentVelocity, m_velocitySetpoint);

        if (Robot.isReal()) {
            m_voltage += Math.sin(m_currentAngle + m_bottomAngle) * Constants.WristConstants.kG;
        }

        setVoltage();

    }

    public void simulate() {
        var newX = m_motorSim.calculateX(VecBuilder.fill(m_currentVelocity), VecBuilder.fill(m_voltage), 0.02);

        m_topAngleEncoderSim.setDistance(m_angle + (newX.get(0, 0) / (2 * Math.PI) * 0.02));

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

    }

    public void setVelocity(double desiredSpeed, double bottomAngle) {
        m_desiredAngle += desiredSpeed * 0.02;

        m_bottomAngle = bottomAngle;

        if (m_desiredAngle > WristConstants.kUpperBound || m_desiredAngle < WristConstants.kLowerBound) {
            m_desiredAngle -= desiredSpeed * 0.02;
            desiredSpeed = 0;
        }

        m_velocitySetpoint = desiredSpeed;

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
