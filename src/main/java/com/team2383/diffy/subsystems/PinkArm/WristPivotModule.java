package com.team2383.diffy.subsystems.PinkArm;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Constants;
import com.team2383.diffy.Robot;
import com.team2383.diffy.Constants.WristConstants;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristPivotModule implements Sendable {
    private final Ninja_CANSparkMax m_pivotMotor;

    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId
            .identifyVelocitySystem(Constants.WristConstants.kV, Constants.WristConstants.kA);

    private final DutyCycleEncoder m_topAngleEncoder;
    private final DutyCycleEncoderSim m_topAngleEncoderSim;

    private final SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(Constants.WristConstants.kS,
            Constants.WristConstants.kV, Constants.WristConstants.kA);
    private final PIDController m_fb = new PIDController(Constants.WristConstants.kP, 0, 0);

    private double m_voltage;

    // In radians
    private double m_desiredAngle;
    private double m_bottomAngle;
    private double m_angle;
    private double m_prevAngle = 0;

    // In radians per second
    private double m_desiredSpeed;
    private double m_currentVelocity = 0;

    private final DataLog m_log;

    private final DoubleLogEntry m_motorCurrent;

    private final DoubleLogEntry m_moduleAngleLog;

    private final DoubleLogEntry m_expectedSpeed;
    private final DoubleLogEntry m_expectedAngle;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State state = new TrapezoidProfile.State();

    double m_simVelocity = 0;

    public WristPivotModule(DataLog log) {
        m_pivotMotor = new Ninja_CANSparkMax(WristConstants.kMotorID, MotorType.kBrushless);

        m_pivotMotor.setVelocityConversionFactor(2.0 * Math.PI * 60);

        m_topAngleEncoder = new DutyCycleEncoder(Constants.WristConstants.kEncoderPortAbs);

        m_topAngleEncoderSim = new DutyCycleEncoderSim(m_topAngleEncoder);

        m_log = log;

        m_motorCurrent = new DoubleLogEntry(m_log, "/motorCurrent");

        m_moduleAngleLog = new DoubleLogEntry(m_log, "/moduleAngle");

        m_expectedSpeed = new DoubleLogEntry(m_log, "/expectedSpeed");
        m_expectedAngle = new DoubleLogEntry(m_log, "/expectedAngle");
    }

    public void periodic() {
        m_angle = getAngleRadians();

        m_motorCurrent.append(m_pivotMotor.getOutputCurrent());

        m_moduleAngleLog.append(m_angle);

        m_expectedSpeed.append(m_desiredSpeed);
        m_expectedAngle.append(m_desiredAngle);

        m_currentVelocity = (m_angle - m_prevAngle) / 0.02;
        m_prevAngle = m_angle;

        state = new TrapezoidProfile(constraints, goal, state).calculate(0.02);

        m_voltage = m_ff.calculate(state.velocity)
                + m_fb.calculate(m_angle, state.position);

        if (Robot.isReal()) {
            m_voltage += Math.sin(m_angle + m_bottomAngle) * Constants.WristConstants.kG;
        }

        setVoltage();
    }

    public void simulate() {
        var newVel = m_motorSim.calculateX(VecBuilder.fill(m_currentVelocity), VecBuilder.fill(m_voltage), 0.02).get(0,
                0);

        m_topAngleEncoderSim.setDistance(m_angle / (2 * Math.PI) + (newVel / (2 * Math.PI) * 0.02));

        SmartDashboard.putNumber("Simulated Top Pivot Motor Output Velocity",
                m_pivotMotor.get());

        SmartDashboard.putNumber("Simulated Encoder Radians", getAngleRadians());
    }

    public void setAngle(double desiredAngle, double bottomAngle) {
        m_desiredAngle = desiredAngle;

        m_bottomAngle = bottomAngle;

        goal = new TrapezoidProfile.State(
                m_desiredAngle,
                0);
    }

    public void setVelocity(double desiredSpeed, double bottomAngle) {
        m_desiredAngle += desiredSpeed * 0.02;

        if (m_desiredAngle > WristConstants.kUpperBound || m_desiredAngle < WristConstants.kLowerBound) {
            m_desiredAngle -= desiredSpeed * 0.02;
            desiredSpeed = 0;
        }

        setAngle(m_desiredAngle, bottomAngle);
    }

    public double getAngleRadians() {
        return m_topAngleEncoder.get() * 2 * Math.PI;
    }

    public double getAngleDegrees() {
        return m_topAngleEncoder.get() * 360;
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
            return m_currentVelocity;
        }, null);

        builder.addDoubleProperty("Angle (Degrees)", () -> {
            return m_angle;
        }, null);

        builder.addDoubleProperty("Voltage", () -> {
            return m_voltage;
        }, null);

        builder.addDoubleProperty("Velocity State", () -> {
            return state.velocity;
        }, null);
    }
}