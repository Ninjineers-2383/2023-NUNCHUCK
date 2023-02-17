package com.team2383.diffy.subsystems.PinkArm;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Constants;
import com.team2383.diffy.Robot;
import com.team2383.diffy.Constants.BottomPivotConstants;
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

public class MainPivotModule implements Sendable {
    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId.identifyVelocitySystem(
            Constants.BottomPivotConstants.kV,
            Constants.BottomPivotConstants.kA);

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

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State state = new TrapezoidProfile.State();

    private double m_currentVelocity = 0;
    private double m_prevAngle = 0;

    private double setVelocity = 0;

    private double kG = Constants.BottomPivotConstants.kG;

    public MainPivotModule(DataLog log) {
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

        m_angle = getAngleRadians();

        m_leftMotorCurrent.append(m_leftMotor.getOutputCurrent());
        m_rightMotorCurrent.append(m_rightMotor.getOutputCurrent());

        m_leftMotorVel.append(m_leftSpeed);
        m_rightMotorVel.append(m_rightSpeed);

        m_moduleAngleLog.append(m_angle);

        m_expectedAngle.append(m_desiredAngle);

        m_currentVelocity = (m_angle - m_prevAngle) / 0.02;
        m_prevAngle = m_angle;

        var fb = m_fb.calculate(m_angle, goal.position);

        m_leftVoltage = Math.min(Math.abs(fb), 3) * Math.signum(fb);

        m_rightVoltage = m_leftVoltage;

        if (Robot.isReal()) { // Am I on a planet with gravity
            m_leftVoltage += Math.sin(m_angle) * 1 * kG;
        }

        setVoltage();
    }

    public void simulate() {
        var newX = m_motorSim.calculateX(VecBuilder.fill(m_currentVelocity), VecBuilder.fill(m_leftVoltage), 0.02);

        m_leftMotor.set(newX.get(0, 0));
        m_rightMotor.set(newX.get(0, 0));

        m_bottomAngleEncoderSim
                .setDistance(m_angle / (2 * Math.PI) + (newX.get(0, 0) / (2 * Math.PI) * 0.02));

        SmartDashboard.putNumber("Simulated Left Motor Output Velocity",
                m_leftMotor.get());

        SmartDashboard.putNumber("Simulated Right Motor Output Velocity",
                m_rightMotor.get());

        SmartDashboard.putNumber("Simulated Encoder Radians", getAngleRadians());
    }

    public void setAngle(double angle, double extension) {
        if (m_desiredAngle > BottomPivotConstants.kUpperBound) {
            m_desiredAngle = BottomPivotConstants.kUpperBound;
        } else if (m_desiredAngle < BottomPivotConstants.kLowerBound) {
            m_desiredAngle = BottomPivotConstants.kLowerBound;
        } else {
            m_desiredAngle = angle;
        }

        goal = new TrapezoidProfile.State(
                m_desiredAngle, 0);

    }

    public void setVelocity(double angularVelocity, double extension) {
        m_desiredAngle += angularVelocity * 0.02;

        setAngle(m_desiredAngle, extension);
    }

    public double getAngleRadians() {
        return (-m_bottomAngleEncoder.getDistance() + 0.095) * 2 * Math.PI;
    }

    public double getAngleDegrees() {
        return (-m_bottomAngleEncoder.getDistance() + 0.095) * 360;
    }

    public void setVoltage() {
        m_leftMotor.setVoltage(m_leftVoltage);
        m_rightMotor.setVoltage(m_rightVoltage);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Bottom Pivot");

        builder.addDoubleProperty("Desired Angle (Degrees)", () -> {
            return m_desiredAngle * 360 / (2 * Math.PI);
        }, null);

        builder.addDoubleProperty("Angle", () -> {
            return m_angle;
        }, null);

        builder.addDoubleProperty("Angle Degrees", this::getAngleDegrees, null);

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
    }
}
