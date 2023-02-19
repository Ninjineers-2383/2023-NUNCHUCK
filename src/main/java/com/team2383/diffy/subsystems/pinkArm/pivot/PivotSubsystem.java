package com.team2383.diffy.subsystems.pinkArm.pivot;


import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Robot;
import com.team2383.diffy.helpers.Clip;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PivotSubsystem extends SubsystemBase {
    private final Ninja_CANSparkMax m_rightMotor;
    private final Ninja_CANSparkMax m_leftMotor;

    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId.identifyVelocitySystem(
            PivotConstants.FEEDFORWARD_CONTROLLER.kv,
            PivotConstants.FEEDFORWARD_CONTROLLER.ka);

    private final DutyCycleEncoder m_absEncoder;
    private final DutyCycleEncoderSim m_absEncoderSim;


    private double m_voltage;

    private TrapezoidProfile.State m_targetState;

    private double m_velocity;

    private double m_prevTime;

    private double m_prevAngle = 0;

    private TrapezoidProfile m_profile;

    private Timer m_timer;

    public PivotSubsystem() {
        m_leftMotor = new Ninja_CANSparkMax(PivotConstants.BOTTOM_MOTOR_LEFT_ID, MotorType.kBrushless);
        m_rightMotor = new Ninja_CANSparkMax(PivotConstants.BOTTOM_MOTOR_RIGHT_ID, MotorType.kBrushless);

        m_rightMotor.setVelocityConversionFactor(PivotConstants.VELOCITY_CONVERSION_FACTOR.getRadians());
        m_leftMotor.setVelocityConversionFactor(PivotConstants.VELOCITY_CONVERSION_FACTOR.getRadians());

        m_absEncoder = new DutyCycleEncoder(PivotConstants.ABS_ENCODER_PORT);
        m_absEncoderSim = new DutyCycleEncoderSim(m_absEncoder);

        m_leftMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);
        m_rightMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);


        m_absEncoder.setPositionOffset(PivotConstants.ENCODER_OFFSET);
        m_timer = new Timer();
        m_timer.stop();
        m_prevTime = Timer.getFPGATimestamp();
    }

    public void periodic() {
        // calculate dT for discrete-time derivitives 
        double deltaTime = Timer.getFPGATimestamp() - m_prevTime;

        // If the position is reached, reset all of the profiling
        if (isAtPosition()) {
            m_targetState = null;
            m_profile = null;
        }

        
        double angle = getAngle().getRadians();
        m_velocity = (angle - m_prevAngle) / deltaTime; // Discrete-time derivitive

        // Set target state
        if (m_profile != null) {
            m_targetState = m_profile.calculate(m_timer.get());
        }
        // set desired velocity for PIDF Control
        double desiredVelocity = m_targetState != null ? m_targetState.velocity : 0;
        m_voltage = PivotConstants.PID_CONTROLLER.calculate(m_velocity, desiredVelocity);
        m_voltage += PivotConstants.FEEDFORWARD_CONTROLLER.calculate(m_velocity, desiredVelocity, deltaTime);
        if (Robot.isReal()) { // Am I on a planet with gravity
            m_voltage += Math.sin(angle) * 1 * PivotConstants.kG;
        }

        m_leftMotor.setVoltage(m_voltage);
        m_rightMotor.setVoltage(m_voltage);
        // Set Discrete-time variables
        m_prevAngle = angle;
        m_prevTime = Timer.getFPGATimestamp();
    }

    @Override
    public void simulationPeriodic() {
        var newX = m_motorSim.calculateX(VecBuilder.fill(m_targetState.velocity), VecBuilder.fill(m_voltage), 0.02);

        m_leftMotor.set(newX.get(0, 0));
        m_rightMotor.set(newX.get(0, 0));

        m_absEncoderSim
                .setDistance(getAngle().getRotations() + (newX.get(0, 0) / (2 * Math.PI) * 0.02));
    }

    /**
     * Uses trapezoidal motion-profiling to implement pseudo-positional control
     * @return boolean state to determine whether the input angle is safe
     */
    public boolean setAngle(Rotation2d angle, double extension) {
        // safety for upper bounds
        double adjustedAngle = Clip.clip(PivotConstants.LOWER_BOUND.getRadians(), angle.getRadians(), PivotConstants.UPPER_BOUND.getRadians());

        // safety for inside robot
        adjustedAngle = extension < PivotConstants.EXTENSION_SAFETY ? adjustedAngle : 
            Clip.invClip(PivotConstants.LOWER_SAFETY.getRadians(), adjustedAngle, PivotConstants.UPPER_SAFETY.getRadians());

        // generate profile based on current state
        m_profile = new TrapezoidProfile(
                PivotConstants.TRAPEZOIDAL_CONSTRAINTS,
                new TrapezoidProfile.State(getAngle().getRadians(), getVelocity().getRadians()),
                new TrapezoidProfile.State(adjustedAngle, 0.0));

        // reset profile clock
        m_timer.reset();
        return adjustedAngle != angle.getRadians();
    }

    /**
     * Gets velocity of the abs encoder
     * @return
     */
    public Rotation2d getVelocity() {
        return Rotation2d.fromRadians(m_targetState != null ? m_targetState.velocity : 0);
    }

    /** 
     * Returns current angle of offset abs encoder
     * @return angle in Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_absEncoder.get() * 2 * Math.PI);
    }

    /** 
     * Returns target arm angle
     * @return angle in Rotation2d
     */
    public Rotation2d getDesiredAngle() {
        return m_targetState != null ? Rotation2d.fromRadians(m_targetState.position) : getAngle();
    }

    /**
     * Gets whether the arm has reached target point
     * @return boolean state
     */
    public boolean isAtPosition() {
        return m_profile == null || m_profile.isFinished(m_timer.get());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");
        
        builder.addDoubleProperty("Angle (Deg)",  () -> {
            return getAngle().getDegrees();
        }, null);

        builder.addDoubleProperty("Desired Angle (Deg)", () -> {
            return getDesiredAngle().getDegrees();
        }, null);

        builder.addDoubleProperty("Velocity (Deg per sec)", () -> {
            return Units.radiansToDegrees(m_velocity);
        }, null);

        builder.addDoubleProperty("Voltage (Volts)", () -> {
            return m_voltage;
        }, null);
    }

}
