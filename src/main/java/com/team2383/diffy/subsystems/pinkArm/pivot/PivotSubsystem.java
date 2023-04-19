package com.team2383.diffy.subsystems.pinkArm.pivot;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.helpers.Clip;
import com.team2383.diffy.helpers.SparkMaxSimWrapper;
import com.team2383.diffy.helpers.TrapezoidalSubsystemBase;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeConstants;
import com.team2383.diffy.Robot;
import com.team2383.diffy.helpers.AngularVelocityWrapper;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotSubsystem extends TrapezoidalSubsystemBase {
    private final SparkMaxSimWrapper m_rightMotor;
    private final SparkMaxSimWrapper m_leftMotor;

    private final DutyCycleEncoder m_absEncoder;
    private final DutyCycleEncoderSim m_absEncoderSim;

    private final AngularVelocityWrapper m_velocity;

    private final DoubleSupplier m_extensionSupplier;

    /**
     * Pivot Subsystem Constructor (The pivot is the big swinging joint on the pink
     * arm)
     * 
     * @param extension for safety; checks extension to make sure it's not too long
     */
    public PivotSubsystem(DoubleSupplier extension) {
        super("Pivot", PivotConstants.TRAPEZOIDAL_CONSTRAINTS, PivotConstants.SIMULATION_SUBSYSTEM,
                PivotConstants.POSITION_THRESHOLD.getRadians());
        m_leftMotor = new SparkMaxSimWrapper(PivotConstants.BOTTOM_MOTOR_LEFT_ID, MotorType.kBrushless);
        m_rightMotor = new SparkMaxSimWrapper(PivotConstants.BOTTOM_MOTOR_RIGHT_ID, MotorType.kBrushless);
        m_rightMotor.restoreFactoryDefaults();
        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.setVelocityConversionFactor(PivotConstants.VELOCITY_CONVERSION_FACTOR.getRadians());
        m_leftMotor.setVelocityConversionFactor(PivotConstants.VELOCITY_CONVERSION_FACTOR.getRadians());
        m_leftMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);
        m_rightMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);
        m_leftMotor.setInverted(false);
        m_rightMotor.setInverted(true);

        m_absEncoder = new DutyCycleEncoder(PivotConstants.ABS_ENCODER_ID);
        m_absEncoderSim = new DutyCycleEncoderSim(m_absEncoder);

        m_absEncoder.setPositionOffset(PivotConstants.ENCODER_OFFSET);

        m_velocity = new AngularVelocityWrapper(getAngle());

        m_extensionSupplier = extension;

        SmartDashboard.putData("Pivot FF", PivotConstants.FEEDFORWARD_CONTROLLER);
        SmartDashboard.putData("Pivot PID", PivotConstants.PID_CONTROLLER);
    }

    @Override
    public void periodic() {
        super.periodic();
        Rotation2d angle = getAngle();
        m_velocity.calculate(angle);
    }

    /**
     * Uses trapezoidal motion-profiling to implement pseudo-positional control
     * Using this method disables velocity control
     * 
     * @param angle
     * @return boolean state to determine whether the input angle is safe
     */
    public boolean setGoal(Rotation2d angle) {
        // safety for upper bounds
        double adjustedAngle = Clip.clip(PivotConstants.LOWER_BOUND.getRadians(), angle.getRadians(),
                PivotConstants.UPPER_BOUND.getRadians());

        // safety for inside robot
        adjustedAngle = (m_extensionSupplier != null ? m_extensionSupplier.getAsDouble()
                : 0) < TelescopeConstants.SAFETY_BOUND ? adjustedAngle
                        : Clip.invClip(PivotConstants.LOWER_SAFETY.getRadians(), adjustedAngle,
                                PivotConstants.UPPER_SAFETY.getRadians());

        super.setGoal(new TrapezoidProfile.State(adjustedAngle, 0));
        return adjustedAngle == angle.getRadians();
    }

    /**
     * Set velocity of the pivot using PID and feedforward control
     * Using this method disables trapezoidal motion profiling
     * 
     * @param desiredVelocity in radians per second
     */
    public void setVelocity(Rotation2d desiredVelocity) {
        super.setVelocity(desiredVelocity.getRadians());
    }

    /** Handles simulation */
    protected void setSimulatedMotors(Matrix<N1, N1> matrix) {
        double simVelocity = matrix.get(0, 0);
        m_leftMotor.setSimVelocity(simVelocity);
        m_rightMotor.setSimVelocity(simVelocity);

        m_rightMotor.setSimPosition(m_rightMotor.getPosition() + simVelocity * 0.02);
        m_leftMotor.setSimPosition(m_leftMotor.getPosition() + simVelocity * 0.02);

        m_absEncoderSim
                .setDistance(getAngle().getRotations() + (simVelocity / (2 * Math.PI) * 0.02));

    }

    /**
     * Sets the voltage of the pivot motors
     * 
     * @param voltage
     */
    @Override
    protected void setVoltage(double voltage) {
        m_rightMotor.setVoltage(voltage);
        m_leftMotor.setVoltage(voltage);
    }

    /**
     * PIDF calculations used by trapezoidal motion profiling
     * 
     * @param velocity in radians per second
     */
    @Override
    protected double calculateVoltage(double velocity, double position) {
        double voltage = PivotConstants.PID_CONTROLLER.calculate(getAngle().getRadians(), position);
        voltage += PivotConstants.FEEDFORWARD_CONTROLLER.calculate(velocity);
        if (Robot.isReal()) { // Am I on a planet with gravity
            voltage += Math.sin(getAngle().getRadians()) * PivotConstants.kG
                    * ((m_extensionSupplier != null ? m_extensionSupplier.getAsDouble() : 0) + PivotConstants.kGOFFSET)
                    / (PivotConstants.kGOFFSET + 20);
        }
        return voltage;
    }

    /**
     * Gets velocity of the abs encoder
     * 
     * @return
     */
    public Rotation2d getVelocity() {
        return m_velocity.get();
    }

    /**
     * Returns current angle of offset abs encoder
     * 
     * @return angle in Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_absEncoder.get());
    }

    protected TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getAngle().getRadians(), m_velocity.get().getRadians());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty("extension existance", () -> m_extensionSupplier != null, null);
        builder.addDoubleProperty("extension",
                () -> (m_extensionSupplier != null ? m_extensionSupplier.getAsDouble() : 0), null);
    }

}
