package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.helpers.DoubleEncoder;
import frc.robot.helpers.SwerveModuleOptimizer;

public class DiffSwerveModule implements Sendable {
    private final WPI_TalonFX m_topMotor;
    private final TalonFXSimCollection m_topMotorSim;
    private final WPI_TalonFX m_bottomMotor;
    private final TalonFXSimCollection m_bottomMotorSim;

    private final DCMotorSim m_topMotorSimulator;
    private final DCMotorSim m_bottomMotorSimulator;

    private final DoubleEncoder m_encoder;

    private final String m_name;
    private final DataLog m_log;

    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            Constants.ModuleConstants.kPModuleTurningController,
            0.0,
            0.0,
            new Constraints(Constants.ModuleConstants.kMaxAngularVelocity,
                    Constants.ModuleConstants.kMaxAngularAcceleration));

    private final PIDController m_drivePIDController = new PIDController(
            Constants.ModuleConstants.kPModuleDriveController,
            0.0,
            0.0);

    private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
            Constants.ModuleConstants.ks,
            Constants.ModuleConstants.kv,
            Constants.ModuleConstants.ka);

    private final DoubleLogEntry m_topMotorCurrent;
    private final DoubleLogEntry m_bottomMotorCurrent;

    private final DoubleLogEntry m_topMotorRPM;
    private final DoubleLogEntry m_bottomMotorRPM;

    private final DoubleLogEntry m_wheelSpeed;
    private final DoubleLogEntry m_moduleAngleLog;

    private final DoubleLogEntry m_expectedSpeed;
    private final DoubleLogEntry m_expectedAngle;

    /**
     * Drive speed in meters per second
     */
    private double m_driveSpeed;
    /**
     * Module angle in degrees
     * <p>
     * Can be converted to radians with Rotation2d.fromDegrees()
     * <p>
     * CCW+ CW-
     */
    private double m_moduleAngle;

    private Rotation2d m_staticAngle;

    private double m_topVoltage;
    private double m_bottomVoltage;

    private double m_offset;

    private double m_desiredSpeed;
    /**
     * Desired angle in degrees
     * <p>
     * CCW+ CW-
     */
    private double m_desiredAngle;

    private double m_driveOutput;
    private double m_turnOutput;

    private int i = 0;

    /**
     * Creates a new DiffSwerveModule.
     * 
     * @param topMotorID     The CAN ID of the top motor
     * @param bottomMotorID  The CAN ID of the bottom motor
     * @param encoderPortA   The DIO channel of the quadrature encoder's A channel
     * @param encoderPortB   The DIO channel of the quadrature encoder's B channel
     * @param encoderPortAbs The DIO channel of the absolute encoder's PWM channel
     * @param name           The name of the module
     * @param CANbus         The CAN bus the module is on
     * @param log            The robot's DataLog
     */
    public DiffSwerveModule(
            int topMotorID,
            int bottomMotorID,
            int encoderPortA,
            int encoderPortB,
            int encoderPortAbs,
            Rotation2d staticAngle,
            String name,
            String CANbus, DataLog log) {
        // Init all the fields
        m_topMotor = new WPI_TalonFX(topMotorID, CANbus);
        m_topMotorSim = m_topMotor.getSimCollection();

        m_bottomMotor = new WPI_TalonFX(bottomMotorID, CANbus);
        m_bottomMotorSim = m_bottomMotor.getSimCollection();

        m_topMotorSimulator = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.005);
        m_bottomMotorSimulator = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.005);

        m_encoder = new DoubleEncoder(encoderPortA, encoderPortB, encoderPortAbs);

        m_name = name;
        m_log = log;

        m_staticAngle = staticAngle;

        // Reset motors and encoders
        m_topMotor.configFactoryDefault();
        m_bottomMotor.configFactoryDefault();

        m_topMotor.configVoltageCompSaturation(Constants.ModuleConstants.kDriveMaxVoltage);
        m_topMotor.enableVoltageCompensation(true);
        m_bottomMotor.configVoltageCompSaturation(Constants.ModuleConstants.kDriveMaxVoltage);
        m_bottomMotor.enableVoltageCompensation(true);

        SupplyCurrentLimitConfiguration supply = new SupplyCurrentLimitConfiguration(
                true,
                Constants.ModuleConstants.kMaxCurrent,
                Constants.ModuleConstants.kMaxCurrent, 10);

        m_topMotor.configSupplyCurrentLimit(supply);
        m_bottomMotor.configSupplyCurrentLimit(supply);

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_topMotor.setNeutralMode(NeutralMode.Coast);
        m_bottomMotor.setNeutralMode(NeutralMode.Coast);

        m_topMotorCurrent = new DoubleLogEntry(m_log, "/" + m_name + "/topMotorCurrent");
        m_bottomMotorCurrent = new DoubleLogEntry(m_log, "/" + m_name + "/bottomMotorCurrent");

        m_topMotorRPM = new DoubleLogEntry(m_log, "/" + m_name + "/topMotorRPM");
        m_bottomMotorRPM = new DoubleLogEntry(m_log, "/" + m_name + "/bottomMotorRPM");

        m_wheelSpeed = new DoubleLogEntry(m_log, "/" + m_name + "/wheelSpeed");
        m_moduleAngleLog = new DoubleLogEntry(m_log, "/" + m_name + "/moduleAngle");

        m_expectedSpeed = new DoubleLogEntry(m_log, "/" + m_name + "/expectedSpeed");
        m_expectedAngle = new DoubleLogEntry(m_log, "/" + m_name + "/expectedAngle");
    }

    /**
     * Gets the current state of the module
     * Called from periodic
     */
    public SwerveModuleState getState() {
        double topMotorSpeed = m_topMotor.getSelectedSensorVelocity();
        double bottomMotorSpeed = m_bottomMotor.getSelectedSensorVelocity();

        m_driveSpeed = getDriveSpeed(topMotorSpeed, bottomMotorSpeed);
        m_moduleAngle = getModuleAngle();

        // data logging
        m_topMotorCurrent.append(m_topMotor.getStatorCurrent());
        m_bottomMotorCurrent.append(m_bottomMotor.getStatorCurrent());

        m_topMotorRPM.append(topMotorSpeed);
        m_bottomMotorRPM.append(bottomMotorSpeed);

        m_wheelSpeed.append(m_driveSpeed);
        m_moduleAngleLog.append(m_moduleAngle);

        if (i < 200) {
            if (i == 199) {
                m_encoder.reset();
            }
            i++;
        }

        return new SwerveModuleState(m_driveSpeed, Rotation2d.fromDegrees(m_moduleAngle));
    }

    public void simulate() {
        // Simulate the motors
        SmartDashboard.putNumber("Simulated/" + m_name + "/Top Motor Simulator/Input Voltage", m_topVoltage);
        SmartDashboard.putNumber("Simulated/" + m_name + "/Bottom Motor Simulator/Input Voltage", m_bottomVoltage);

        m_topMotorSimulator.setInputVoltage(m_topVoltage);
        m_bottomMotorSimulator.setInputVoltage(m_bottomVoltage);

        m_topMotorSimulator.update(0.02);
        m_bottomMotorSimulator.update(0.02);

        SmartDashboard.putNumber("Simulated/" + m_name + "/Top Motor Simulator/Output Velocity",
                m_topMotorSimulator.getAngularVelocityRPM());
        SmartDashboard.putNumber("Simulated/" + m_name + "/Bottom Motor Simulator/Output Velocity",
                m_bottomMotorSimulator.getAngularVelocityRPM());

        m_topMotorSim.setIntegratedSensorRawPosition(
                (int) (m_topMotorSimulator.getAngularPositionRad() / (2 * Math.PI)) * 2048);
        m_topMotorSim.setIntegratedSensorVelocity(
                (int) (m_topMotorSimulator.getAngularVelocityRadPerSec() / (2 * Math.PI)) * 2048 / 10);

        m_bottomMotorSim.setIntegratedSensorRawPosition(
                (int) (m_bottomMotorSimulator.getAngularPositionRad() / (2 * Math.PI)) * 2048);
        m_bottomMotorSim.setIntegratedSensorVelocity(
                (int) (m_bottomMotorSimulator.getAngularVelocityRadPerSec() / (2 * Math.PI)) * 2048 / 10);

        // Simulate the encoder
        double averagePos = (m_topMotor.getSelectedSensorPosition() + m_bottomMotor.getSelectedSensorPosition()) / 2;
        averagePos /= 2048;
        averagePos /= Constants.ModuleConstants.kTurnGearRatio;

        m_encoder.simulate(averagePos * 360);

        SmartDashboard.putNumber("Simulated/" + m_name + "/Encoder/Rotation", getModuleAngle());
    }

    /**
     * Get drive speed of the module
     * 
     * @returns the speed of the module in m/s
     */
    public double getDriveSpeed(double topMotorSpeed, double bottomMotorSpeed) {
        double speed = ((topMotorSpeed - bottomMotorSpeed) / 2) /* Average sensor Velocity (raw / 100ms) */ *
                (10.0 / 2048.0) /* Motor revolutions per second */ *
                Constants.ModuleConstants.kDriveGearRatio /* Output revolutions per second */ *
                (Constants.ModuleConstants.kDriveWheelDiameterMeters * Math.PI) /*
                                                                                 * Circumference in meters
                                                                                 * (meters/second)
                                                                                 */;

        return speed;
    }

    /**
     * Get the angle of the module
     * <p>
     * CCW+
     * 
     * @returns the angle of the module in degrees
     */
    public double getModuleAngle() {
        double angle = m_encoder.get() - m_offset;

        return angle;
    }

    /**
     * Sets the desired state of the module
     * 
     * @param desiredState the desired state of the module
     * @return the max voltage of the motors
     */
    public double setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleOptimizer.customOptimize(desiredState,
                Rotation2d.fromDegrees(getModuleAngle()), m_staticAngle);

        m_desiredSpeed = state.speedMetersPerSecond;
        m_desiredAngle = state.angle.getDegrees();

        m_expectedSpeed.append(m_desiredSpeed);
        m_expectedAngle.append(m_desiredAngle);

        m_turnOutput = m_turningPIDController.calculate((m_moduleAngle / 180.0 * Math.PI),
                (m_desiredAngle / 180.0 * Math.PI));
        m_turnOutput = MathUtil.clamp(m_turnOutput, -Constants.ModuleConstants.kMaxTurnOutput,
                Constants.ModuleConstants.kMaxTurnOutput);

        m_desiredSpeed *= Math.abs(Math.cos(m_turningPIDController.getPositionError()));

        m_driveOutput = m_drivePIDController.calculate(m_driveSpeed, m_desiredSpeed);

        double driveFeedForward = m_driveFeedForward.calculate(m_desiredSpeed);

        m_topVoltage = m_driveOutput + driveFeedForward + Constants.ModuleConstants.kDriveMaxVoltage * m_turnOutput;

        m_bottomVoltage = -m_driveOutput - driveFeedForward
                + Constants.ModuleConstants.kDriveMaxVoltage * m_turnOutput;

        return Math.max(Math.abs(m_topVoltage), Math.abs(m_bottomVoltage));
    }

    /**
     * Set the voltage of the motors
     * 
     * @param driveMaxScale the max drive voltage divided by the maximum requested
     *                      voltage
     */
    public void setVoltage(double driveMaxScale) {
        m_topMotor.set(ControlMode.PercentOutput,
                (m_topVoltage * driveMaxScale / Constants.ModuleConstants.kDriveMaxVoltage));
        m_bottomMotor.set(ControlMode.PercentOutput,
                (m_bottomVoltage * driveMaxScale / Constants.ModuleConstants.kDriveMaxVoltage));
    }

    public void resetEncoders() {
        m_topMotor.setSelectedSensorPosition(0);
        m_bottomMotor.setSelectedSensorPosition(0);
        m_encoder.reset();
    }

    public void motorsOff() {
        m_topMotor.set(ControlMode.PercentOutput, 0);
        m_bottomMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setZeroOffset() {
        m_encoder.reset();
        double steerPosition = m_encoder.get();
        Preferences.setDouble(m_name, steerPosition);
        loadZeroOffset();
    }

    public void loadZeroOffset() {
        m_offset = Preferences.getDouble(m_name, 0);
        DataLogManager.log(String.format("INFO: %s steerPosition %f\n", m_name, m_offset));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");

        builder.addDoubleProperty("Desired Speed", () -> {
            return m_desiredSpeed;
        }, null);
        builder.addDoubleProperty("Drive Speed", () -> {
            return m_driveSpeed;
        }, null);

        builder.addDoubleProperty("Desired Angle (Degrees", () -> {
            return m_desiredAngle;
        }, null);

        builder.addDoubleProperty("Module Angle (Degrees)", () -> {
            return m_moduleAngle;
        }, null);

        builder.addDoubleProperty("Encoder Zero Offset", () -> {
            return m_encoder.getZeroOffset();
        }, null);

        builder.addDoubleProperty("Raw Quad", () -> {
            return m_encoder.getRawQuad();
        }, null);
        builder.addDoubleProperty("Raw Abs", () -> {
            return m_encoder.getRawAbs();
        }, null);
        builder.addDoubleProperty("Abs Distance", () -> {
            return m_encoder.getAbsDistance();
        }, null);

        builder.addDoubleProperty("Drive Output", () -> {
            return m_driveOutput;
        }, null);
        builder.addDoubleProperty("Turn Output", () -> {
            return m_turnOutput;
        }, null);

        builder.addDoubleProperty("Top Voltage", () -> {
            return m_topVoltage;
        }, null);
        builder.addDoubleProperty("Bottom Voltage", () -> {
            return m_bottomVoltage;
        }, null);

        builder.addDoubleProperty("Top Temperature", () -> {
            return m_topMotor.getTemperature();
        }, null);
        builder.addDoubleProperty("Bottom Temperature", () -> {
            return m_bottomMotor.getTemperature();
        }, null);

        builder.addDoubleProperty("Drive PID P", () -> {
            return m_drivePIDController.getP();
        }, (value) -> {
            m_drivePIDController.setP(value);
        });
        builder.addDoubleProperty("Drive PID I", () -> {
            return m_drivePIDController.getI();
        }, (value) -> {
            m_drivePIDController.setI(value);
        });
        builder.addDoubleProperty("Drive PID D", () -> {
            return m_drivePIDController.getD();
        }, (value) -> {
            m_drivePIDController.setD(value);
        });

        builder.addDoubleProperty("Turn PID P", () -> {
            return m_turningPIDController.getP();
        }, (value) -> {
            m_turningPIDController.setP(value);
        });
        builder.addDoubleProperty("Turn PID I", () -> {
            return m_turningPIDController.getI();
        }, (value) -> {
            m_turningPIDController.setI(value);
        });
        builder.addDoubleProperty("Turn PID D", () -> {
            return m_turningPIDController.getD();
        }, (value) -> {
            m_turningPIDController.setD(value);
        });

        SendableRegistry.addLW(m_drivePIDController, m_name + "/Drive PID");
        SendableRegistry.addLW(m_turningPIDController, m_name + "/Turn PID");
    }
}
