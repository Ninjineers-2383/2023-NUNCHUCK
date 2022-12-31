package com.team2383.diffy.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team2383.diffy.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final DiffSwerveModule m_frontLeftModule;
    private final DiffSwerveModule m_frontRightModule;
    private final DiffSwerveModule m_rearModule;

    private final DiffSwerveModule[] m_modules;
    private final SwerveModuleState[] m_lastStates;
    private ChassisSpeeds m_lastChassisSpeed = new ChassisSpeeds();

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    private final int m_gyroSimHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    private final SimDouble m_gyroSimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_gyroSimHandle, "Yaw"));

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            Constants.FrontLeftModule.translation,
            Constants.FrontRightModule.translation,
            Constants.RearModule.translation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getHeading());

    private final Field2d m_field = new Field2d();

    private int m_counter;

    public DrivetrainSubsystem(DataLog log) {
        m_frontLeftModule = new DiffSwerveModule(Constants.FrontLeftModule.kTopMotorID,
                Constants.FrontLeftModule.kBottomMotorID, Constants.FrontLeftModule.kEncoderPortA,
                Constants.FrontLeftModule.kEncoderPortB, Constants.FrontLeftModule.kEncoderPortAbs,
                Constants.FrontLeftModule.staticAngle,
                Constants.FrontLeftModule.mountAngle,
                Constants.FrontLeftModule.name, Constants.kCANivoreBus, log);
        m_frontRightModule = new DiffSwerveModule(Constants.FrontRightModule.kTopMotorID,
                Constants.FrontRightModule.kBottomMotorID, Constants.FrontRightModule.kEncoderPortA,
                Constants.FrontRightModule.kEncoderPortB, Constants.FrontRightModule.kEncoderPortAbs,
                Constants.FrontRightModule.staticAngle,
                Constants.FrontRightModule.mountAngle,
                Constants.FrontRightModule.name, Constants.kCANivoreBus, log);
        m_rearModule = new DiffSwerveModule(Constants.RearModule.kTopMotorID, Constants.RearModule.kBottomMotorID,
                Constants.RearModule.kEncoderPortA, Constants.RearModule.kEncoderPortB,
                Constants.RearModule.kEncoderPortAbs,
                Constants.RearModule.staticAngle,
                Constants.RearModule.mountAngle,
                Constants.RearModule.name, Constants.kCANivoreBus, log);

        m_modules = new DiffSwerveModule[] { m_frontLeftModule, m_frontRightModule, m_rearModule };
        m_lastStates = new SwerveModuleState[m_modules.length];

        loadWheelOffsets();

        SmartDashboard.putData("Field", m_field);

        addChild(Constants.FrontLeftModule.name, m_frontLeftModule);
        addChild(Constants.FrontRightModule.name, m_frontRightModule);
        addChild(Constants.RearModule.name, m_rearModule);

        if (RobotBase.isSimulation()) {
            m_odometry.resetPosition(new Pose2d(new Translation2d(0, 0), new Rotation2d()), new Rotation2d());
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());

        for (int i = 0; i < m_modules.length; i++) {
            m_lastStates[i] = m_modules[i].getState();
        }

        m_lastChassisSpeed = m_kinematics.toChassisSpeeds(m_lastStates[0], m_lastStates[1], m_lastStates[2]);

        m_odometry.update(getHeading(), m_lastStates[0], m_lastStates[1], m_lastStates[2]);

        m_field.setRobotPose(m_odometry.getPoseMeters());

        if (RobotController.getUserButton() && m_counter == 0) {
            setWheelOffsets();
            m_counter = 100;
            DataLogManager.log("INFO: User Button Pressed\nSetting all module rotation offsets\n");
        }

        if (m_counter > 0)
            m_counter--;
    }

    @Override
    public void simulationPeriodic() {
        m_frontLeftModule.simulate();
        m_frontRightModule.simulate();
        m_rearModule.simulate();

        // It is important that this is the only place that uses getYaw because it is
        // not offset
        // The chassis speed also needs to be negated because Yaw is CW + not CCW+ like
        // all other angles
        m_gyroSimAngle.set(m_gyro.getYaw() + (-m_lastChassisSpeed.omegaRadiansPerSecond * 180 / Math.PI) * 0.02);
    }

    /**
     * Drive the robot using field or robot relative velocity
     * 
     * @param xSpeed        The speed in the x (forward+) direction (m/s)
     * @param ySpeed        The speed in the y (left+) direction (m/s)
     * @param rotSpeed      The rotation rate (rad/s) CCW+
     * @param fieldRelative Whether the speeds are relative to the field or the
     *                      robot
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative,
            Translation2d centerOfRotation) {
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getHeading())
                        : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed),
                centerOfRotation);

        setModuleStates(swerveModuleStates);

        SmartDashboard.putNumber("RotSpeed", rotSpeed);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveConstants.kMaxSpeed);

        double driveMaxVolts = 0;

        for (int i = 0; i < m_modules.length; i++) {
            driveMaxVolts = Math.max(driveMaxVolts, m_modules[i].setDesiredState(states[i]));
        }

        double driveMaxScale = 1;

        if (driveMaxVolts > Constants.ModuleConstants.kDriveMaxVoltage) {
            driveMaxScale = Constants.ModuleConstants.kDriveMaxVoltage / driveMaxVolts;
        }

        for (DiffSwerveModule module : m_modules) {
            module.setVoltage(driveMaxScale);
        }
    }

    public void resetEncoders() {
        for (DiffSwerveModule module : m_modules) {
            module.resetEncoders();
        }
    }

    /**
     * Get the current robot heading
     * Note: This is normalized so that 0 is facing directly away from your alliance
     * wall
     * <p>
     * Note: CCW is positive
     * 
     * @return The heading of the robot in a Rotation2D
     */
    public Rotation2d getHeading() {
        return m_gyro.getRotation2d();
    }

    /**
     * Reset the heading to the param
     * 
     * @param currentHeading the heading to reset the gyro to. This must be in field
     *                       relative coordinates when CCW is position and 0 is
     *                       facing directly towards the opposing alliance wall
     */
    public void resetHeading(Rotation2d currentHeading) {
        // getYaw is CW positive not CCW positive
        m_gyro.setAngleAdjustment(m_gyro.getYaw() - currentHeading.getDegrees());
        m_odometry.resetPosition(m_odometry.getPoseMeters(), getHeading());
    }

    /**
     * Get turn rate of robot
     * 
     * @return turn rate in degrees per second CCW positive
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetHeading(pose.getRotation());
        m_odometry.resetPosition(pose, getHeading());
    }

    public void motorsOff() {
        for (DiffSwerveModule module : m_modules) {
            module.motorsOff();
        }
    }

    public void setWheelOffsets() {
        for (DiffSwerveModule module : m_modules) {
            module.setZeroOffset();
        }

        DataLogManager.log("INFO: SetWheelOffsets Complete\n");
    }

    public void loadWheelOffsets() {
        for (DiffSwerveModule module : m_modules) {
            module.loadZeroOffset();
        }

        DataLogManager.log("INFO: LoadWheelOffsets Complete\n");
    }
}
