package com.team2383.diffy.subsystems.drivetrain;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.sim.Pigeon2SimState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team2383.diffy.Constants;
import com.team2383.diffy.subsystems.drivetrain.vision.PhotonCameraWrapper;

public class DrivetrainSubsystem extends SubsystemBase {
    private final DiffSwerveModule m_frontLeftModule;
    private final DiffSwerveModule m_frontRightModule;
    private final DiffSwerveModule m_rearModule;

    private final DiffSwerveModule[] m_modules;
    private final SwerveModuleState[] m_lastStates;
    private ChassisSpeeds m_lastChassisSpeed = new ChassisSpeeds();

    private final PhotonCameraWrapper m_camera = new PhotonCameraWrapper();

    private final Pigeon2 m_gyro = new Pigeon2(0, Constants.kCANivoreBus);
    private final Pigeon2SimState m_gyroSim = m_gyro.getSimState();

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            DriveConstants.frontLeftConstants.translation,
            DriveConstants.frontRightConstants.translation,
            DriveConstants.rearConstants.translation);

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final Field2d m_field = new Field2d();
    private final FieldObject2d m_COR;

    private int m_counter1;
    private int m_counter2;

    public DrivetrainSubsystem(DataLog log) {
        m_frontLeftModule = new DiffSwerveModule(DriveConstants.frontLeftConstants,
                Constants.kCANivoreBus, log);
        m_frontRightModule = new DiffSwerveModule(DriveConstants.frontRightConstants,
                Constants.kCANivoreBus, log);
        m_rearModule = new DiffSwerveModule(DriveConstants.rearConstants,
                Constants.kCANivoreBus, log);

        m_modules = new DiffSwerveModule[] { m_frontLeftModule, m_frontRightModule, m_rearModule };

        m_poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                getHeading(),
                getModulePositions(),
                new Pose2d());

        m_lastStates = new SwerveModuleState[m_modules.length];

        loadWheelOffsets();

        SmartDashboard.putData("Field", m_field);
        m_COR = m_field.getObject("COR");

        addChild(DriveConstants.frontLeftConstants.name, m_frontLeftModule);
        addChild(DriveConstants.frontRightConstants.name, m_frontRightModule);
        addChild(DriveConstants.rearConstants.name, m_rearModule);

        if (RobotBase.isSimulation()) {
            m_poseEstimator.resetPosition(new Rotation2d(), getModulePositions(),
                    new Pose2d(new Translation2d(0, 0), new Rotation2d()));
        }

        resetHeading();

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].periodic();
            m_lastStates[i] = m_modules[i].getState();
        }

        m_lastChassisSpeed = m_kinematics.toChassisSpeeds(m_lastStates[0], m_lastStates[1], m_lastStates[2]);

        m_poseEstimator.update(getHeading(), getModulePositions());

        EstimatedRobotPose cam_pose = m_camera.getEstimatedGlobalPose(getPose());

        if (cam_pose != null) {
            var estimate = cam_pose.estimatedPose.toPose2d();
            if (estimate.getX() > 0 && estimate.getY() > 0 && estimate.getX() < 17 &&
                    estimate.getY() < 9) {
                m_poseEstimator.addVisionMeasurement(cam_pose.estimatedPose.toPose2d(),
                        cam_pose.timestampSeconds,
                        VecBuilder.fill(1, 1, 1));
            }
        }

        Pose2d estimatedPose = m_poseEstimator.getEstimatedPosition();

        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            Translation2d transformedTranslation = new Translation2d(16.46 - estimatedPose.getX(),
                    8.02 - estimatedPose.getY());
            Rotation2d transformedHeading = estimatedPose.getRotation().times(-1);

            estimatedPose = new Pose2d(transformedTranslation, transformedHeading);
        }

        m_field.setRobotPose(estimatedPose);

        if (RobotController.getUserButton()) {
            if (m_counter1 == 0) {
                m_counter2++;
                m_counter1 = 100;
                DataLogManager.log("Counter 2 = " + m_counter2);
            }
        } else {
            if (m_counter2 == 1) {
                setWheelOffsets();
            } else if (m_counter2 == 2) {
                setCompassOffset();
                resetHeading();
            }
            m_counter2 = 0;
        }

        if (m_counter1 > 0) {
            m_counter1--;
        }
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
        m_gyroSim.setRawYaw(
                m_gyro.getYaw().getValue() + (-m_lastChassisSpeed.omegaRadiansPerSecond * 180 / Math.PI) * 0.02);

        m_camera.simulate(getPose());
    }

    /**
     * Drive the robot using field or robot relative velocity
     * 
     * @param drive            The speed for driving
     * @param angle            The set angular speed
     * @param fieldRelative    Whether the speeds are relative to the field or the
     *                         robot
     * @param centerOfRotation Its the center of rotation duh
     */
    public void drive(Translation2d drive, Rotation2d angle, boolean fieldRelative,
            Translation2d centerOfRotation) {
        ChassisSpeeds speeds;

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(drive.getX(), drive.getY(), angle.getRadians(),
                    getHeading());
        } else {
            speeds = new ChassisSpeeds(drive.getX(), drive.getY(), angle.getRadians());
        }

        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                speeds,
                centerOfRotation);

        setModuleStates(swerveModuleStates);

        m_COR.setPose(getPose().plus(new Transform2d(centerOfRotation, new Rotation2d())));
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxVelocity);

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setDesiredState(states[i]);
        }
    }

    public void resetEncoders() {
        // for (DiffSwerveModule module : m_modules) {
        // module.resetEncoders();
        // }
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
    public void forceHeading(Rotation2d currentHeading) {
        m_gyro.setYaw(currentHeading.getDegrees());
        m_poseEstimator.resetPosition(currentHeading, getModulePositions(), m_poseEstimator.getEstimatedPosition());
    }

    /**
     * Set the current heading to the calculated compass heading
     * <p>
     * Gyro offset needs to be saved to robot before this can be used.
     * If the compass heading is not stored this will set the forward direction to
     * north
     */
    public void resetHeading() {
        // getYaw is CW positive not CCW positive
        m_gyro.setYaw(getCompassHeading());
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(),
                new Pose2d(m_poseEstimator.getEstimatedPosition().getTranslation(), getHeading()));
        resetEncoders();
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
        return m_poseEstimator.getEstimatedPosition();
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                m_rearModule.getPosition() };
    }

    public void forceOdometry(Pose2d pose) {
        forceHeading(pose.getRotation());
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
        resetEncoders();
    }

    public void setPosition(Translation2d position) {
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(), new Pose2d(position, getHeading()));
        resetEncoders();
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

    public double getCompassHeading() {
        // Compass is CW positive not CCW positive
        // double fieldCompassHeading = Preferences.getDouble("Compass", 0);
        // double currentCompassHeading = m_gyro.getComas().getValue();

        // return currentCompassHeading - fieldCompassHeading;
        return 0;
    }

    public void setCompassOffset() {
        // Compass is CW positive not CCW positive
        // double fieldCompassHeading = m_gyro.getCompassHeading();
        // Preferences.setDouble("Compass", fieldCompassHeading);

        // DataLogManager.log("INFO: Compass offset set to " + fieldCompassHeading +
        // "\n");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        // builder.addDoubleProperty("Compass Heading", m_gyro::getCompassHeading,
        // null);
    }
}