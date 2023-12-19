//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.k;

public class SwerveDrive {
    private int ModuleCount;

    private SwerveModule[] m_modules;
    private Pigeon2 m_pigeon2;
    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;
    private SwerveModulePosition[] m_modulePositions;
    private Translation2d[] m_moduleLocations;
    private OdometryThread m_odometryThread;
    private Field2d m_field;
    private PIDController m_turnPid;

    /* Put smartdashboard calls in separate thread to reduce performance impact */
    public void updateDashboard() {
        SmartDashboard.putNumber("Successful Daqs", m_odometryThread.getSuccessfulDaqs());
        SmartDashboard.putNumber("Failed Daqs", m_odometryThread.getFailedDaqs());
        SmartDashboard.putNumber("X Pos", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y Pos", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Angle", m_odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Odometry Loop Time", m_odometryThread.getTime());
    }

    /* Perform swerve module updates in a separate thread to minimize latency */
    private class OdometryThread extends Thread {
        private BaseStatusSignal[] m_allSignals;
        public int SuccessfulDaqs = 0;
        public int FailedDaqs = 0;

        private LinearFilter lowpass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;
        private double averageLoopTime = 0;

        public OdometryThread() {
            super();
            // 4 signals for each module + 2 for Pigeon2
            m_allSignals = new BaseStatusSignal[(ModuleCount * 4) + 2];
            for (int i = 0; i < ModuleCount; ++i) {
                var signals = m_modules[i].getSignals();
                m_allSignals[(i * 4) + 0] = signals[0];
                m_allSignals[(i * 4) + 1] = signals[1];
                m_allSignals[(i * 4) + 2] = signals[2];
                m_allSignals[(i * 4) + 3] = signals[3];
            }
            m_allSignals[m_allSignals.length - 2] = m_pigeon2.getYaw();
            m_allSignals[m_allSignals.length - 1] = m_pigeon2.getAngularVelocityZ();
        }

        @Override
        public void run() {
            /* Make sure all signals update at around 250hz */
            for (var sig : m_allSignals) {
                sig.setUpdateFrequency(250);
            }
            /* Run as fast as possible, our signals will control the timing */
            while (true) {
                /* Synchronously wait for all signals in drivetrain */
                var status = BaseStatusSignal.waitForAll(0.1, m_allSignals);
                lastTime = currentTime;
                currentTime = Utils.getCurrentTimeSeconds();
                averageLoopTime = lowpass.calculate(currentTime - lastTime);

                /* Get status of the waitForAll */
                if (status.isOK()) {
                    SuccessfulDaqs++;
                } else {
                    FailedDaqs++;
                }

                /* Now update odometry */
                for (int i = 0; i < ModuleCount; ++i) {
                    /*
                     * No need to refresh since it's automatically refreshed from the waitForAll()
                     */
                    m_modulePositions[i] = m_modules[i].getPosition(false);
                }
                // Assume Pigeon2 is flat-and-level so latency compensation can be performed
                double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(
                        m_pigeon2.getYaw(), m_pigeon2.getAngularVelocityZ());

                m_odometry.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);
                m_field.setRobotPose(m_odometry.getPoseMeters());
            }
        }

        public double getTime() {
            return averageLoopTime;
        }

        public int getSuccessfulDaqs() {
            return SuccessfulDaqs;
        }

        public int getFailedDaqs() {
            return FailedDaqs;
        }
    }
    public SwerveDrive() {
        // TODO: Calibrate the PID values and SlipCurrent for stator
        SwerveDriveTrainConstants m_drivetrainConstants = new SwerveDriveTrainConstants()
                .withPigeon2Id(5)
                .withCANbusName(k.ROBOT.CANVORE_CANFD_NAME)
                .withTurnKp(5)
                .withTurnKi(0.1);
        Slot0Configs m_steerGains = new Slot0Configs();
        Slot0Configs m_driveGains = new Slot0Configs();
        m_steerGains.kP = 30;
        m_steerGains.kI = 0.0;
        m_steerGains.kD = 0.2;
        m_driveGains.kP = 1;
        m_driveGains.kI = 0;
        SwerveDriveConstantsCreator m_constantsCreator = new SwerveDriveConstantsCreator(
                k.DRIVE.GEAR_RATIO, // ratio for the drive motor
                k.STEER.GEAR_RATIO_TO_CANCODER, // ratio for the steer motor
                k.DRIVE.WHEEL_DIAMETER_m, // 4 inch diameter for the wheels
                17, // Only apply 24 stator amps to prevent slip
                m_steerGains, // Use the specified steer gains
                m_driveGains, // Use the specified drive gains
                true // CANcoder not reversed from the steer motor. For WCP Swerve X this should be
                     // true.
        );
        SwerveModuleConstants m_frontRight = m_constantsCreator.createModuleConstants(
            23, 13, 3, -0.538818,k.DRIVEBASE.WHEEL_BASE_X_m / 2.0, -k.DRIVEBASE.WHEEL_BASE_Y_m / 2.0);
    
        SwerveModuleConstants m_frontLeft = m_constantsCreator.createModuleConstants(
            22, 12, 2, -0.474609, k.DRIVEBASE.WHEEL_BASE_X_m / 2.0, k.DRIVEBASE.WHEEL_BASE_Y_m / 2.0);
        SwerveModuleConstants m_back = m_constantsCreator.createModuleConstants(
            21, 11, 1, -0.928467, -k.DRIVEBASE.WHEEL_BASE_X_m / 2.0, 0.0);
        initialize(m_drivetrainConstants, m_frontLeft, m_frontRight, m_back);    
            
    }
    public void initialize(SwerveDriveTrainConstants _driveTrainConstants, SwerveModuleConstants... _modules){
        ModuleCount = _modules.length;

        m_pigeon2 = new Pigeon2(_driveTrainConstants.m_pigeon2Id, _driveTrainConstants.m_canBusName);

        m_modules = new SwerveModule[ModuleCount];
        m_modulePositions = new SwerveModulePosition[ModuleCount];
        m_moduleLocations = new Translation2d[ModuleCount];

        int iteration = 0;
        for (SwerveModuleConstants module : _modules) {
            m_modules[iteration] = new SwerveModule(module, _driveTrainConstants.m_canBusName);
            m_moduleLocations[iteration] = new Translation2d(module.m_locationX_m, module.m_locationY_m);
            m_modulePositions[iteration] = m_modules[iteration].getPosition(true);

            iteration++;
        }
        m_kinematics = new SwerveDriveKinematics(m_moduleLocations);
        m_odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon2.getRotation2d(), getSwervePositions());
        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);

        m_turnPid = new PIDController(_driveTrainConstants.m_rotateKp, 0, _driveTrainConstants.m_rotateKd);
        m_turnPid.enableContinuousInput(-Math.PI, Math.PI);
        m_turnPid.setTolerance(Math.toRadians(1));
        
        m_odometryThread = new OdometryThread();
        m_odometryThread.start();
    }
    private SwerveModulePosition[] getSwervePositions() {
        return m_modulePositions;
    }

    public void driveRobotCentric(ChassisSpeeds _speeds) {
        var swerveStates = m_kinematics.toSwerveModuleStates(_speeds);
        for (int i = 0; i < ModuleCount; ++i) {
            m_modules[i].apply(swerveStates[i]);
        }
    }

    public void driveFieldCentric(ChassisSpeeds _speeds) {
        var roboCentric = ChassisSpeeds.fromFieldRelativeSpeeds(_speeds, m_pigeon2.getRotation2d());
        var swerveStates = m_kinematics.toSwerveModuleStates(roboCentric);
        for (int i = 0; i < ModuleCount; ++i) {
            m_modules[i].apply(swerveStates[i]);
        }
    }

    public void driveAngleFieldCentric(double _xSpeeds, double _ySpeeds, Rotation2d _targetAngle) {
        var currentAngle = m_pigeon2.getRotation2d();
        double rotationalSpeed = m_turnPid.calculate(currentAngle.getRadians(), _targetAngle.getRadians());

        var roboCentric = ChassisSpeeds.fromFieldRelativeSpeeds(
                _xSpeeds, _ySpeeds, rotationalSpeed, m_pigeon2.getRotation2d());
        var swerveStates = m_kinematics.toSwerveModuleStates(roboCentric);
        for (int i = 0; i < ModuleCount; ++i) {
            m_modules[i].apply(swerveStates[i]);
        }
    }

    public void driveStopMotion() {
        /* Point every module toward (0,0) to make it close to a X configuration */
        for (int i = 0; i < ModuleCount; ++i) {
            var angle = m_moduleLocations[i].getAngle();
            m_modules[i].apply(new SwerveModuleState(0, angle));
        }
    }

    public void resetYaw() {
        m_pigeon2.setYaw(0);
    }

    public Pose2d getPoseMeters() {
        return m_odometry.getPoseMeters();
    }

    public double getSuccessfulDaqs() {
        return m_odometryThread.SuccessfulDaqs;
    }

    public double getFailedDaqs() {
        return m_odometryThread.FailedDaqs;
    }

    public double getRobotYaw() {
        return m_pigeon2.getYaw().getValueAsDouble();
    }

    public boolean isTurnPIDatSetpoint() {
        return m_turnPid.atSetpoint();
    }
}
