package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import java.util.HashMap;
import java.util.Map;

public class SwerveDrivetrain extends SubsystemBase {
    private final SwerveModFalcon m_0Mod;
    private final SwerveModFalcon m_1Mod;
    private final SwerveModFalcon m_2Mod;
    private final SwerveModFalcon m_3Mod;

    private final WPI_Pigeon2 m_gyro;
//    private final TimeOfFlight m_tofSensor;

    private boolean fieldOriented = true;

    private final Field2d m_field;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDrivePoseEstimator m_visionEstimator;

    private double m_currentPitch = 0;
    private double m_currentTime = 0;
    private double m_speedMult = 1;
    private double m_rotationMult = 1;

    public SwerveDrivetrain() {
        // Check current robot mode for the proper hardware
        m_1Mod = new SwerveModFalcon(DriveConstants.MOD_1_OFFSET, DriveConstants.MOD_1_CANS);
        m_0Mod = new SwerveModFalcon(DriveConstants.MOD_0_OFFSET, DriveConstants.MOD_0_CANS);
        m_2Mod = new SwerveModFalcon(DriveConstants.MOD_2_OFFSET, DriveConstants.MOD_2_CANS);
        m_3Mod = new SwerveModFalcon(DriveConstants.MOD_3_OFFSET, DriveConstants.MOD_3_CANS);

        // open gyro and ToF sensor
        m_gyro = new WPI_Pigeon2(DriveConstants.GYRO_CAN);


        // open field data and advantagekit outputs
        m_field = new Field2d();

        // construct the pose estimator for odometry
        m_poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.DRIVE_KINEMATICS,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.75, 0.75, Units.degreesToRadians(1.5))
        );

        // construct a secondary estimator for testing with cameras
        m_visionEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS, 
            new Rotation2d(), 
            getModulePositions(), 
            new Pose2d());


        // Setup field and initialize gyro
        SmartDashboard.putData("Field", m_field);
        resetGyro();
        m_visionEstimator.update(new Rotation2d(), getModulePositions());
    }

    @Override
    public void periodic() {
        // Update logged values
        updatePoseEstimator();
        m_field.setRobotPose(getPose());
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        SmartDashboard.putNumber("Swerve Heading", getGyroYaw().getDegrees());
}

    // Getters
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modPos = new SwerveModulePosition[4];

        modPos[0] = m_0Mod.getPosition();
        modPos[1] = m_1Mod.getPosition();
        modPos[2] = m_2Mod.getPosition();
        modPos[3] = m_3Mod.getPosition();

        return modPos;
    }


    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = m_0Mod.getState();
        states[1] = m_1Mod.getState();
        states[2] = m_2Mod.getState();
        states[3] = m_3Mod.getState();

        return states;
    }

    public Rotation2d getGyroYaw() {
        return m_gyro.getRotation2d();
    }


    // Setters
    public void drive(double xTranslation, double yTranslation, double zRotation) {
        SwerveModuleState[] states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xTranslation,
                        yTranslation,
                        zRotation * m_rotationMult,
                        getGyroYaw()
                )
                        : new ChassisSpeeds(xTranslation, yTranslation, zRotation)
        );


        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] states) {
       for (SwerveModuleState state : states) {
           state.speedMetersPerSecond *= m_speedMult;
       }
        m_0Mod.setDesiredState(states[0]);
        m_1Mod.setDesiredState(states[1]);
        m_2Mod.setDesiredState(states[2]);
        m_3Mod.setDesiredState(states[3]);
    }

    public void resetGyro(double heading) {
        m_gyro.setYaw(heading);
    }

    public void resetGyro() {
        resetGyro(0);
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(m_gyro.getPitch());
    }

    public void updatePoseEstimator() {
        /*
         * Get swerve odometry
         */
        m_poseEstimator.update(getGyroYaw(), getModulePositions());
    }

    public void resetPose(Pose2d newPose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), newPose);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    // Command factories and their respective methods
    public Command resetGyroBase() {
        return runOnce(this::resetGyro);
    }

    public SwerveAutoBuilder getAutoBuilder(Map<String, Command> eventMap) {
        return new SwerveAutoBuilder(
                this::getPose,
                this::resetPose,
                Constants.DriveConstants.DRIVE_KINEMATICS,
                Constants.AutoConstants.TranslationConstants,
                Constants.AutoConstants.RotationConstants,
                this::setModuleStates,
                eventMap,
                false,
                this
        );
    }
}
