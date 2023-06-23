package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDrivetrain extends SubsystemBase {
    private final SwerveModFalcon m_frMod;
    private final SwerveModFalcon m_flMod;
    private final SwerveModFalcon m_blMod;
    private final SwerveModFalcon m_brMod;

    private final WPI_Pigeon2 m_gyro;
//    private final TimeOfFlight m_tofSensor;

    private boolean fieldOriented = true;

    private final Field2d m_field;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDrivePoseEstimator m_visionEstimator;

    private double m_currentPitch = 0;
    private double m_previousPitch = 0;
    private double m_currentTime = 0;
    private double m_prevTime = 0;
    private boolean slowmode = false;
    private double m_speedMult = 1;
    private double m_rotationMult = 1;

    public enum AlignmentOptions {
        LEFT_ALIGN,
        CENTER_ALIGN,
        RIGHT_ALIGN,
        HUMAN_PLAYER_ALIGN
    }

    public SwerveDrivetrain() {
        // Check current robot mode for the proper hardware
        m_flMod = new SwerveModFalcon(DriveConstants.MOD_FL_OFFSET, DriveConstants.MOD_FL_CANS);
        m_frMod = new SwerveModFalcon(DriveConstants.MOD_FR_OFFSET, DriveConstants.MOD_FR_CANS);
        m_blMod = new SwerveModFalcon(DriveConstants.MOD_BL_OFFSET, DriveConstants.MOD_BL_CANS);
        m_brMod = new SwerveModFalcon(DriveConstants.MOD_BR_OFFSET, DriveConstants.MOD_BR_CANS);

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

        SwerveModuleState[] states = getModuleStates();
        for (SwerveModuleState s : states) {

        }

        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        // Record prev and current pitch and time used for auto balance
        m_previousPitch = m_currentPitch;
        m_currentPitch = getGyroPitch().getDegrees();

        m_prevTime = m_currentTime;
        m_currentTime = RobotController.getFPGATime();
    }

    // Getters
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modPos = new SwerveModulePosition[4];

        modPos[0] = m_flMod.getPosition();
        modPos[1] = m_frMod.getPosition();
        modPos[2] = m_blMod.getPosition();
        modPos[3] = m_brMod.getPosition();

        return modPos;
    }


    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        states[0] = m_flMod.getState();
        states[1] = m_frMod.getState();
        states[2] = m_blMod.getState();
        states[3] = m_brMod.getState();

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
//            if (SmartDashboard.getBoolean("Stella Mode", true)) {
//                state.speedMetersPerSecond *= 0.2;
//            }
       }

        m_flMod.setDesiredState(states[0]);
        m_frMod.setDesiredState(states[1]);
        m_blMod.setDesiredState(states[2]);
        m_brMod.setDesiredState(states[3]);
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

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(m_gyro.getRoll());
    }

    public double getGyroPitchRate() {
        //Account for initail boot time
        if (m_prevTime == 0) {
            m_prevTime = RobotController.getFPGATime();
        }
        // Return the rate of falling
        double fpgaElapsedTime = RobotController.getFPGATime() - m_prevTime;
        return (m_currentPitch - m_previousPitch) / fpgaElapsedTime;
    }


    public void updatePoseEstimator() {
        /*
         * Get swerve odometry
         */
        m_poseEstimator.update(getGyroYaw(), getModulePositions());
    }

    public Pose2d getFrontCamTagPose() {
        return new Pose2d();
    }

    public int getFrontCamTagID() {
        return 0;
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

    public void setFieldRelative(boolean relative) {
        fieldOriented = relative;
    }

    public CommandBase resetPoseBase() {
        return runOnce(() -> resetPose(new Pose2d()));
    }

    public void setSlowmode() {
        slowmode = !slowmode;
    }

    public boolean getSlowmode() {
        return slowmode;
    }

    public CommandBase setSlowmodeFactory() {
        return runOnce(this::setSlowmode);
    }

    public void setSpeedMult(double mult) {
        m_speedMult = mult;
    }

    public void setRotationMult(double mult) {
        m_rotationMult = mult;
    }
}
