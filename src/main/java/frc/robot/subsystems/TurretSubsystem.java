package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;

public class TurretSubsystem extends SubsystemBase {
    private final TalonSRX m_turret;
    private final PIDController m_pid;
    private final LimelightSubsystem m_limelight;

    private final double m_gearRatio = (1/25) * (18/166);
    private boolean turretEnabled = false;
    private final double minAngle = 25;
    private final double maxAngle = 45;

    public TurretSubsystem() {
        m_turret = new TalonSRX(19);

        m_turret.configFactoryDefault();

        m_turret.config_kP(0, 0.02);
        m_turret.config_kI(0, 0.0001);

        m_turret.configMotionProfileTrajectoryInterpolationEnable(true);
        m_turret.configMotionAcceleration(50000); // degrees per 100ms
        m_turret.configMotionCruiseVelocity(50000); // degrees per 100ms

        m_pid = new PIDController(0.5, 0.001, 0.0);

        m_limelight = new LimelightSubsystem();
    }

    @Override
    public void periodic() {
        // Calculate output based on limelight data
        double turretAngle = Utils.falconToDegrees(m_turret.getSelectedSensorPosition(), m_gearRatio);
        double limelightAngle = m_limelight.limelightAngle();

        double output = m_pid.calculate(limelightAngle, 0.0);

        SmartDashboard.putNumber("Measured Angle", turretAngle);
        SmartDashboard.putNumber("Limelight Angle", limelightAngle);
        SmartDashboard.putNumber("Calculated Output", output);

        if (minAngle < turretAngle && output < 0) {
            output = 0;
        } else if (turretAngle < maxAngle && output > 0) {
            output = 0;
        }

        if (turretEnabled) {
            m_turret.set(ControlMode.PercentOutput, output);
        }
    }

    public CommandBase enableTurret(boolean isEnabled) {
        return runOnce(() -> turretEnabled = isEnabled);
    }
}

