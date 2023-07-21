package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;

public class TurretSubsystem extends SubsystemBase {
    private final TalonSRX m_turret;
    private final ProfiledPIDController m_pid;
    private final LimelightSubsystem m_limelight;

    private final double m_gearRatio = (25.0/1.0) * (166.0/18.0);//(1.0/25.0) * (18.0/166.0);
    private boolean turretEnabled = true;
    private final double minAngle = 25;
    private final double maxAngle = 45;

    public TurretSubsystem() {
        m_turret = new TalonSRX(19);

        m_turret.configFactoryDefault();

        m_turret.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        m_turret.config_kP(0, 0.02);
        m_turret.config_kI(0, 0.0001);

        m_turret.configMotionProfileTrajectoryInterpolationEnable(true);
        m_turret.configMotionAcceleration(50000); // degrees per 100ms
        m_turret.configMotionCruiseVelocity(50000); // degrees per 100ms

        m_pid = new ProfiledPIDController(0.25, 0.001, 0.0,
                new TrapezoidProfile.Constraints(0.5, 0.1));

        m_limelight = new LimelightSubsystem();
    }

    @Override
    public void periodic() {

    }

    public void setTurretPower(double power) {
        double turretAngle = Utils.falconToDegrees4096(m_turret.getSelectedSensorPosition(), m_gearRatio);

        if (minAngle > turretAngle && power < 0) {
            power = 0;
        } else if (turretAngle > maxAngle && power > 0) {
            power = 0;
        }

        power = MathUtil.clamp(power, -0.25, 0.25);
        SmartDashboard.putNumber("Actual Output", power);

        m_turret.set(ControlMode.PercentOutput, power);
    }

    public void trackTarget() {
        // Calculate output based on limelight data
        double limelightAngle = m_limelight.limelightAngle();
        setTurretAngle(limelightAngle);
    }

    public void setTurretAngle(double inputAngle) {
        double turretAngle = Utils.falconToDegrees4096(m_turret.getSelectedSensorPosition(), m_gearRatio);
        double output = m_pid.calculate(turretAngle, inputAngle);

        // Log dashboard data
        SmartDashboard.putNumber("Measured Angle", turretAngle);
        SmartDashboard.putNumber("Raw Encoder Reading", m_turret.getSelectedSensorPosition());
        SmartDashboard.putNumber("Calculated Output", output);

        if (turretEnabled) {
            setTurretPower(output);
        }
    }

    public CommandBase trackTargetFactory() {
        return run(this::trackTarget);
    }

    public CommandBase enableTurret(boolean isEnabled) {
        return run(() -> turretEnabled = isEnabled);
    }

    public CommandBase stopTurretFactory() {
        return runOnce(() -> m_turret.set(ControlMode.PercentOutput, 0.0));
    }
}

