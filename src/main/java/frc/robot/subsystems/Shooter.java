package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;

public class Shooter extends SubsystemBase {
    private TalonFX m_left;
    private TalonFX m_right;
    private TalonFX m_kicker;

    private PIDController m_pid;

    public Shooter() {
        SmartDashboard.putNumber("Shooter RPM", 2000);

        m_left = new TalonFX(21);
        m_right = new TalonFX(22);
        m_kicker = new TalonFX(18);

        m_right.setInverted(InvertType.InvertMotorOutput);
        m_kicker.setInverted(true);

        m_right.config_kP(0, 0.5);
        m_left.config_kP(0, 0.5);

        m_pid = new PIDController(0.5, 0.0, 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Recorded RPM", Utils.falconToRPM(m_left.getSelectedSensorVelocity() + m_right.getSelectedSensorVelocity() / 2, 1.0));
        SmartDashboard.putNumber("Desired RPM", SmartDashboard.getNumber("Shooter RPM", 2000));
    }

    public void shoot() {
        double output = m_pid.calculate(Utils.falconToRPM(m_left.getSelectedSensorVelocity(), 1.0),// + m_right.getSelectedSensorVelocity()) / 2, 1.0),
                SmartDashboard.getNumber("Shooter RPM", 2000));

        m_right.set(ControlMode.Velocity, Utils.RPMToFalcon(output, 1.0));
        m_left.set(TalonFXControlMode.Velocity, Utils.RPMToFalcon(output, 1.0));
        m_kicker.set(TalonFXControlMode.PercentOutput, 1.0);
    }

    public void stop() {
        m_left.set(TalonFXControlMode.PercentOutput, 0.0);
        m_right.set(TalonFXControlMode.PercentOutput, 0.0);
        m_kicker.set(TalonFXControlMode.PercentOutput, 0.0);
    }
}
