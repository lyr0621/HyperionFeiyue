package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonFX m_left;
    private TalonFX m_right;
    private TalonFX m_kicker;

    private PIDController m_pid;

    public Shooter() {
        SmartDashboard.putNumber("Shooter RPM", 500);

        m_left = new TalonFX(15);
        m_right = new TalonFX(16);
        m_kicker = new TalonFX(14);

        m_pid = new PIDController(1.0, 0.0, 0.0);
    }

    public void shoot() {
        
    }
}
