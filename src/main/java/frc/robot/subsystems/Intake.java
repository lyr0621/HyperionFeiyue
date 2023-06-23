package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    DoubleSolenoid m_pistons;
    TalonFX m_intake;
    TalonFX m_indexer;

    public Intake() {
        m_pistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
        m_intake = new TalonFX(12);
        m_indexer = new TalonFX(13);
    }

    public void extend() {
        m_pistons.set(DoubleSolenoid.Value.kForward);
        m_indexer.set(ControlMode.PercentOutput, 0.75);
        m_intake.set(ControlMode.PercentOutput, 1.0);
    }

    public void retract() {
        m_pistons.set(DoubleSolenoid.Value.kReverse);
        m_indexer.set(TalonFXControlMode.PercentOutput, 0.0);
        m_intake.set(TalonFXControlMode.PercentOutput, 0.0);
    }
}
