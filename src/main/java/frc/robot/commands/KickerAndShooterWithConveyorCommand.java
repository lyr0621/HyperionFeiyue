package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class KickerAndShooterWithConveyorCommand extends CommandBase{

    Shooter m_shooter;
    Intake m_intake;
    int m_counter = 0;
    double m_rpm;

    public KickerAndShooterWithConveyorCommand(Shooter shooter, Intake intake, double rpm){
        m_shooter = shooter;
        m_intake = intake;
        m_rpm = rpm;
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        m_intake.intaking();
        m_shooter.runFlywheel(m_rpm);
        double shooter_rpm = m_shooter.get_shooter_rpm();
        if (shooter_rpm > m_rpm-400 && shooter_rpm < m_rpm+400){
            m_counter++;
            if (m_counter >= 3) {
                m_shooter.runKicker();
            }
        }
        else {
            m_counter = 0;
        }

    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.stop();
        m_intake.retract();
    }
}
