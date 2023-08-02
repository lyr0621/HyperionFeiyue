package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class DockAndEngageCommand extends CommandBase {

    SwerveDrivetrain m_drivetrain;
    int m_counter = 0;

    public DockAndEngageCommand(SwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_drivetrain.dockAndEngage();
    }

    @Override
    public boolean isFinished() {
        double angle = m_drivetrain.getPitch();
        if (angle > -2 && angle < 2){
            m_counter++;
            if (m_counter >= 5) {
                return true;
            }
            return false;
        }
        else {
            m_counter = 0;
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted == true){
            m_drivetrain.drive(0,0,0);
        }
    }

}
