package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;

public class KickerAndShooterWithDistanceCommand extends CommandBase{

    LimelightSubsystem m_limelight;
    Shooter m_shooter;
    int m_counter = 0;

    public KickerAndShooterWithDistanceCommand(Shooter shooter, LimelightSubsystem limelightSubsystem){
        m_shooter = shooter;
        m_limelight = limelightSubsystem;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        System.out.println();
        SmartDashboard.putNumber("Desired RPM", m_shooter.get_target_rpm(m_limelight.get_distance()));
        double actual_distance = m_limelight.get_distance();
        m_shooter.shoot_from_anywhere(actual_distance);
        double target_rpm = m_shooter.get_target_rpm(actual_distance);
        double shooter_rpm = m_shooter.get_shooter_rpm();
        if (shooter_rpm > target_rpm-400 && shooter_rpm < target_rpm+400){
            m_counter++;
            System.out.println("Ready to shoot?");
            if (m_counter >= 3) {
                System.out.println("Counter 3");
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
        //m_shooter.stop();
    }
}
