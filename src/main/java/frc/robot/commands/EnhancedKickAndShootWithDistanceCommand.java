package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EnhancedShooter;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;

public class EnhancedKickAndShootWithDistanceCommand extends CommandBase {
    LimelightSubsystem m_limelight;
    EnhancedShooter m_shooter;
    int m_counter = 0;
    private boolean targetSeen;
    private boolean desiredFlyWheelSpeedReached;
    /**
     * to calculate the time elapsed after the fly wheel is ready(at which point the shooter module automatically launches the kicker)
     * so that the program knows if 0.6 second is elapsed and stops the shooter after which
     */
    private Timer speedReachedTimer;

    public EnhancedKickAndShootWithDistanceCommand(EnhancedShooter shooter, LimelightSubsystem limelightSubsystem){
        m_shooter = shooter;
        m_limelight = limelightSubsystem;
        addRequirements(shooter);
        speedReachedTimer = new Timer();
    }

    /** called when the button turns from unpressed to pressed */
    @Override
    public void initialize(){
        targetSeen = false;
        desiredFlyWheelSpeedReached = false;
        if (m_limelight.limelightAngle() == 0)
            return;
        double distance = m_limelight.get_distance();
        m_shooter.shoot_from_anywhere(distance);
        m_shooter.setWaitAndShootTask();
    }

    /** called repeatedly when the button is pressed */
    @Override
    public void execute(){
        if (!desiredFlyWheelSpeedReached) // if the desired fly wheel speed is not reached yet
            if (!m_shooter.isOnWaitAndShootTask()) { // sense for reaching
                desiredFlyWheelSpeedReached = true;
                speedReachedTimer.start();
                return;
            }
    }

    @Override
    public boolean isFinished(){
        return (!targetSeen) && desiredFlyWheelSpeedReached && speedReachedTimer.get() > 0.6;
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.stop();
    }
}
