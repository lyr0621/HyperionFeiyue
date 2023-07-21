package frc.robot.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PathPlannerFlipper;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.Map;


public class FollowPathWithEventsCommand extends CommandBase {

    private SwerveDrivetrain m_swerve;
    private SwerveAutoBuilder m_swerveBuilder;
    private PathPlannerTrajectory traj;
    private boolean isFinished = false;
    public FollowPathWithEventsCommand(SwerveDrivetrain swerve, String pathName, Map<String, Command> eventMap) {
        m_swerve = swerve;
        m_swerveBuilder = new SwerveAutoBuilder(
                swerve::getPose,
                swerve::resetPose,
                Constants.DriveConstants.DRIVE_KINEMATICS,
                Constants.AutoConstants.TranslationConstants,
                Constants.AutoConstants.RotationConstants,
                swerve::setModuleStates,
                eventMap,
                false,
                swerve
        );

        traj = PathPlanner.loadPath(pathName, Constants.ModuleConstants.L1_MAX_SPEED_MPS, Constants.ModuleConstants.L1_MAX_SPEED_MPS);
        traj = PathPlannerFlipper.flipTrajectory(traj);

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(swerve);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        Command controllerCommand = m_swerveBuilder.followPath(traj);
        controllerCommand.execute();
        isFinished = controllerCommand.isFinished();
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(0.0, 0.0, 0.0);
    }
}
