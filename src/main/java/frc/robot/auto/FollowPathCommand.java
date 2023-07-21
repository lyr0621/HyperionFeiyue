package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.PathPlannerFlipper;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.HashMap;
import java.util.List;


public class FollowPathCommand extends CommandBase {
    private final SwerveDrivetrain swerveDrivetrain;
    private PathPlannerTrajectory traj;
    private final PathConstraints constraints;
    private boolean isFinished = false;

    public FollowPathCommand(SwerveDrivetrain swerveDrivetrain, String pathname) {
        this.swerveDrivetrain = swerveDrivetrain;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        constraints = new PathConstraints(Constants.ModuleConstants.L1_MAX_SPEED_MPS, Constants.ModuleConstants.L1_MAX_SPEED_MPS);
        traj = PathPlanner.loadPath(pathname, constraints);

        if (traj == null) {
            DriverStation.reportWarning("Trajectory Not Found", true);
            throw new RuntimeException("Trajectory not found");
        }

        traj = PathPlannerFlipper.flipTrajectory(traj);
        if (traj == null) {
            DriverStation.reportWarning("Flip Trajectory Not found", true);
        }

        addRequirements(this.swerveDrivetrain);
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
        CommandScheduler.getInstance().schedule(swerveDrivetrain.getAutoBuilder(new HashMap<>()).fullAuto(traj));
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
        return true;
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
    }
}
