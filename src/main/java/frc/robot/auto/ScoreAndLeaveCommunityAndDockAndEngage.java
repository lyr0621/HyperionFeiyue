package frc.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DockAndEngageCommand;
import frc.robot.commands.KickerAndShooterWithConveyorCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.HashMap;
import java.util.List;

public class ScoreAndLeaveCommunityAndDockAndEngage extends SequentialCommandGroup {

    public ScoreAndLeaveCommunityAndDockAndEngage(Shooter shooter, SwerveDrivetrain drivetrain, Intake intake, String path){

        List<PathPlannerTrajectory> trajectory;
        trajectory = PathPlanner.loadPathGroup(path, new PathConstraints(48, 48));

        //addCommands(new KickerAndShooterWithConveyorCommand(shooter, intake, 1000)
        //        .withTimeout(4));

        addCommands(drivetrain.getAutoBuilder(new HashMap<>()).fullAuto(trajectory));

        addCommands(new DockAndEngageCommand(drivetrain));

    }
}
