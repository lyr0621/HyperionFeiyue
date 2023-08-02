package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.HashMap;
import java.util.Map;

public class AutonomousFactory {

    Shooter m_shooter;
    SwerveDrivetrain m_drivetrain;
    Intake m_intake;

    private static final AutonMode DEFAULT_MODE = AutonMode.DO_NOTHING;

    public enum AutonMode {
        DO_NOTHING,
        SCORE_AND_LEAVE_COMMUNITY_1,
        SCORE_AND_LEAVE_COMMUNITY_7,
        SCORE_AND_LEAVE_COMMUNITY_AND_DOCK_AND_ENGAGE_4,
        TEST_LINE
    }

    private final SendableChooser<AutonMode> m_chooseAutoOption;

    private final Map<AutonMode, Command> m_autoOptions = new HashMap<>();

    public AutonomousFactory(Shooter shooter, Intake intake, SwerveDrivetrain drivetrain) {
        m_shooter = shooter;
        m_intake = intake;
        m_drivetrain = drivetrain;

        m_chooseAutoOption = new SendableChooser<>();

        m_autoOptions.put(AutonMode.DO_NOTHING, new SequentialCommandGroup());
        m_autoOptions.put(AutonMode.SCORE_AND_LEAVE_COMMUNITY_1, new ScoreAndLeaveCommunity(m_shooter, m_drivetrain, m_intake, "1-leave"));
        m_autoOptions.put(AutonMode.SCORE_AND_LEAVE_COMMUNITY_7, new ScoreAndLeaveCommunity(m_shooter, m_drivetrain, m_intake, "7-leave"));
        m_autoOptions.put(AutonMode.SCORE_AND_LEAVE_COMMUNITY_AND_DOCK_AND_ENGAGE_4, new ScoreAndLeaveCommunityAndDockAndEngage(m_shooter, m_drivetrain, m_intake, "4-leave-charge"));
        m_autoOptions.put(AutonMode.TEST_LINE, new ScoreAndLeaveCommunity(m_shooter, m_drivetrain, m_intake, "4-charge"));


        for (AutonMode auto: AutonMode.values()) {
            if (auto == DEFAULT_MODE) {
                m_chooseAutoOption.setDefaultOption(auto.toString(), auto);
            }
            else {
                m_chooseAutoOption.addOption(auto.toString(), auto);
            }
        }
        SmartDashboard.putData("Autonomous Mode", m_chooseAutoOption);

    }

    public Command getAutonomousCommand() {
        AutonMode mode = m_chooseAutoOption.getSelected();
        return m_autoOptions.get(mode);
    }

}
