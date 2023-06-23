package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrivetrain;

public class Drive extends CommandBase {
    private final SwerveDrivetrain m_drive;
    private final CommandXboxController m_driverController;

    public Drive(SwerveDrivetrain drive, CommandXboxController driverController) {
        m_drive = drive;
        m_driverController = driverController;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        m_drive.drive(
                m_driverController.getLeftX() * 0.5,
                -m_driverController.getLeftY() * 0.5,
                m_driverController.getRightX() * 0.5
        );
    }
}
