// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.auto.AutonomousFactory;
import frc.robot.commands.DockAndEngageCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.EnhancedKickAndShootWithDistanceCommand;
import frc.robot.commands.KickerAndShooterWithDistanceCommand;
//import frc.robot.commands.KickerAndShooterStopCommand;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final SwerveDrivetrain m_drive = new SwerveDrivetrain();
    private final Intake m_intake = new Intake();
    //private final PracticeShooter m_shooter = new PracticeShooter(new int[]{21,22},18,new boolean[]{true,false,true});
    private final EnhancedShooter m_shooter = new EnhancedShooter();
    private final TurretSubsystem m_turret = new TurretSubsystem();
    private final LimelightSubsystem m_limelight = new LimelightSubsystem();

    private final AutonomousFactory m_autonomousFacotry = new AutonomousFactory(m_shooter, m_intake, m_drive);

    private final PneumaticHub ph = new PneumaticHub(2);
    
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        ph.enableCompressorDigital();
        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        
        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        m_drive.setDefaultCommand(new Drive(m_drive, driverController));

        driverController.x()
                .and(driverController.y().negate())
                .whileTrue(new InstantCommand(m_intake::extendAndIntake))
                .whileFalse(new InstantCommand(m_intake::retract));


        driverController.y()
                .and(driverController.x().negate())
                .whileTrue(new InstantCommand(m_shooter::setDesiredFlyWheelRPM))
                .whileFalse(new InstantCommand(m_shooter::stop));

        driverController.y()
                .and(driverController.x())
                .whileTrue(new EnhancedKickAndShootWithDistanceCommand(m_shooter, m_limelight));

        //driverController.y()
        //        .and(driverController.x())
        //        .onTrue(new KickerAndShooterCommand(m_shooter, m_limelight))
        //        .onFalse(new KickerAndShooterStopCommand(m_shooter, m_limelight));

        //driverController.y()
        //        .and(driverController.x())
        //        .whileTrue(new InstantCommand(m_shooter::shoot))
        //        .whileFalse(new InstantCommand(m_shooter::stop));

        driverController.start().onTrue(m_drive.resetGyroBase());
//        driverController.rightBumper()
//                .whileTrue(m_turret.trackTargetFactory())
//                .whileFalse(m_turret.stopTurretFactory());

        driverController.b()
                .whileTrue(new DockAndEngageCommand(m_drive));

        driverController.leftBumper()
                .whileTrue(new InstantCommand(m_intake::runMagazineReverse).alongWith(new InstantCommand(m_intake::extend).alongWith(new InstantCommand(m_intake::outtake))))
                .whileFalse(new InstantCommand(m_intake::stopMagazine).alongWith(new InstantCommand(m_intake::retract)).alongWith(new InstantCommand(m_intake::stopIntake)));


    }

    public void onDisabled() {
        this.m_shooter.onDisabled();
    }

    public void onEnabled() {
        this.m_shooter.onEnabled();
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return m_autonomousFacotry.getAutonomousCommand();
        // An example command will be run in autonomous
        // return new FollowPathCommand(m_drive, "New Path");

    }
}
