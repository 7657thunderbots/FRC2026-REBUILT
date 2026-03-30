package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.Constants.*;
import frc.robot.Constants.ShooterConstants.ShootDistance;
import swervelib.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(() ->
  // m_swerveSubsystem.getPose(),
  // m_swerveSubsystem.getSwerveDrive().field,
  // m_swerveSubsystem.getSwerveDrive()::addVisionMeasurement);
  private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem();

  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(() -> m_swerveSubsystem.getPose(),
      m_swerveSubsystem.getField());

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final PowerDistribution m_PowerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true)
      .scaleRotation(-1.0);

  // Establish a Sendable Chooser that will be able to be sent to the
  // SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  private final AutoSlowShootCommand shootCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    shootCommand = new AutoSlowShootCommand(m_ShooterSubsystem, m_SpindexerSubsystem);
    NamedCommands.registerCommand("Intake", m_IntakeSubsystem.engageIntake());
    NamedCommands.registerCommand("pivot Intake", m_IntakeSubsystem.setIntakePivotPosition(0));
    NamedCommands.registerCommand("shoot", shootCommand);
    NamedCommands.registerCommand("reverse pivot Intake", m_IntakeSubsystem.setIntakePivotPosition(45));
    // 45 degrees is arbritrary untill we measure the actual degree of rotation
    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // Configure the trigger bindings
    configureBindings();
    // m_ShooterSubsystem.setDefaultCommand(null);
    // Create the NamedCommands that will be used in PathPlanner
    // NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    SmartDashboard.putData("PDH", m_PowerDistribution);

    // Set the default auto (do nothing)
    // autoChooser.setDefaultOption("Center move back", Commands.none());

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    m_driverController.povDown().whileTrue(m_ShooterSubsystem.RevShooter(ShootDistance.SLOW_SHOOT));
    m_driverController.povUp().whileTrue(m_ShooterSubsystem.RevShooter(ShootDistance.SHOOT));
    m_driverController.povRight().whileTrue(m_ShooterSubsystem.RevShooter(ShootDistance.PASS));

    m_swerveSubsystem.zeroGyroWithAlliance();
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    m_driverController.x().whileTrue(shootCommand);
    // m_driverController.povUp().whileTrue(m_ShooterSubsystem.engageShooter());
    // m_driverController.povDown().whileTrue(m_ShooterSubsystem.engageSlowShoot());
    // m_driverController.povRight().whileTrue(m_ShooterSubsystem.engagePass());
    // Left bumper to bring up intake
    m_driverController.rightTrigger().whileTrue(m_SpindexerSubsystem.engageKicker());
    m_driverController.b().whileTrue(m_SpindexerSubsystem.reverseKicker());
    m_driverController.a().whileTrue(m_SpindexerSubsystem.engageSpindexer());
    m_driverController.a().and(m_driverController.rightTrigger()).whileTrue(m_SpindexerSubsystem.engageBoth());
    // Right bumper to lower intake
    m_driverController.leftBumper().whileTrue(m_IntakeSubsystem.setIntakePivotPosition(0.3));

    // Left trigger to start intake
    m_driverController.leftTrigger().whileTrue(m_IntakeSubsystem.engageIntake());
    m_driverController.y().whileTrue(m_IntakeSubsystem.reverseIntake());
    m_driverController.povLeft().whileTrue(m_swerveSubsystem.resetOdometry());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous
    // commmand
    Command autoCommand = autoChooser.getSelected();
    System.out.println(autoCommand.getName());
    return autoCommand;
  }

  public void setMotorBrake(boolean brake) {
    m_swerveSubsystem.setMotorBrake(brake);
  }
}
