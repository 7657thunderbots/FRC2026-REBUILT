package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.Constants.*;

import swervelib.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(() ->
  // m_swerveSubsystem.getPose(),
  // m_swerveSubsystem.getSwerveDrive().field,
  // m_swerveSubsystem.getSwerveDrive()::addVisionMeasurement);

  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(() -> m_swerveSubsystem.getPose(),
      m_swerveSubsystem.getField());

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
    // m_ShooterSubsystem.setDefaultCommand(null);
    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Center move back", Commands.none());

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);

    m_swerveSubsystem.zeroGyroWithAlliance();
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    m_driverController.a().whileTrue(m_swerveSubsystem.driveForward());

    m_driverController.povUp().whileTrue(m_swerveSubsystem.PointWheelsAt(0));
    m_driverController.povRight().whileTrue(m_swerveSubsystem.PointWheelsAt(90));
    m_driverController.povDown().whileTrue(m_swerveSubsystem.PointWheelsAt(180));
    m_driverController.povLeft().whileTrue(m_swerveSubsystem.PointWheelsAt(270));

    // Left bumper to bring up intake
    // m_driverController.leftBumper().whileTrue(m_IntakeSubsystem.setIntakePivotPosition(0));

    // Right bumper to lower intake
    // m_driverController.rightBumper().whileTrue(m_IntakeSubsystem.setIntakePivotPosition(0.3));

    // Left trigger to start intake
    m_driverController.leftTrigger().whileTrue(m_IntakeSubsystem.engageIntake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous
    // commmand
    return autoChooser.getSelected();
  }
}
