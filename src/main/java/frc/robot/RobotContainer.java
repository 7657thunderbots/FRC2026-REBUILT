package frc.robot;

import java.io.File;

import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.*;
import frc.robot.Constants.*;

import swervelib.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.Filesystem;

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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // m_ShooterSubsystem.setDefaultCommand(null);

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
}
