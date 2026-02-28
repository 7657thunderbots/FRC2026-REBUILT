package frc.robot;

import java.io.File;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.Constants.*;

import swervelib.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.Filesystem;

public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(() -> m_swerveSubsystem.getPose(),
      m_swerveSubsystem.getSwerveDrive().field, m_swerveSubsystem.getSwerveDrive()::addVisionMeasurement);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      // .scaleTranslation(0.8)
      .allianceRelativeControl(true)
      .scaleRotation(-1.0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
  }
}
