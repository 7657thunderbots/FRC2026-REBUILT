// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class JoystickDrive extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  // private final JoystickDrive m_joystickDrive;
  private final SwerveSubsystem m_swerveSubsystem;
  private final XboxController m_xboxController;
  private double velocityX;
  private double velocityY;
  private double rotationSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickDrive(SwerveSubsystem swerveSubsystem, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_xboxController = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocityX = m_xboxController.getLeftX();
    velocityY = m_xboxController.getLeftY();
    rotationSpeed = m_xboxController.getRightX();

    velocityX = MathUtil.applyDeadband(velocityX, Constants.SwerveConstants.STICK_DEADBAND);
    velocityY = MathUtil.applyDeadband(velocityY, Constants.SwerveConstants.STICK_DEADBAND);
    rotationSpeed = MathUtil.applyDeadband(rotationSpeed, Constants.SwerveConstants.STICK_DEADBAND);

    ChassisSpeeds chassisSpeed = new ChassisSpeeds(velocityX, velocityY, rotationSpeed);

    m_swerveSubsystem.driveCommand(chassisSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
