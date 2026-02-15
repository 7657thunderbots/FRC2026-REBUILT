// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class JoystickSwerveCommand extends Command {
  private final SwerveSubsystem swerve;
  private final XboxController controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickSwerveCommand( SwerveSubsystem swerve, XboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -controller.getLeftY();
    double ySpeed = -controller.getLeftX();
    double rot    = -controller.getRightX();

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.05);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.05);
    rot    = MathUtil.applyDeadband(rot, 0.05);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed, rot,swerve.GetRotation());
    System.out.println("executing");
    swerve.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
