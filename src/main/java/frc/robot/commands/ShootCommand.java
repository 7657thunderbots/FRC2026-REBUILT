package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class ShootCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;
    private final SpindexerSubsystem m_SpindexerSubsystem;

    public ShootCommand(ShooterSubsystem shooter, SpindexerSubsystem spindexer) {
        m_ShooterSubsystem = shooter;
        m_SpindexerSubsystem = spindexer;
        addRequirements(shooter);
        addRequirements(spindexer);
    }

    public void initialize() {
        m_ShooterSubsystem.set

    }

    public void execute() {
        m_ShooterSubsystem.engageShooter();
        m_SpindexerSubsystem.engageKicker();
        m_SpindexerSubsystem.engageSpindexer();
    }
}
