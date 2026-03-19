package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class ShootCommand extends Command {
    private final ShooterSubsystem m_ShooterSubsystem;
    private final SpindexerSubsystem m_SpindexerSubsystem;
    private final Timer motorStartup = new Timer();

    public ShootCommand(ShooterSubsystem shooter, SpindexerSubsystem spindexer) {
        m_ShooterSubsystem = shooter;
        m_SpindexerSubsystem = spindexer;
        addRequirements(shooter);
        addRequirements(spindexer);
    }
    // set shoot velocity 5000
    // set kicker velocity 800
    // set spindexer velocity 800

    public void initialize() {
        m_ShooterSubsystem.setShootVelocity(5000);
        motorStartup.restart();

    }

    public void execute() {
        m_ShooterSubsystem.setShootVelocity(5000);
        if (motorStartup.hasElapsed(1)) {
            m_SpindexerSubsystem.setKickerVelocity(800);
            m_SpindexerSubsystem.setSpindexerVelocity(800);
        }

    }

    public void end(boolean interrupted) {
        m_SpindexerSubsystem.setSpindexerVelocity(0);
        m_SpindexerSubsystem.setKickerVelocity(0);
        m_ShooterSubsystem.setShootVelocity(0);
    }

    // public boolean isFinished() {
    // // m_SpindexerSubsystem.engageSpindexer();
    // // m_SpindexerSubsystem.engageKicker();
    // // m_ShooterSubsystem.engageShooter();
    // return true;
    // }
}
