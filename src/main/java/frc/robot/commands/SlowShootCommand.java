package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpindexerSubsystem;
import static frc.robot.Constants.*;

public class SlowShootCommand extends Command {
    private final SpindexerSubsystem m_SpindexerSubsystem;
    private final Timer motorStartup = new Timer();

    public SlowShootCommand(SpindexerSubsystem spindexer) {
        m_SpindexerSubsystem = spindexer;
        addRequirements(spindexer);
    }
    // set shoot velocity 5000
    // set kicker velocity 800
    // set spindexer velocity 800

    public void initialize() {
        motorStartup.restart();

    }

    public void execute() {
        if (motorStartup.hasElapsed(1)) {
            m_SpindexerSubsystem.setKickerVelocity(SpindexerConstants.KICKER_SPEED);
            m_SpindexerSubsystem.setSpindexerVelocity(SpindexerConstants.SPINDEXER_SPEED);
        }

    }

    public void end(boolean interrupted) {
        m_SpindexerSubsystem.setSpindexerVelocity(0);
        m_SpindexerSubsystem.setKickerVelocity(0);
    }

    // public boolean isFinished() {
    // // m_SpindexerSubsystem.engageSpindexer();
    // // m_SpindexerSubsystem.engageKicker();
    // // m_ShooterSubsystem.engageShooter();
    // return true;
    // }
}
