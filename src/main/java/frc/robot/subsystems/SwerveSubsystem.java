package frc.robot.subsystems;

import frc.robot.Constants;

import java.io.File;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.*;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;

public class SwerveSubsystem extends SubsystemBase{
    private SwerveDrive m_SwerveDrive;

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        File m_SwerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve/");
        try {
            SwerveParser parser = new SwerveParser(m_SwerveJsonDirectory);
            m_SwerveDrive = parser.createSwerveDrive(Constants.SwerveConstants.MAX_SPEED);
        } catch(Exception m_Exception) {
            throw new RuntimeException(m_Exception);
        }
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
    
    public void driveCommand(ChassisSpeeds chassisSpeed) {
        m_SwerveDrive.drive(chassisSpeed);
    }
}
