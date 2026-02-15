package frc.robot.subsystems.swervedrive;

import java.io.File;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Meter;

// WPILIB 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// YAGSL
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {

  public double xSpeed;
  public double ySpeed;
  public double rotspeed;

   /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;
  private final Field2d field = new Field2d();

  public SwerveSubsystem(File directory) {

    //try to initialize the swervedrive, throw an execption if it doesn't work
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
          new Pose2d(new Translation2d(Meter.of(1),
              Meter.of(4)),
              Rotation2d.fromDegrees(0)));

    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    SmartDashboard.putData("Field",field);
  }

  @Override
  public void periodic() {
    field.setRobotPose(swerveDrive.getPose());
    
  }

  @Override
  public void simulationPeriodic() {
 

  }

  public void drive(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
  }
  public Rotation2d GetRotation() {
    return swerveDrive.getYaw();
  }

}
