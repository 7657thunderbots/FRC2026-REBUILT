// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import swervelib.math.Matter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double MAX_SPEED = Units.feetToMeters(4);
  public static final double MAXIMUM_AMBIGUITY = 0.90;
  public static final Pose2d SIM_START_POSE = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(0));

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 4;
  }

  public static class ShooterConstants {
    public static final Pose2d hub_pose = new Pose2d(4.65, 4, Rotation2d.kZero);
    public static final int azimuthCANBUSID = 12;
    public static final double azimuth_Kp = 10;
    public static final double azimuth_Kd = 0.1;
    public static final double azimuth_Ki = 0.0;
    public static final float azimuthGearRatio = 200 / 21; // 200 tooth turrent gear with 21 tooth drive gear
    public static final int azimuthEncoderCountsPerRev = Math.round(42 * azimuthGearRatio); // Gear ratio times Neo
                                                                                            // Motor Encoder 42 counts
                                                                                            // per rev
  }

  public static class CameraConstants {
    // Camera configuration class to hold camera name and transform
    public static class CameraConfig {
      public final String name;
      public final Transform3d robotToCamera;

      public CameraConfig(String name, Transform3d robotToCamera) {
        this.name = name;
        this.robotToCamera = robotToCamera;
      }
    }

    // Array of camera configurations - add or remove cameras here
    public static final CameraConfig[] CAMERA_CONFIGS = {
        // Front camera - facing forward, half a meter forward of center, half a meter
        // up from center
        new CameraConfig("FrontCamera", new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0))),
        // Back camera - facing backwards, half a meter back of center, half a meter up
        // from center
        new CameraConfig("BackCamera", new Transform3d(new Translation3d(-0.5, 0.0, 0.5),
            new Rotation3d(0, 0, Math.toRadians(180))))
        // Add more cameras here as needed
    };

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(.4, .4, .8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.2, 2, 3);
  }
}
