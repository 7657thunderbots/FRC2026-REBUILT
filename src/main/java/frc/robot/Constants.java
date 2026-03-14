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
    public static final int DRIVER_CONTROLLER_PORT = 0;

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 4;
  }

  public static class ShooterConstants {
    public static final int LIMIT_SW_DIO = 0; // limit switch digital IO number
    public static final Pose2d HUB_POSE = new Pose2d(4.65, 4, Rotation2d.kZero);
    // shooter motor velocity once engaged
    public static final double SHOOT_RPMS = 5000;

    // About PID Coefficients
    // They have units. The output of the sparkmax controller PID is in duty cycle.
    // duty cycle can be thought of as the percentage of the battery voltage to
    // apply to the motor
    // The native units are all in rotations, but since we set the conversion
    // factors rotations will become the unit we convert to
    // kP - Duty Cycle per rotation
    // kI - Duty Cycle per rotation*ms (percent output over time)
    // kD - (Duty Cycle)*ms per rotation (how much the duty changed over time)
    // kS - Volts - This number should be the first thing to tune and is the
    // smallest output required to just make the motor turn
    // kV - Volts per RPM - This number needs to be tuned but you can start with the
    // motors Kv on the datasheet is the unloaded RPMs per 1 volt. 473 for a neo 1.1
    // and 565 for a Vortex

    // Azimuth Motor Configuration
    public static final int AZIMUTH_CAN_BUS_ID = 10;
    public static final double AZIMUTH_KP = 0.00325;
    public static final double AZIMUTH_KD = 1.0;
    public static final double AZIMUTH_KI = 0.0;
    public static final double AZIMUTH_KS = 0.01; // static friction feed forward
    public static final double AZIMUTH_KV = 0.0; // velocity feed forward, not used if mode is position control
    public static final float AZIMUTH_GEAR_RATIO = 200 / 21; // 200 tooth turrent gear with 21 tooth drive gear
    public static final int AZIMUTH_ENCODER_COUNTS_PER_REV = Math.round(42 * AZIMUTH_GEAR_RATIO); // Gear ratio times
                                                                                                  // Neo Motor Encoder
                                                                                                  // 42 counts per rev
    // Hood Motor Configuration
    public static final int HOOD_CAN_BUS_ID = 12;
    public static final double HOOD_KP = 0.003;
    public static final double HOOD_KD = 0.01;
    public static final double HOOD_KI = 0.0;
    public static final double HOOD_KS = 0.01; // static friction feed forward
    public static final double HOOD_KV = 0.0; // velocity feed forward, not used if mode is position control
    public static final float HOOD_GEAR_RATIO = 1 / 3; // The hood has a rack and pinion gear
    public static final int HOOD_ENCODER_COUNTS_PER_REV = Math.round(42 * HOOD_GEAR_RATIO);

    // Shoot motor configuation
    public static final int SHOOT_CAN_BUS_ID = 11;
    public static final double SHOOT_KP = 0.0005;
    public static final double SHOOT_KD = 0.0000;
    public static final double SHOOT_KI = 0.0;
    public static final double SHOOT_KS = 0.01; // static friction feed forward
    public static final double SHOOT_KV = 1 / 565; // neo vortex Kv
    // shoter V 0.00215
    // d filter 0.05
    public static final float SHOOT_GEAR_RATIO = 1 / 1; // The Shoot motor is direct drive on the shaft
    public static final int SHOOT_ENCODER_COUNTS_PER_REV = Math.round(42 * SHOOT_GEAR_RATIO);
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
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(.4, .4, .8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(.2, 2, 3);
  }
}
