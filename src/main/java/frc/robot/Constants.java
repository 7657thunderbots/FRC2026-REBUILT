// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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
  public static final double MAX_SPEED = Units.feetToMeters(14);
  public static final double MAXIMUM_AMBIGUITY = 0.90;
  public static final Pose2d RED_START_POSE = new Pose2d(new Translation2d(14, 4), Rotation2d.fromDegrees(180));
  public static final Pose2d BLUE_START_POSE = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(0));

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 4;
  }

  public static class DefaultSparkMaxConfig {
    // Default SparkMaxConfig values for all motors

    // Setup to run at 12V
    public static final double VOLTAGE_COMPENSATION = 12.0;
  }

  public static class IntakeConstants {

    public static final double INTAKE_UP_SETPOINT = 0.35;
    public static final double INTAKE_DOWN_SETPOINT = -0.01;

    public static final double INTAKE_SPEED = 800.0;
    public static final double INTAKE_REV_SPEED = -800.0;
    // Intake Motor Configuration
    public static final int INTAKE_CAN_BUS_ID = 13;
    public static final double INTAKE_KP = 0.0005;
    public static final double INTAKE_KD = 0.0;
    public static final double INTAKE_KI = 0.0;
    public static final double INTAKE_KS = 0.155; // static friction feed forward
    public static final double INTAKE_KV = 0.0107; // velocity feed forward, not used if mode is position control
    public static final double INTAKE_GEAR_RATIO = 1.0 / 5.0; // 5 motor revolutions per 1 revolution of intake wheels

    // Intake Pivot Motor Configuration
    public static final int PIVOT_CAN_BUS_ID = 16;
    public static final double PIVOT_KP = 0.2;
    public static final double PIVOT_KD = 0.0;
    public static final double PIVOT_KI = 0.0;
    public static final double PIVOT_KS = 0.1; // static friction feed forward
    public static final double PIVOT_KV = 0.5; // velocity feed forward, not used if mode is position control
    public static final double PIVOT_GEAR_RATIO = 1.0 / 128; // 120 motor revolutions per 1 revolution of pivot

  }

  public static class SpindexerConstants {
    // Kicker motor (replaces intake)
    public static final double KICKER_SPEED = 800.0;
    public static final int KICKER_CAN_BUS_ID = 20;
    public static final double KICKER_KP = 0.0001;
    public static final double KICKER_KD = 0.0;
    public static final double KICKER_KI = 0.0;
    public static final double KICKER_KS = 0.1;
    public static final double KICKER_KV = 0.006;
    public static final double KICKER_GEAR_RATIO = 1.0 / 3.0;

    // Spin Motor Constants
    public static final double SPINDEXER_SPEED = 700.0;
    public static final int SPIN_CAN_BUS_ID = 15;
    public static final double SPIN_KP = 0.0001;
    public static final double SPIN_KD = 0.0;
    public static final double SPIN_KI = 0.0;
    public static final double SPIN_KS = 0.12;
    public static final double SPIN_KV = 0.002;
    public static final double SPIN_GEAR_RATIO = 1.0 / 3.0;
  }

  public static class ShooterConstants {
    public static final int LIMIT_SW_DIO = 0; // limit switch digital IO number
    // forward is +x, left is +y.
    // the chassis is 23" square and the shooter is 12.5" square mounted to the back
    // left corner 2 inches inset from the frame perimeter
    // the back right corner of the bot would be at 23/2 inches = 11.5 inches.
    // the shooter corner is then at 11.5 - 2 inches = 9.5 inches from the center of
    // the robot in the x direction and the same in y. 11.5 - 12.5/2 inches = 5.75
    // inches from the center of the robot in the x and y direction. The shooter is
    // also rotated 180 degrees from the forward facing direction of the robot, so
    // we need to add a rotation of 180 degrees to get from the robot pose to the
    // shooter pose.
    // So the transform should be -X 5.75 inches and +Y 5.75 inches with a rotation
    // of 180 degrees.

    public enum ShootDistance {
      SLOW_SHOOT,
      SHOOT,
      PASS
    };

    public static final double SLOW_SHOOT_RPM = 3500;
    public static final double SHOOT_RPM = 3800;
    public static final double PASS_RPM = 6500;

    public static final Transform2d ROBOT_TO_SHOOTER = new Transform2d(
        new Translation2d(Units.inchesToMeters(-5.75), Units.inchesToMeters(5.75)),
        Rotation2d.fromDegrees(180.0));

    public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.65, 4, Rotation2d.kZero);
    public static final Pose2d RED_HUB_POSE = new Pose2d(11.9, 4, Rotation2d.kZero);
    // shooter motor velocity once engaged

    public class PrecalculatedValues {
    }

    // public static final int[] SHOOTER_SPEEDS = { 0, 1500, 2000, 2500 }; // index
    // is distance/some unit
    public InterpolatingDoubleTreeMap SHOOTER_SPEEDS = new InterpolatingDoubleTreeMap();

    // Access: PrecalculatedValues.SHOOTER_SPEEDS[index]

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
    public static final boolean AZIMUTH_MOTOR_ENABLED = false; // Set to true to enable the azimuth motor and use it for
                                                               // aiming
    public static final int AZIMUTH_CAN_BUS_ID = 17;
    public static final double AZIMUTH_KP = 0.005;
    public static final double AZIMUTH_KD = 0.05;
    public static final double AZIMUTH_KI = 0.0;
    public static final double AZIMUTH_KS = 0.001; // static friction feed forward
    public static final double AZIMUTH_KV = 0.0; // velocity feed forward, not used if mode is position control
    public static final double AZIMUTH_GEAR_RATIO = 200.0 / 21.0; // 200 tooth turrent gear with 21 tooth drive gear

    // Hood Motor Configuration
    public static final boolean HOOD_MOTOR_ENABLED = false;
    public static final int HOOD_CAN_BUS_ID = 18;
    public static final double HOOD_KP = 0.06;
    public static final double HOOD_KD = 0.0;
    public static final double HOOD_KI = 0.0;
    public static final double HOOD_KS = 3.2; // static friction feed forward
    public static final double HOOD_KV = 0.0; // velocity feed forward, not used if mode is position control
    public static final double HOOD_GEAR_RATIO = 1.0 / 3.0; // The hood has a 3:1 reduction from the motor to the hood
                                                            // output

    // Shoot motor configuation
    public static final int SHOOT_CAN_BUS_ID = 19;
    public static final double SHOOT_KP = 0.0002;
    public static final double SHOOT_KD = 0.0000;
    public static final double SHOOT_KI = 0.0;
    public static final double SHOOT_KS = 0.005; // static friction feed forward
    public static final double SHOOT_KV = 0.00183; // tuned by hand but the vortex has a Kv of 565; // neo vortex Kv
    public static final double SHOOT_GEAR_RATIO = 1.0 / 1.0; // The Shoot motor is direct drive on the shaft

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
