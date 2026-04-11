package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.DefaultSparkMaxConfig.*;

import frc.robot.Constants.ShooterConstants;

import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfig.Presets;
import java.util.NavigableMap;
import java.util.TreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  /**
   * Objects for the azimuth control
   */
  private final SparkClosedLoopController azimuthPid;
  private final SparkMax azimuthMotor; // sparkmax driving the big azimuth gear
  private final RelativeEncoder azimuthEncoder; // Integrated NEO encoder.

  /**
   * Objects for the hood control
   */
  private final SparkClosedLoopController hoodPid;
  private final SparkMax hoodMotor; // sparkmax driving the hood
  private final RelativeEncoder hoodEncoder; // Integrated NEO encoder.

  /**
   * Objects for the shoot motor control
   */
  private final SparkClosedLoopController shootPid;
  private final SparkMax shootMotor; // sparkmax driving the shooter wheels
  private final RelativeEncoder shootEncoder; // Integrated encoder.

  private final DigitalInput limitSwitch = new DigitalInput(LIMIT_SW_DIO);
  private final Field2d field;
  private final StringLogEntry shooterLog = new StringLogEntry(DataLogManager.getLog(), "/shooter/events");
  private final DoublePublisher turretBearingPublisher;
  private final DoublePublisher turretCalcPublisher;
  private final StringPublisher targetModePublisher;
  // private final DoublePublisher
  private Pose2d targetPose;

  // The shooterPose is always fixed offset from the robot pose, so we can
  // calculate it based on the robot pose and a constant transform that we define
  // in constants. This code uses the term turret Pose to refer to the postiion of
  // the turret
  // turret and shooter pose are equal when the turrent is pointed at 0 degrees
  // (straight ahead or backwards depending on how you want to think about it
  // since the turrent is mounted facing the rear of the robot)
  private Pose2d shooterPose;

  private Double manualBearingSetpoint = 0.0;

  private ShooterMode currentMode = ShooterMode.AUTO;

  private final Supplier<Pose2d> robot_pose;

  // Made By Xavier
  public Command RevShooter(ShootDistance shooterDistance) {
    return run(
        () -> {
          switch (shooterDistance) {
            case SLOW_SHOOT:
              this.setShootVelocity(SLOW_SHOOT_RPM);
              break;
            case SHOOT:
              this.setShootVelocity(SHOOT_RPM);
              break;
            case PASS:
              this.setShootVelocity(PASS_RPM);
              break;
            default:
              break;
          }
        }).finallyDo(() -> {
          this.setShootVelocity(0);
        });
  }

  /**
   * Creates a new ShooterSubsystem.
   * 
   * @param azimuthMotorId      CAN ID of the shooter azimuth control motor
   * @param countsPerRevolution The number of encoder pulses for the {@link Type}
   *                            encoder per revolution.
   **/
  public ShooterSubsystem(Supplier<Pose2d> pose_supplier, Field2d fieldObj) {

    field = fieldObj;
    robot_pose = pose_supplier;

    if (AZIMUTH_MOTOR_ENABLED) {
      azimuthMotor = new SparkMax(AZIMUTH_CAN_BUS_ID, MotorType.kBrushless);
      // Get the onboard PID controller.
      azimuthPid = azimuthMotor.getClosedLoopController();
      azimuthEncoder = azimuthMotor.getEncoder();
      azimuthEncoder.setPosition(0);
      // configure the motor
      configureAzimuthMotor();
      setTurretPosition(0);
    } else {
      azimuthMotor = null;
      azimuthPid = null;
      azimuthEncoder = null;
    }

    if (HOOD_MOTOR_ENABLED) {
      hoodMotor = new SparkMax(HOOD_CAN_BUS_ID, MotorType.kBrushless);
      // Get the onboard PID controller.
      hoodPid = hoodMotor.getClosedLoopController();
      hoodEncoder = hoodMotor.getEncoder();
      hoodEncoder.setPosition(0);
      configureHoodMotor();
      // set the hood to be at zero... it already should be
      setHoodPosition(0);
    } else {
      hoodMotor = null;
      hoodPid = null;
      hoodEncoder = null;
    }

    shootMotor = new SparkMax(SHOOT_CAN_BUS_ID, MotorType.kBrushless);
    // Get the onboard PID controller.
    shootPid = shootMotor.getClosedLoopController();
    shootEncoder = shootMotor.getEncoder();
    // zero the encoder
    shootEncoder.setPosition(0);
    // configure the motor
    configureShootMotor();
    // set the shooter velocity to zero
    setShootVelocity(0);

    // set the current target to be the hub based on alliance color
    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
      setTargetPose(BLUE_HUB_POSE);
    } else {
      setTargetPose(RED_HUB_POSE);
    }

    // offset the shooter pose from the robot using the transform defined in
    // constants. This will be used to calculate the bearing to the target and also
    // just to visualize where the shooter is pointing on the field in simulation
    shooterPose = robotToShooter();

    // Only update field object in simulation
    if (RobotBase.isSimulation()) {
      field.getObject("turretPose").setPose(shooterPose);
    }

    shooterLog.append("Shooter has Initialized!");

    turretBearingPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
        "shooter/targetbearing").publish();

    turretCalcPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
        "shooter/debug").publish();
    turretCalcPublisher.set(0);

    targetModePublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getStringTopic(
        "shooter/targetmode").publish();
    targetModePublisher.set("AUTO");

  }

  /**
   * Sets up the azimuth control Sparkmax / Neo to control pointing direction of
   * the shooter
   * All of thee parameters can also be found in the REV 2.0 GUI. If they don't
   * work there, they won't work here
   * 
   **/
  private void configureAzimuthMotor() {
    SparkMaxConfig azimuthCfg = new SparkMaxConfig().apply((SparkMaxConfig) Presets.REV_NEO);

    // Setup the Sparkmax to control the NEO
    // These are the same Parameters from the REV 2.0 GUI
    azimuthCfg.voltageCompensation(VOLTAGE_COMPENSATION);

    // Time to go from zero to full throttle at the controller output
    azimuthCfg.closedLoopRampRate(0.25);

    // PID control constants
    azimuthCfg.closedLoop.pid(AZIMUTH_KP, AZIMUTH_KI,
        AZIMUTH_KD);
    azimuthCfg.closedLoop.feedForward.kS(AZIMUTH_KS, ClosedLoopSlot.kSlot0);
    azimuthCfg.closedLoop.feedForward.kV(AZIMUTH_KV, ClosedLoopSlot.kSlot0);
    azimuthCfg.closedLoop.dFilter(0.1, ClosedLoopSlot.kSlot0);
    // The controller has a max range of -1 to 1
    azimuthCfg.closedLoop.outputRange(-1, 1);

    // Configure feedback of the PID controller as the integrated encoder.
    azimuthCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // set up the controller to stop the motor when it hits the 90 degree limits
    azimuthCfg.limitSwitch.limitSwitchPositionSensor(FeedbackSensor.kPrimaryEncoder);
    azimuthCfg.softLimit.forwardSoftLimit(90);
    azimuthCfg.softLimit.forwardSoftLimitEnabled(true);
    azimuthCfg.softLimit.reverseSoftLimit(-90);
    azimuthCfg.softLimit.reverseSoftLimitEnabled(true);

    double degreesPerRevolution = 360 / AZIMUTH_GEAR_RATIO;
    azimuthCfg.encoder.positionConversionFactor(degreesPerRevolution); // change turret rotations to degrees

    // velocity is in revolutions per minute and we want to convert to degrees per
    // second. So divide degrees per revolution by 60
    azimuthCfg.encoder.velocityConversionFactor(degreesPerRevolution / 60); // revolutions per minute to degrees per
                                                                            // second

    // Send the configuration to the sparkmax, reset to defaults prior to
    // configuration
    azimuthMotor.configure(azimuthCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // clear sparkmax faults
    clearStickyFaults(azimuthMotor);
  }

  /**
   * Sets up the Hood control Sparkmax / Neo to control the hood of the shooter
   * All of thee parameters can also be found in the REV 2.0 GUI. If they don't
   * work there, they won't work here
   * 
   **/
  private void configureHoodMotor() {
    SparkMaxConfig hoodCfg = new SparkMaxConfig().apply((SparkMaxConfig) Presets.REV_NEO);

    // Setup the Sparkmax to control the NEO
    // These are the same Parameters from the REV 2.0 GUI
    hoodCfg.voltageCompensation(VOLTAGE_COMPENSATION);

    // Time to go from zero to full throttle
    hoodCfg.closedLoopRampRate(0.25);

    // PID control constants
    hoodCfg.closedLoop.pid(HOOD_KP, HOOD_KI,
        HOOD_KD, ClosedLoopSlot.kSlot0);
    hoodCfg.closedLoop.feedForward.kS(HOOD_KS, ClosedLoopSlot.kSlot0);
    hoodCfg.closedLoop.feedForward.kV(HOOD_KV, ClosedLoopSlot.kSlot0);
    hoodCfg.closedLoop.dFilter(0.1, ClosedLoopSlot.kSlot0);

    // The controller has a max range of -1 to 1,
    hoodCfg.closedLoop.outputRange(-1, 1);

    // Configure feedback of the PID controller as the integrated encoder.
    hoodCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // set up the controller to stop the motor when it hits 100% position
    hoodCfg.limitSwitch.limitSwitchPositionSensor(FeedbackSensor.kPrimaryEncoder);
    hoodCfg.softLimit.forwardSoftLimit(100);
    hoodCfg.softLimit.forwardSoftLimitEnabled(true);
    hoodCfg.softLimit.reverseSoftLimit(0);
    hoodCfg.softLimit.reverseSoftLimitEnabled(true);

    // The hood range is about 0 to 0.8 rotations of the motor, but we want to
    // control it based on percentage of that range. So we need to convert the
    // position and velocity units to be in percentage of the range instead of
    // rotations or RPMs
    double percentageToRevolution = (0.8 / HOOD_GEAR_RATIO) * 100;
    hoodCfg.encoder.positionConversionFactor(percentageToRevolution); // change hood rotations to percentage of position
                                                                      // range

    hoodCfg.encoder.velocityConversionFactor(percentageToRevolution / 60); // revolutions per minute to percent range
                                                                           // per second

    // Send the configuration to the sparkmax
    hoodMotor.configure(hoodCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // clear sparkmax faults
    clearStickyFaults(hoodMotor);

  }

  /**
   * Sets up the shooter control Sparkmax / Neo to control the output velocity of
   * the shooter
   * All of thee parameters can also be found in the REV 2.0 GUI. If they don't
   * work there, they won't work here
   * 
   **/
  private void configureShootMotor() {
    SparkMaxConfig shootCfg = new SparkMaxConfig().apply((SparkMaxConfig) Presets.REV_Vortex);

    // Setup the Sparkmax to control the NEO
    // These are the same Parameters from the REV 2.0 GUI
    shootCfg.voltageCompensation(VOLTAGE_COMPENSATION);
    shootCfg.inverted(true);
    // Time to go from zero to full throttle at the controller output
    // We want spin up to be quick
    shootCfg.closedLoopRampRate(0.1);

    // PID control constants
    shootCfg.closedLoop.pid(SHOOT_KP, SHOOT_KI,
        SHOOT_KD, ClosedLoopSlot.kSlot0);
    shootCfg.closedLoop.dFilter(0.1, ClosedLoopSlot.kSlot0);
    shootCfg.closedLoop.feedForward.kS(SHOOT_KS, ClosedLoopSlot.kSlot0);
    shootCfg.closedLoop.feedForward.kV(SHOOT_KV, ClosedLoopSlot.kSlot0);

    // The controller has a max range of -1 to 1, we don't want it to ever run in
    // reverse so set to 0 to 1
    shootCfg.closedLoop.outputRange(0, 1);

    // Configure feedback of the PID controller as the integrated Hall encoder.
    shootCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // leave the position in rotations
    shootCfg.encoder.positionConversionFactor(SHOOT_GEAR_RATIO);

    // We will control based on RPMs, so no conversion
    shootCfg.encoder.velocityConversionFactor(SHOOT_GEAR_RATIO);

    // Send the configuration to the sparkmax
    shootMotor.configure(shootCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // clear sparkmax faults
    clearStickyFaults(shootMotor);

  }

  /**
   * 
   * @return Return the robot relative rotation of the shooter
   */
  public Rotation2d getAzimuthPosition() {

    if (AZIMUTH_MOTOR_ENABLED) {
      return Rotation2d.fromDegrees(azimuthEncoder.getPosition());
    } else {
      return Rotation2d.fromDegrees(0);
    }

  }

  /**
   * Sets the shooter target for AUTO mode
   *
   * @param target a pose2d on the field to target
   */
  public void setTargetPose(Pose2d target) {
    targetPose = target;
  }

  /**
   * Sets the shooter mode to either AUTO or MANUAL
   *
   * @param mode AUTO or MANUAL
   */
  public void setShooterMode(ShooterMode mode) {

    if (mode == ShooterMode.AUTO) {
      shooterLog.append("Shooter Mode Set to AUTO");
      targetModePublisher.set("AUTO");
    } else {
      shooterLog.append("Shooter Mode Set to Manual");
      targetModePublisher.set("MANUAL");
    }

    currentMode = mode;
  }

  public ShooterMode getShooterMode() {
    return currentMode;
  }

  /**
   * Engage the shooter motor
   * return motor speed to zero when command ends
   * resulting in a toggle
   *
   * @return a command
   */
  public Command engageShooter() {

    return run(
        () -> {
          this.setShootVelocity(SHOOT_RPM);
        }).finallyDo(() -> {
          this.setShootVelocity(0);
        });
  }

  public Command engageSlowShoot() {

    return run(
        () -> {
          this.setShootVelocity(SLOW_SHOOT_RPM);
        }).finallyDo(() -> {
          this.setShootVelocity(0);
        });
  }

  public Command engagePass() {

    return run(
        () -> {
          this.setShootVelocity(PASS_RPM);
        }).finallyDo(() -> {
          this.setShootVelocity(0);
        });
  }

  public Command calibrateShooterAzimuth() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  // TODO: we need to calibrate the azimuth encoder somehow...
  private void calibrateAzimuth() {
    // probably should make sure we don't wake up on the switch
    if (limitSwitch.get()) {

    }

    while (!limitSwitch.get()) {
      // use open loop to run the motor really slow until the limit switch is hit.

    }

  }

  /**
   * Sets the controller for the shooter azimuth position
   *
   * @param azimuth_cmd The shooter relative angle to point the shooter
   */
  private void setTurretPosition(double azimuth_cmd) {

    // restrict position command to the configured -90 to 90 range
    if (azimuth_cmd > 90.0) {
      azimuth_cmd = 90.0;
    } else if (azimuth_cmd < -90.0) {
      azimuth_cmd = -90.0;
    }
    final double final_cmd = azimuth_cmd;
    if (AZIMUTH_MOTOR_ENABLED) {
      configureSparkMax(() -> azimuthPid.setSetpoint(
          final_cmd,
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0));
    }

    // if in simulation set the encoder to the setpoint
    if (RobotBase.isSimulation() && (DriverStation.isAutonomous() || DriverStation.isTeleop())) {
      if (AZIMUTH_MOTOR_ENABLED) {
        azimuthEncoder.setPosition(final_cmd);
      }
    }
  }

  /**
   * Sets the controller for the shooter Hood position
   *
   * @param hoodPosition the percentage from 0 to 100 of the range of motion
   */
  private void setHoodPosition(double hoodPosition) {
    if (HOOD_MOTOR_ENABLED) {
      configureSparkMax(() -> hoodPid.setSetpoint(
          -1 * hoodPosition,
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0));
    }
    // if in simulation set the encoder to the setpoint
    if (RobotBase.isSimulation()) {
      hoodEncoder.setPosition(hoodPosition);
    }
  }

  /**
   * Sets the controller for the shooter outout velocity
   *
   * @param shootRPMs the RPMs for the output flywheel
   */
  public void setShootVelocity(double shootRPMs) {
    configureSparkMax(() -> shootPid.setSetpoint(
        shootRPMs,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0));

  }

  /**
   * Run the configuration until it succeeds or times out.
   *
   * @param config Lambda supplier returning the error state.
   */
  private void configureSparkMax(Supplier<REVLibError> config) {

    for (int i = 0; i < 4; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
      Timer.delay(Milliseconds.of(5).in(Seconds));
    }
    // failureConfiguringAlert.set(true);
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  private void clearStickyFaults(SparkMax motor) {
    configureSparkMax(motor::clearFaults);
  }

  private Pose2d robotToShooter() {
    Pose2d currRobotPose = robot_pose.get();

    // adjust the robot pose by a transform that defines the fixed position of the
    // shooter relative to the robot.
    return currRobotPose.transformBy(ROBOT_TO_SHOOTER);
  }

  private Pose2d robotToTurret() {
    // There are 3 steps to get the turret position. First we need the shooter
    // position based on the robot position and the fixed transform from the robot
    // to the shooter. Then we need to get the rotation of the turret relative to
    // the shooter from the azimuth encoder. Finally we can apply that rotation as a
    // transform to the shooter pose to get the turret pose.

    // get the shooter position
    Pose2d baseShooterPose = robotToShooter();
    // get the encoder position which is always relative to the shooter, so it gives
    // us the rotation of the turret relative to the shooter
    Rotation2d turretRot = Rotation2d.fromDegrees(getAzimuthPosition().getDegrees());

    // now create a transform which will apply the turret rotation to the shooter
    // pose, but not change the translation so it rotates about its center
    Transform2d turretTransform = new Transform2d(Translation2d.kZero, turretRot);

    // apply the transform to the shooter pose to get the turret pose
    return baseShooterPose.transformBy(turretTransform);
  }

  @Override
  public void periodic() {

    shooterPose = robotToShooter();

    double targetBearing = manualBearingSetpoint;
    // Based on the updated positions above, find the bearing angle to the target if
    // in AUTO
    if (currentMode == ShooterMode.AUTO) {
      targetBearing = calculateTargetBearing(targetPose);
    }
    // send the bearing angle to network tables so we can see it in Advantage Scope
    turretBearingPublisher.set(targetBearing);
    // update the current turret Position
    setTurretPosition(targetBearing);

  }

  @Override
  public void simulationPeriodic() {

    // Update turret pose in the field object so we can see it in simulation
    field.getObject("turretPose").setPose(robotToTurret());

  }

  private double calculateTargetBearing(Pose2d target) {
    // This is just finding the difference in X and Y between the target and
    // shooter.
    // think of it as
    // deltax = targetX - shooterX
    // deltay = targetY - shooterY.
    // We do it this way so we can rotate the delta into the shooters frame of
    // reference and then just
    // use atan2 to get the angle to the target relative to where the shooter is
    // pointing instead of relative to the field which would be more complicated
    // when we want to find the angle from the turret
    Translation2d deltaField = target.getTranslation().minus(shooterPose.getTranslation());

    // this rootate into the shooters frame of reference by using the inverse of the
    // shooters rotation.
    Translation2d deltaTurret = deltaField.rotateBy(shooterPose.getRotation().unaryMinus());

    // dx and dy are two sides of a right triangle with its hypotenuse being the
    // line from the shooter to the hub. We can use the arc tangent function to get
    // the angle
    Rotation2d shooterRelAngle = Rotation2d.fromRadians(Math.atan2(deltaTurret.getY(), deltaTurret.getX()));

    double bearingAdj = shooterRelAngle.getDegrees();

    return bearingAdj;
  }

  // find the distance to the current target
  private double calculateTargetDistance(Pose2d target) {
    // Calculate the difference in X and Y position from the shooter to the target
    double dx = target.getX() - shooterPose.getX();
    double dy = target.getY() - shooterPose.getY();

    // Use Pythagorean's Theorem to find the length of the hypotenuse of the
    // triangle

    return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

  }

  // set the target bearing for manual mode
  public void setManualTargetBearing(double targetBearing) {
    manualBearingSetpoint = targetBearing;
  }

  public Command setManualAzimuth(double cmd) {
    this.setShooterMode(ShooterMode.MANUAL);
    return runOnce(
        () -> {
          this.setManualTargetBearing(0);
          /* one-time action goes here */
        });
  }

  /**
  * 
  */
  public enum ShooterMode {
    /**
     * Auto Position Mode
     */
    AUTO,
    /**
     * Manual Position Mode
     */
    MANUAL
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // keep the base class sendables and then add to them
    super.initSendable(builder);

    builder.addStringProperty(
        ".mode",
        () -> getShooterMode().name(), null);

  }

  public class FiringSolutionSubsystem {
    // A map to store distance (key) to shooter speed (value) data points
    private final InterpolatingDoubleTreeMap m_shooterSpeedMap = new InterpolatingDoubleTreeMap(); //

    public FiringSolutionSubsystem() {
      // Populate the lookup table with known, pre-tested values
      m_shooterSpeedMap.put(1.0, 1500.0); // Distance 1m, Speed 1500 RPM
      m_shooterSpeedMap.put(2.0, 2000.0); // Distance 2m, Speed 2000 RPM
      m_shooterSpeedMap.put(3.0, 2500.0); // Distance 3m, Speed 2500 RPM
      // Add more points as needed
    }

    /**
     * Gets the interpolated shooter speed for a given distance.
     * 
     * @param distance The distance to the target in meters.
     * @return The calculated shooter speed in RPM.
     */
    public double getShooterSpeedForDistance(double distance) {
      // The .get() method automatically interpolates between the nearest points
      return m_shooterSpeedMap.get(distance); //
    }
  }
}
