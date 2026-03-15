package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.DefaultSparkMaxConfig.*;

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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private final StringPublisher targetModePublisher;
  // private final DoublePublisher
  private Pose2d targetPose;
  private Pose2d shooterPose;

  private Double manualBearingSetpoint = 0.0;

  private ShooterMode currentMode = ShooterMode.AUTO;

  private final Supplier<Pose2d> robot_pose;

  /**
   * Creates a new ShooterSubsystem.
   * 
   * @param azimuthMotorId      CAN ID of the shooter azimuth control motor
   * @param countsPerRevolution The number of encoder pulses for the {@link Type}
   *                            encoder per revolution.
   **/
  public ShooterSubsystem(Supplier<Pose2d> pose_supplier, Field2d fieldObj) {

    azimuthMotor = new SparkMax(AZIMUTH_CAN_BUS_ID, MotorType.kBrushless);
    // Get the onboard PID controller.
    azimuthPid = azimuthMotor.getClosedLoopController();
    azimuthEncoder = azimuthMotor.getEncoder();

    hoodMotor = new SparkMax(HOOD_CAN_BUS_ID, MotorType.kBrushless);
    // Get the onboard PID controller.
    hoodPid = hoodMotor.getClosedLoopController();
    hoodEncoder = hoodMotor.getEncoder();

    shootMotor = new SparkMax(SHOOT_CAN_BUS_ID, MotorType.kBrushless);
    // Get the onboard PID controller.
    shootPid = shootMotor.getClosedLoopController();
    shootEncoder = shootMotor.getEncoder();

    field = fieldObj;
    robot_pose = pose_supplier;
    // zero the encoders. if there is a procedure to provide an initial value use
    // that here instead
    azimuthEncoder.setPosition(0);
    hoodEncoder.setPosition(0);
    shootEncoder.setPosition(0);

    // configure the motors
    configureAzimuthMotor();
    configureHoodMotor();
    configureShootMotor();

    // set the current target to be the hub based on alliance color
    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
      setTargetPose(BLUE_HUB_POSE);
    } else {
      setTargetPose(RED_HUB_POSE);
    }
    // set the hood to be at zero... it already should be
    setHoodPosition(0);
    // set the shooter velocity to zero
    setShootVelocity(0);

    // add a turret pose to the field object so we can see it in simulation
    Pose2d currAzimuthPose = robot_pose.get();

    shooterPose = new Pose2d(currAzimuthPose.getX(), currAzimuthPose.getY(),
        currAzimuthPose.getRotation().plus(getAzimuthPosition()));

    // Only update field object in simulation
    if (RobotBase.isSimulation()) {
      field.getObject("turretPose").setPose(shooterPose);
    }

    // set the turret position to zero
    setTurretPosition(0.0);
    shooterLog.append("Shooter has Initialized!");

    turretBearingPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getDoubleTopic(
        "shooter/targetbearing").publish();

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
    hoodCfg.closedLoop.feedForward.kS(HOOD_KV, ClosedLoopSlot.kSlot0);
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

    // Time to go from zero to full throttle at the controller output
    // We want spin up to be quick
    shootCfg.closedLoopRampRate(0.1);

    // PID control constants
    shootCfg.closedLoop.pid(SHOOT_KP, SHOOT_KI,
        SHOOT_KD, ClosedLoopSlot.kSlot0);
    shootCfg.closedLoop.dFilter(0.1, ClosedLoopSlot.kSlot0);
    shootCfg.closedLoop.feedForward.kS(SHOOT_KS, ClosedLoopSlot.kSlot0);
    shootCfg.closedLoop.feedForward.kS(SHOOT_KV, ClosedLoopSlot.kSlot0);

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
    return Rotation2d.fromDegrees(azimuthEncoder.getPosition());
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
          this.setShootVelocity(SHOOT_RPMS);
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
   * @param azimuth_cmd The Robot relative angle to point the shooter
   */
  private void setTurretPosition(double azimuth_cmd) {
    double final_cmd;
    // restrict position command to the configured -90 to 90 range
    if (azimuth_cmd > 90) {
      final_cmd = 90;
    } else if (azimuth_cmd < -90) {
      final_cmd = -90;
    } else {
      final_cmd = azimuth_cmd;

    }
    configureSparkMax(() -> azimuthPid.setSetpoint(
        final_cmd,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0));

    // if in simulation set the encoder to the setpoint
    if (RobotBase.isSimulation()) {
      azimuthEncoder.setPosition(final_cmd);
    }
  }

  /**
   * Sets the controller for the shooter Hood position
   *
   * @param hoodPosition the percentage from 0 to 100 of the range of motion
   */
  private void setHoodPosition(double hoodPosition) {
    configureSparkMax(() -> hoodPid.setSetpoint(
        -1 * hoodPosition,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0));

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
  private void setShootVelocity(double shootRPMs) {
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

  @Override
  public void periodic() {

    // get current robot pose
    Pose2d currRobotPose = robot_pose.get();
    // adjust the robot pose based upon the encoder position to get the shooter Pose
    shooterPose = new Pose2d(currRobotPose.getX(), currRobotPose.getY(),
        currRobotPose.getRotation().plus(getAzimuthPosition()));

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
    field.getObject("turretPose").setPose(shooterPose);

  }

  private double calculateTargetBearing(Pose2d target) {
    // Calculate the difference in X and Y position from the shooter to the target
    double dx = target.getX() - shooterPose.getX();
    double dy = target.getY() - shooterPose.getY();

    // dx and dy are two sides of a right triangle with its hypotenuse being the
    // line from the shooter to the hub. We can use the arc tangent function to get
    // the angle
    Rotation2d bearingAngle = Rotation2d.fromRadians(Math.atan2(dy, dx));

    // adjust 0-360 to +/- 180
    double bearingAdj = bearingAngle.getDegrees();
    if (bearingAdj > 180) {
      bearingAdj = 360 - bearingAdj;
    }
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

}
