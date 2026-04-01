package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig.Presets;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.DefaultSparkMaxConfig.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  /**
   * Objects for the shoot motor control
   */
  private final SparkClosedLoopController intakePid;
  private final SparkMax intakeMotor; // sparkmax driving the big azimuth gear
  private final RelativeEncoder intakeEncoder; // Integrated NEO encoder.

  private final SparkClosedLoopController pivotPid;
  private final SparkMax pivotMotor; // sparkmax driving the big azimuth gear
  private final RelativeEncoder pivotEncoder; // Integrated NEO encoder.
  private final AbsoluteEncoder pivotAbsoluteEncoder; // Pivot absolute encoder

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    intakeMotor = new SparkMax(INTAKE_CAN_BUS_ID, MotorType.kBrushless);
    // Get the onboard PID controller.
    intakePid = intakeMotor.getClosedLoopController();
    intakeEncoder = intakeMotor.getEncoder();

    pivotMotor = new SparkMax(PIVOT_CAN_BUS_ID, MotorType.kBrushless);
    // Get the onboard PID controller.
    pivotPid = pivotMotor.getClosedLoopController();
    pivotEncoder = pivotMotor.getEncoder();
    pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();

    intakeEncoder.setPosition(0);

    configureIntakeMotor();

    configurePivotMotor();

    // set the intake to be at zero... it already should be
    setIntakeVelocity(0);
    // Set Pivot Motor to off until it gets a command
    pivotMotor.stopMotor();
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
   * Sets up the shooter control Sparkmax / Neo to control the output velocity of
   * the shooter
   * All of thee parameters can also be found in the REV 2.0 GUI. If they don't
   * work there, they won't work here
   * 
   **/
  private void configureIntakeMotor() {
    SparkMaxConfig intakeCfg = new SparkMaxConfig().apply((SparkMaxConfig) Presets.REV_NEO);

    // Setup the Sparkmax to control the NEO
    // These are the same Parameters from the REV 2.0 GUI
    intakeCfg.voltageCompensation(VOLTAGE_COMPENSATION);
    intakeCfg.inverted(true);

    // Time to go from zero to full throttle at the controller output
    // We want spin up to be quick
    intakeCfg.closedLoopRampRate(1);

    // PID control constants
    intakeCfg.closedLoop.pid(INTAKE_KP, INTAKE_KI,
        INTAKE_KD, ClosedLoopSlot.kSlot0);
    intakeCfg.closedLoop.dFilter(0.1, ClosedLoopSlot.kSlot0);
    intakeCfg.closedLoop.feedForward.kS(INTAKE_KS, ClosedLoopSlot.kSlot0);
    intakeCfg.closedLoop.feedForward.kV(INTAKE_KV, ClosedLoopSlot.kSlot0);
    // The controller has a max range of -1 to 1, we don't want it to ever run in
    // reverse so set to 0 to 1
    intakeCfg.closedLoop.outputRange(-1, 1);

    // Configure feedback of the PID controller as the integrated Hall encoder.
    intakeCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // leave the position in rotations
    intakeCfg.encoder.positionConversionFactor(INTAKE_GEAR_RATIO);

    // We will control based on RPMs, so no conversion
    intakeCfg.encoder.velocityConversionFactor(INTAKE_GEAR_RATIO);

    // Send the configuration to the sparkmax
    intakeMotor.configure(intakeCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // clear sparkmax faults after writing configuration
    clearStickyFaults(intakeMotor);

  }

  private void configurePivotMotor() {
    SparkMaxConfig pivotCfg = new SparkMaxConfig().apply((SparkMaxConfig) Presets.REV_NEO);

    // Setup the Sparkmax to control the NEO
    // These are the same Parameters from the REV 2.0 GUI
    pivotCfg.voltageCompensation(VOLTAGE_COMPENSATION);

    // Time to go from zero to full throttle at the controller output
    // TODO: This is really not the right way to do this. We should use max motion
    pivotCfg.closedLoopRampRate(2);

    // PID control constants
    pivotCfg.closedLoop.pid(PIVOT_KP, PIVOT_KI,
        PIVOT_KD, ClosedLoopSlot.kSlot0);
    pivotCfg.closedLoop.dFilter(0.1, ClosedLoopSlot.kSlot0);
    pivotCfg.closedLoop.feedForward.kS(PIVOT_KS, ClosedLoopSlot.kSlot0);
    pivotCfg.closedLoop.feedForward.kV(PIVOT_KV, ClosedLoopSlot.kSlot0);
    // The controller has a max range of -1 to 1, we don't want it to ever run in
    // reverse so set to 0 to 1
    pivotCfg.closedLoop.outputRange(-1, 1);

    // Configure feedback of the PID controller as the integrated Hall encoder.
    pivotCfg.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pivotCfg.closedLoop.allowedClosedLoopError(0.1, ClosedLoopSlot.kSlot0);

    // leave the position in rotations
    pivotCfg.encoder.positionConversionFactor(PIVOT_GEAR_RATIO);

    // We will control based on RPMs, so no conversion
    pivotCfg.encoder.velocityConversionFactor(PIVOT_GEAR_RATIO);
    pivotCfg.idleMode(IdleMode.kCoast);

    // Send the configuration to the sparkmax, reset to safe parameters and store
    // the new configuration so it is persistent
    pivotMotor.configure(pivotCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // clear sparkmax faults
    clearStickyFaults(pivotMotor);
  }

  /**
   * Sets the controller for the shooter outout velocity
   *
   * @param intakeRPMS the RPMs for the output flywheel
   */
  private void setIntakeVelocity(double intakeRPMs) {
    configureSparkMax(() -> intakePid.setSetpoint(
        intakeRPMs,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0));

  }

  /**
   * Sets the controller for the shooter outout velocity
   *
   * @param pivotPosition the RPMs for the output flywheel
   */
  private void setPivotPosition(double position) {
    configureSparkMax(() -> pivotPid.setSetpoint(
        position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0));

  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  private void clearStickyFaults(SparkMax motor) {
    configureSparkMax(motor::clearFaults);
  }

  /**
   * Engage the shooter motor
   * return motor speed to zero when command ends
   * resulting in a toggle
   *
   * @return a command
   */
  public Command engageIntake() {

    return run(
        () -> {
          this.setIntakeVelocity(INTAKE_SPEED);
        }).finallyDo(() -> {
          this.setIntakeVelocity(0);
        });
  }

  public Command reverseIntake() {

    return run(
        () -> {
          this.setIntakeVelocity(INTAKE_REV_SPEED);
        }).finallyDo(() -> {
          this.setIntakeVelocity(0);
        });
  }

  /**
   * Engage the shooter motor
   * return motor speed to zero when command ends
   * resulting in a toggle
   *
   * @return a command
   */
  public Command setIntakePivotPosition(double position) {

    return run(
        () -> {
          this.setPivotPosition(position);
        }).finallyDo(() -> {
          pivotMotor.stopMotor();
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Intake Velocity", intakeEncoder::getVelocity, null);
    builder.addDoubleProperty("Intake Position", intakeEncoder::getPosition, null);
    builder.addDoubleProperty("Pivot Velocity", pivotEncoder::getVelocity, null);
    builder.addDoubleProperty("Pivot Position", pivotEncoder::getPosition, null);
  }
}
