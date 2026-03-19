package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
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
import com.revrobotics.spark.config.SparkMaxConfig.Presets;

import static frc.robot.Constants.SpindexerConstants.*;
import static frc.robot.Constants.DefaultSparkMaxConfig.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {

    private final SparkClosedLoopController kickerPid;
    private final SparkMax kickerMotor;
    private final RelativeEncoder kickerEncoder;

    private final SparkClosedLoopController spinPid;
    private final SparkMax spinMotor;
    private final RelativeEncoder spinEncoder;

    /** Creates a new SpindexerSubsystem. */
    public SpindexerSubsystem() {
        kickerMotor = new SparkMax(KICKER_CAN_BUS_ID, MotorType.kBrushless);
        kickerPid = kickerMotor.getClosedLoopController();
        kickerEncoder = kickerMotor.getEncoder();

        spinMotor = new SparkMax(SPIN_CAN_BUS_ID, MotorType.kBrushless);
        spinPid = spinMotor.getClosedLoopController();
        spinEncoder = spinMotor.getEncoder();

        kickerEncoder.setPosition(0);
        spinEncoder.setPosition(0);

        configureKickerMotor();
        configureSpinMotor();

        setKickerVelocity(0);
        setSpindexerVelocity(0);
    }

    private void configureSparkMax(Supplier<REVLibError> config) {
        for (int i = 0; i < 4; i++) {
            if (config.get() == REVLibError.kOk) {
                return;
            }
            Timer.delay(Milliseconds.of(5).in(Seconds));
        }
    }

    private void configureKickerMotor() {
        SparkMaxConfig kickerCfg = new SparkMaxConfig().apply((SparkMaxConfig) Presets.REV_NEO);

        kickerCfg.voltageCompensation(VOLTAGE_COMPENSATION);
        kickerCfg.closedLoopRampRate(1);

        kickerCfg.closedLoop.pid(KICKER_KP, KICKER_KI, KICKER_KD, ClosedLoopSlot.kSlot0);
        kickerCfg.closedLoop.dFilter(0.1, ClosedLoopSlot.kSlot0);
        kickerCfg.closedLoop.feedForward.kS(KICKER_KS, ClosedLoopSlot.kSlot0);
        kickerCfg.closedLoop.feedForward.kV(KICKER_KV, ClosedLoopSlot.kSlot0);
        // Kicker should only run in the forward direction, so set the output range to 0
        // to 1
        kickerCfg.inverted(true);
        kickerCfg.closedLoop.outputRange(-1, 1);

        kickerCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // will control on RPMs so only apply the gear ratio to get RPMs at the wheels
        kickerCfg.encoder.positionConversionFactor(KICKER_GEAR_RATIO);
        kickerCfg.encoder.velocityConversionFactor(KICKER_GEAR_RATIO);

        kickerMotor.configure(kickerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        clearStickyFaults(kickerMotor);
    }

    private void configureSpinMotor() {
        SparkMaxConfig spinCfg = new SparkMaxConfig().apply((SparkMaxConfig) Presets.REV_Vortex);

        spinCfg.voltageCompensation(VOLTAGE_COMPENSATION);
        spinCfg.closedLoopRampRate(1);

        spinCfg.closedLoop.pid(SPIN_KP, SPIN_KI, SPIN_KD, ClosedLoopSlot.kSlot0);
        spinCfg.closedLoop.dFilter(0.1, ClosedLoopSlot.kSlot0);
        spinCfg.closedLoop.feedForward.kS(SPIN_KS, ClosedLoopSlot.kSlot0);
        spinCfg.closedLoop.feedForward.kV(SPIN_KV, ClosedLoopSlot.kSlot0);
        // only allow foward velocity. TODO: do we need a reverse mode in to potentially
        // deal with jams?
        spinCfg.closedLoop.outputRange(0, 1);

        spinCfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // only apply gear ratio to get position and velocity at the output shaft of the
        // spindexer, not the motor shaft
        spinCfg.encoder.positionConversionFactor(SPIN_GEAR_RATIO);
        spinCfg.encoder.velocityConversionFactor(SPIN_GEAR_RATIO);

        spinMotor.configure(spinCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        clearStickyFaults(spinMotor);
    }

    public void setKickerVelocity(double rpm) {
        configureSparkMax(() -> kickerPid.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0));
    }

    public void setSpindexerVelocity(double rpm) {
        configureSparkMax(() -> spinPid.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0));
    }

    private void clearStickyFaults(SparkMax motor) {
        configureSparkMax(motor::clearFaults);
    }

    public Command engageKicker() {
        return run(() -> setKickerVelocity(KICKER_SPEED)).finallyDo(() -> setKickerVelocity(0));
    }

    public Command reverseKicker() {
        return run(() -> setKickerVelocity(-1 * KICKER_SPEED)).finallyDo(() -> setKickerVelocity(0));
    }

    public Command engageSpindexer() {
        return run(() -> setSpindexerVelocity(SPINDEXER_SPEED)).finallyDo(() -> setSpindexerVelocity(0));
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
