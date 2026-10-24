package team3647.frc2026.subsystems;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2026.constants.ShooterConstants;
import team3647.lib.ModifiedSignalLogger;
import team3647.lib.TalonFXSubsystem;
import team3647.lib.tunables.TunableSlotConfigs;

public class Shooter extends TalonFXSubsystem {

    public double idleSpeed = ShooterConstants.kIdleSpeed;
    public double idleSpeedInMid = ShooterConstants.kIdleSpeedMid;
    private BooleanSupplier isInMidSupplier = () -> false;
    // Pre-spin: when prepping, the idle speed is raised to the target shot speed so the default
    // command spins the shooter up ahead of a shot without a separate shooter-requiring command.
    private boolean prepping = false;
    private DoubleSupplier prepSpeedSupplier = () -> 0;
    TunableSlotConfigs configs;
    SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                    new Config(
                            Units.Volts.of(3).per(Units.Second),
                            Units.Volts.of(15),
                            Units.Seconds.of(8),
                            ModifiedSignalLogger.logState("shooter routine")),
                    new Mechanism(
                            (Voltage volts) -> setVoltage(volts.in(Units.Volt)),
                            null,
                            this,
                            "Shooter"));

    public Shooter(
            TalonFX master,
            TalonFX slave,
            TalonFX slave1,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(List.of(master), velocityConversion, positionConversion, nominalVoltage, kDt);
        this.addFollower(slave, MotorAlignmentValue.Opposed);
        this.addFollower(slave1, MotorAlignmentValue.Aligned);
        configs =
                new TunableSlotConfigs(
                        master,
                        0,
                        "Shooter KV Tunable",
                        SlotConfigs.from(ShooterConstants.kShooterConfig.Slot0));
    }

    public double getIdleSpeed() {
        double baseIdle = isInMidSupplier.getAsBoolean() ? idleSpeedInMid : idleSpeed;
        if (prepping) {
            // never idle slower than the base idle, even if the shot-speed lookup is lower
            return Math.max(prepSpeedSupplier.getAsDouble(), baseIdle);
        }
        return baseIdle;
    }

    public void setInMidSupplier(BooleanSupplier supplier) {
        this.isInMidSupplier = supplier;
    }

    /** Supplies the target shot speed used while {@link #startPrepIdle()} is active. */
    public void setPrepSpeedSupplier(DoubleSupplier supplier) {
        this.prepSpeedSupplier = supplier;
    }

    public void startPrepIdle() {
        prepping = true;
    }

    public void stopPrepIdle() {
        prepping = false;
    }

    public void disableIdle() {
        idleSpeedInMid = 0;
        idleSpeed = 0;
    }

    public void enableIdle() {
        idleSpeedInMid = ShooterConstants.kIdleSpeedMid;
        idleSpeed = ShooterConstants.kIdleSpeed;
    }

    public void setOpenLoop(double percent) {
        super.setOpenloop(percent);
    }

    public double getMasterCurrent() {
        return super.getMasterCurrent();
    }

    public void setVelocity(double velocity) {
        super.setVelocity(velocity, 0);
    }

    public void setVelocityVoltage(double velocity) {
        super.setVelocityVoltage(velocity, 0);
    }

    public void setVelocityFOC(double velocity) {
        super.setVelocityFOC(velocity, 0);
    }

    public void setVelocityDutyCycleBangBang(double velocity) {
        super.setVelocity(velocity, 0, 1);
    }

    public void setVelocityFOCBangBang(double velocity) {
        super.setVelocityFOC(velocity, 0, 1);
    }

    @Override
    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    @Override
    public String getName() {
        return "Shooter";
    }

    public Command getSysIdDynamic() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command getSysIdQuasiStatic() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command getSysIdDynamicReverse() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }

    public Command getSysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }
}
