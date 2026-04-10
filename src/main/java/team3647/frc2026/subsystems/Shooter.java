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

import team3647.frc2026.constants.ShooterConstants;
import team3647.lib.ModifiedSignalLogger;
import team3647.lib.TalonFXSubsystem;
import team3647.lib.tunables.TunableSlotConfigs;

public class Shooter extends TalonFXSubsystem {

    public double idleSpeed = 10;
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
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(List.of(master), velocityConversion, positionConversion, nominalVoltage, kDt);
        this.addFollower(slave, MotorAlignmentValue.Opposed);
        configs =
                new TunableSlotConfigs(
                        master,
                        0,
                        "Shooter KV Tunable",
                        SlotConfigs.from(ShooterConstants.kShooterConfig.Slot0));
    }

    public double getIdleSpeed() {
        return idleSpeed;
    }

    public void increaseIdleSpeed() {
        idleSpeed = 12;
    }

    public void decreaseIdleSpeed() {
        idleSpeed = 10;
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
