package team3647.frc2026.subsystems;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.List;
import team3647.frc2026.constants.ShooterConstants;
import team3647.lib.ModifiedSignalLogger;
import team3647.lib.TalonFXSubsystem;
import team3647.lib.tunables.TunableSlotConfigs;

public class ShooterLeft extends TalonFXSubsystem {

    TunableSlotConfigs configs;

    SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                    new Config(
                            Units.Volts.of(6).per(Units.Second),
                            Units.Volts.of(30),
                            Units.Seconds.of(8),
                            ModifiedSignalLogger.logState("left shooter routine")),
                    new Mechanism(
                            (Voltage volts) -> setVoltage(volts.in(Units.Volts)),
                            null,
                            this,
                            "Left Shooter"));

    public ShooterLeft(
            TalonFX master,
            TalonFX slave,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(List.of(master, slave), velocityConversion, positionConversion, nominalVoltage, kDt);
        // this.addFollower(slave, MotorAlignmentValue.Aligned);
        configs =
                new TunableSlotConfigs(
                        master,
                        0,
                        "Left Shooter KV Tunable",
                        SlotConfigs.from(ShooterConstants.kShooterOneConfig.Slot0));
    }

    public void setOpenLoop(double percent) {
        super.setOpenloop(percent);
    }

    public double getMasterCurrent() {
        return super.getMasterCurrent();
    }

    public void setVelocityVoltage(double velocity) {
        super.setVelocityVoltage(velocity, 0);
    }

    public void setVelocity(double velocity) {
        super.setVelocity(velocity, 0);
    }

    @Override
    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    @Override
    public String getName() {
        return "ShooterLeft";
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
