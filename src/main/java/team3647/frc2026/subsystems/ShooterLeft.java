package team3647.frc2026.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import team3647.lib.ModifiedSignalLogger;
import team3647.lib.TalonFXSubsystem;

public class ShooterLeft extends TalonFXSubsystem {

    SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                    new Config(
                            null,
                            null,
                            null,
                            ModifiedSignalLogger.logState("left shooter routine")),
                    new Mechanism(
                            (Voltage volts) -> setVoltage(volts.in(Units.Volt)),
                            null,
                            this,
                            "Left Shooter"));

    public ShooterLeft(
            TalonFX motor,
            TalonFX slave,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(motor, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.addFollower(slave, MotorAlignmentValue.Aligned);
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
}
