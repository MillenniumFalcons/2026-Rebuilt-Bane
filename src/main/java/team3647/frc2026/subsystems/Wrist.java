package team3647.frc2026.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import java.util.List;
import team3647.frc2026.constants.WristConstants;
import team3647.lib.TalonFXSubsystem;

public class Wrist extends TalonFXSubsystem {

    private final Angle maxAngle;
    private final Angle minAngle;
    private final CANcoder cancoder;
    private final TalonFXConfigurator configurator;

    double kG, angle = 0;

    public Wrist(
            TalonFX master,
            CANcoder cancoder,
            Angle maxAngle,
            Angle minAngle,
            double positionConversion,
            double velocityConversion,
            double nominalVoltage,
            double kDt) {
        super(List.of(master), velocityConversion, positionConversion, nominalVoltage, kDt);

        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.cancoder = cancoder;
        this.configurator = master.getConfigurator();
    }

    public void setAngle(Angle angle) {
        super.setPositionExpoVoltage(angle.in(Degree), 0, false);
    }

    public void setAngleDegs(double angle) {
        super.setPositionExpoVoltage(angle, 0);
    }

    public void setAngleSlow(double angle) {
        super.setPositionExpoVoltage(angle, 0, 1);
    }

    public void setSlowConfigs() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        configurator.refresh(config);
        config.MotionMagic.MotionMagicCruiseVelocity = 100;
        config.MotionMagic.MotionMagicAcceleration = 100;
        configurator.apply(config);
    }

    public void setDefaultConfigs() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        configurator.refresh(config);
        config.MotionMagic.MotionMagicCruiseVelocity =
                WristConstants.maxVel / WristConstants.kNativeToDeg;
        config.MotionMagic.MotionMagicAcceleration =
                WristConstants.maxAccel / WristConstants.kNativeToDeg;
        configurator.apply(config);
    }

    public Angle getAngle() {
        return Degree.of(getPosition());
    }

    public void setCancoderAngle(Angle angle) {
        cancoder.setPosition(angle);
    }

    public void setEncoderAngle(Angle angle) {
        cancoder.setPosition(angle);
    }

    public void setEncoderAngle(double angleRads) {
        cancoder.setPosition(Radian.of(angleRads));
    }

    public double getAngleRads() {
        return getAngle().in(Radian);
    }

    public double getAngleDegs() {
        return getAngle().in(Degree);
    }

    public void setEncoderNative(double position) {
        super.setEncoderNative(position);
    }

    public Angle getMinAngle() {
        return minAngle;
    }

    public Angle getMaxAngle() {
        return maxAngle;
    }

    public boolean angleWithin(double lowBound, double highBound) {
        return getAngleRads() > lowBound && getAngleRads() < highBound;
    }

    public boolean angleReached(Angle angle, Angle tolerance) {
        return getAngle().minus(angle).abs(Degree) < tolerance.in(Degree);
    }

    @Override
    public void readPeriodicInputs() {
        super.readPeriodicInputs();
    }

    @Override
    public String getName() {
        return "Wrist";
    }
}
