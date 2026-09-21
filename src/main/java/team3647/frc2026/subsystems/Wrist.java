package team3647.frc2026.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import java.util.List;
import team3647.lib.TalonFXSubsystem;

public class Wrist extends TalonFXSubsystem {

    private final Angle maxAngle;
    private final Angle minAngle;
    private final CANcoder cancoder;

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
    }

    public void setAngle(Angle angle) {
        super.setPositionExpoVoltage(angle.in(Degree), 0, false);
    }

    public void setAngleDegs(double angle) {
        super.setPositionExpoVoltage(angle, 0);
    }

    public void setAngleSlow(double angle) {
        super.setPositionDynamicMM(angle, 0, 50, 50);
    }

    public void setSlowConfigs(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotionMagic.MotionMagicCruiseVelocity = 100;
        config.MotionMagic.MotionMagicAcceleration =
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
