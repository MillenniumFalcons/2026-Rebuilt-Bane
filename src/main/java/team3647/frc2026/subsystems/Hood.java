package team3647.frc2026.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import java.util.List;
import team3647.lib.TalonFXSubsystem;

public class Hood extends TalonFXSubsystem {

    private CANcoder canCoder;

    private final Angle maxAngle;
    private final Angle minAngle;

    double kG, angle = 0;

    public Hood(
            TalonFX master,
            CANcoder canCoder,
            Angle maxAngle,
            Angle minAngle,
            double positionConversion,
            double velocityConversion,
            double nominalVoltage,
            double kDt) {
        super(List.of(master), velocityConversion, positionConversion, nominalVoltage, kDt);

        this.canCoder = canCoder;
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
    }

    public void setAngle(Angle angle) {
        super.setPositionExpoVoltage(angle.in(Degree), 0, false);
    }

    public void setAngleDegs(double angle) {
        super.setPositionExpoVoltage(angle, 0);
    }

    public Angle getAngle() {
        return Degree.of(getPosition());
    }

    public void setEncoderAngle(Angle angle) {
        this.setEncoder(angle.in(Degree));
    }

    public void setEncoderAngle(double angleDegs) {
        this.setEncoder(angleDegs);
    }

    public void setEncoderAngleCANcoder() {
        this.setEncoder(72 - canCoder.getPosition().getValueAsDouble() * 360);
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
        return "Hood";
    }
}
