package team3647.lib.tunables;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import team3647.lib.team6328.VirtualSubsystem;

public class TunableMotionMagicConfigs extends VirtualSubsystem{
    private final TalonFXConfigurator configurator;
    private final MotionMagicConfigs configs;

    private final TunableNumber kV, kA, cruiseVelocity, acceleration, jerk;

    public TunableMotionMagicConfigs(TalonFX fx, String name, MotionMagicConfigs initialConfigs) {
        configurator = fx.getConfigurator();
        if (initialConfigs != null) {
            configs = initialConfigs;
        } else {
            configs = new MotionMagicConfigs();
        }

        String baseKey = "Tuning/" + name + "/MotionMagic";

        kV = new TunableNumber(baseKey + "/kV", configs.MotionMagicExpo_kV);
        kA = new TunableNumber(baseKey + "/kA", configs.MotionMagicExpo_kA);
        cruiseVelocity =
                new TunableNumber(baseKey + "/CruiseVelocity", configs.MotionMagicCruiseVelocity);
        acceleration =
                new TunableNumber(baseKey + "/Acceleration", configs.MotionMagicAcceleration);
        jerk = new TunableNumber(baseKey + "/Jerk", configs.MotionMagicJerk);
    }

    public void update() {
        if (hasChanged()) {
            configs.withMotionMagicExpo_kV(kV.get())
                    .withMotionMagicExpo_kA(kA.get())
                    .withMotionMagicCruiseVelocity(cruiseVelocity.get())
                    .withMotionMagicAcceleration(acceleration.get())
                    .withMotionMagicJerk(jerk.get());

            configurator.apply(configs);
        }
    }

    public void updateKvKaOnly() {
        if (kV.hasChanged() || kA.hasChanged()) {
            configs.withMotionMagicExpo_kV(kV.get()).withMotionMagicExpo_kA(kA.get());

            configurator.apply(configs);
        }
    }

    private boolean hasChanged() {
        return kV.hasChanged()
                || kA.hasChanged()
                || cruiseVelocity.hasChanged()
                || acceleration.hasChanged()
                || jerk.hasChanged();
    }
    
    @Override
    public void periodic() {
        update();
    }
}
