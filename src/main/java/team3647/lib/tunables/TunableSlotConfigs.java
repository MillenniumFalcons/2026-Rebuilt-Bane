package team3647.lib.tunables;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import team3647.lib.team6328.VirtualSubsystem;

public class TunableSlotConfigs extends VirtualSubsystem{

    private final TalonFXConfigurator configurator;
    private final SlotConfigs configs;

    private final TunableNumber kP, kI, kD, kS, kV, kA, kG;

    public TunableSlotConfigs(TalonFX fx, int slot, String name, SlotConfigs initialConfigs) {
        configurator = fx.getConfigurator();
        if (initialConfigs != null) {
            configs = initialConfigs;
        } else {
            configs = new SlotConfigs();
        }
        configs.SlotNumber = slot;

        String baseKey = "Tuning/" + name + "/slot" + slot;

        kP = new TunableNumber(baseKey + "/kP", configs.kP);
        kI = new TunableNumber(baseKey + "/kI", configs.kI);
        kD = new TunableNumber(baseKey + "/kD", configs.kD);
        kS = new TunableNumber(baseKey + "/kS", configs.kS);
        kV = new TunableNumber(baseKey + "/kV", configs.kV);
        kA = new TunableNumber(baseKey + "/kA", configs.kA);
        kG = new TunableNumber(baseKey + "/kG", configs.kG);
    }

    public void update() {
        if (hasChanged()) {
            configs.withKP(kP.get())
                    .withKI(kI.get())
                    .withKD(kD.get())
                    .withKS(kS.get())
                    .withKV(kV.get())
                    .withKA(kA.get())
                    .withKG(kG.get());

            configurator.apply(configs);
        }
    }

    private boolean hasChanged() {
        return kP.hasChanged()
                || kI.hasChanged()
                || kD.hasChanged()
                || kS.hasChanged()
                || kV.hasChanged()
                || kA.hasChanged()
                || kG.hasChanged();
    }
    
    @Override
    public void periodic() {
        update();
    }
}
