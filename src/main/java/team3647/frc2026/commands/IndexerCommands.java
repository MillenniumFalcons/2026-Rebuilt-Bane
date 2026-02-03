package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2026.subsystems.Indexer;

public class IndexerCommands {
    Indexer indexer;

    public IndexerCommands(Indexer indexer) {
        this.indexer = indexer;
    }

    public Command setOpenLoop(double output) {
        return Commands.run(
                () -> {
                    indexer.setOpenLoop(output);
                    indexer.setOpenLoop(output);
                },
                indexer);
    }

    public boolean currentGreater(double current) {
        return indexer.getMasterCurrent() >= current;
    }

    public Command kill() {
        return setOpenLoop(0).withTimeout(0.3);
    }
}
