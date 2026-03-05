package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2026.subsystems.Indexer;

public class IndexerCommands {
    Indexer indexer;

    public IndexerCommands(Indexer indexer) {
        this.indexer = indexer;
    }

    public Command index() {
        return Commands.run(() -> indexer.setOpenLoop(-0.7), indexer);
    }

    public Command reverse() {
        return Commands.run(() -> indexer.setOpenLoop(0.7), indexer);
    }

    public boolean currentGreater(double current) {
        return indexer.getMasterCurrent() >= current;
    }

    public Command kill() {
        return Commands.runOnce(() -> indexer.setOpenLoop(0), indexer);
    }
}
