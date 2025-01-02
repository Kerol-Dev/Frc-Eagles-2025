package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;

public class CustomCommandRunner extends Command {
    private final Command onEnd;
    private final Command toRun;

    public CustomCommandRunner(Command toRun, Command onEnd) {
        this.onEnd = onEnd;
        this.toRun = toRun;
    }

    @Override
    public void initialize() {
        toRun.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.schedule();
    }
}
