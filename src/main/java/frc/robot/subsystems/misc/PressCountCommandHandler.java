package frc.robot.subsystems.misc;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PressCountCommandHandler {
    private int pressCount = 0; // Counts button presses
    private double lastPressTime = 0.0; // Tracks the time of the last button press
    private final double debounceTime; // Debounce time in seconds
    private final Command[] commands; // Array of commands for each press count

    public PressCountCommandHandler(double debounceTime, Command... commands) {
        this.debounceTime = debounceTime;
        this.commands = commands;
    }

    public void recordPress() {
        pressCount++;
        lastPressTime = Timer.getFPGATimestamp(); // Update the time of the last press
    }

    public void executeIfDebounced() {
        // Check if debounce time has passed
        if (pressCount > 0 && Timer.getFPGATimestamp() - lastPressTime > debounceTime) {
            // Execute the command corresponding to the press count
            if (pressCount <= commands.length) {
                CommandScheduler.getInstance().schedule(commands[pressCount - 1]);
            } else {
                System.out.println("No command mapped for press count: " + pressCount);
            }

            // Reset the press count
            pressCount = 0;
        }
    }
}
