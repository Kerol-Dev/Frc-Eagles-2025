package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.LimelightHelpers;

public class LedSubsystem extends SubsystemBase {
    private static AddressableLED addressableLED = new AddressableLED(2);
    private static AddressableLEDBuffer addressableLEDBuffer = new AddressableLEDBuffer(20);

    // 4) Flag to toggle between red/orange flame or bluish flame
    private static boolean useBlueFlame = false;

    // Variable to control the progress of the shooting ray effect
    private static int rayProgress = 0;

    // Timer to control the speed of the ray movement
    private static Timer timer = new Timer();
    private static double lastTime = 0;

    public LedSubsystem() {
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
        timer.start();  // Start the timer when the subsystem is initialized
    }

    @Override
    public void periodic() {
        if (LimelightHelpers.getTargetCount("") < 1 || (RobotContainer.m_robotDrive.getSpeeds().vyMetersPerSecond > 1 && RobotContainer.m_robotDrive.getSpeeds().vxMetersPerSecond > 1 && RobotContainer.m_robotDrive.getSpeeds().omegaRadiansPerSecond > 1)) {
            useBlueFlame = true;
        }
        else {
            useBlueFlame = !RobotContainer.coralMode;
        }

        // Update ray progress (this could be adjusted for speed of the effect)
        rayProgress = (rayProgress + 1) % addressableLEDBuffer.getLength(); // Loop the ray effect
    }

    /**
     * Updates the "shooting ray" animation each frame.
     * The ray moves from the back (off) to the front (colored).
     * Remember to call addressableLED.setData(addressableLEDBuffer) afterward.
     */
    public static void updateShootingRayEffect() {
        // Only proceed if sufficient time has passed to update the ray position
        if (timer.get() - lastTime >= 0.09) { // Update every 100 ms (adjust the speed as needed)
            lastTime = timer.get(); // Update the last time

            // Clear the buffer first
            for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
                addressableLEDBuffer.setRGB(i, 0, 0, 0);  // Turn off all LEDs initially
            }

            // Create the "shooting ray" effect by coloring a few LEDs at a time
            for (int i = rayProgress; i < rayProgress + 14; i++) {
                int index = i % addressableLEDBuffer.getLength(); // Wrap around the LED strip
                addressableLEDBuffer.setRGB(index, 
                    useBlueFlame ? 0 : 255,   // Red (0 if blue, 255 if red)
                    0,                         // Green (always 0)
                    useBlueFlame ? 255 : 0);   // Blue (255 if blue, 0 if red)
            }

            // Update the LEDs with the new data
            addressableLED.setData(addressableLEDBuffer);

            // Move the ray forward
            rayProgress = (rayProgress + 1) % addressableLEDBuffer.getLength(); // Loop the ray effect
        }
    }
}
