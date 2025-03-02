package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.LimelightHelpers;

public class LedSubsystem extends SubsystemBase {
    private AddressableLED addressableLED = new AddressableLED(2);
    private AddressableLEDBuffer addressableLEDBuffer = new AddressableLEDBuffer(20);

    // Flag to toggle between red/orange flame and bluish flame
    private boolean useBlueFlame = false;

    // Continuous ray position for smooth movement
    private double rayPosition = 0.0;
    // Speed in LED indices per second
    private double raySpeed = 18.0;

    // Timer for delta time calculations
    private Timer timer = new Timer();
    private double lastTime = 0;

    public LedSubsystem() {
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
        timer.start();
        lastTime = timer.get();
    }

    @Override
    public void periodic() {
        // Update flame mode based on target count or robot drive speeds
        if (LimelightHelpers.getTargetCount("") < 1) {
            useBlueFlame = true;
        } else {
            useBlueFlame = !RobotContainer.coralMode;
        }
    }

    /**
     * Smoothly updates the "shooting ray" animation.
     * The ray moves continuously along the LED strip with a fading tail.
     */
    public void updateShootingRayEffect() {
        double currentTime = timer.get();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        int ledCount = addressableLEDBuffer.getLength();
        // Update the ray position using delta time for smooth movement
        rayPosition = (rayPosition + raySpeed * deltaTime) % ledCount;

        // Tail width (in LED indices) controlling the gradient length
        double tailWidth = 5.0;

        // Loop through each LED in the strip
        for (int i = 0; i < ledCount; i++) {
            // Compute the distance to the ray position, accounting for wrap-around
            double distance = Math.abs(i - rayPosition);
            if (distance > ledCount / 2.0) {
                distance = ledCount - distance;
            }

            // Compute brightness with a linear fall-off (a simple gradient)
            double brightness = 0.0;
            if (distance < tailWidth) {
                brightness = 1.0 - (distance / tailWidth);
            }

            // (Optional) Uncomment to add a slight flicker effect
            // brightness *= (0.9 + 0.1 * Math.random());

            int r, g, b;
            if (useBlueFlame) {
                r = 0;
                g = 0;
                b = (int)(255 * brightness);
            } else {
                r = (int)(255 * brightness);
                // Add a small green component to achieve an orange hue
                g = (int)(50 * brightness);
                b = 0;
            }
            addressableLEDBuffer.setRGB(i, r, g, b);
        }

        // Push the updated buffer to the LED strip
        addressableLED.setData(addressableLEDBuffer);
    }
}