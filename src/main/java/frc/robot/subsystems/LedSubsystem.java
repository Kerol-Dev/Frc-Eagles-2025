package frc.robot.subsystems;

import java.util.List;
import java.util.Random;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;

public class LedSubsystem extends SubsystemBase {
    private AddressableLED addressableLED = new AddressableLED(2);
    private AddressableLEDBuffer addressableLEDBuffer = new AddressableLEDBuffer(60);

    // 1) An array to store the "heat" of each LED pixel
    private int[] heat;

    // 2) Some parameters you can tune:
    private static final int COOLING = 55; // How quickly the fire cools (higher = faster cooling)
    private static final int SPARKING = 120; // Chance (out of 255) a new spark will light at bottom

    // 3) Random for spark/cooling
    private Random random = new Random();

    // 4) Flag to toggle between red/orange flame or bluish flame
    private boolean useBlueFlame = false;

    public LedSubsystem() {
        heat = new int[addressableLEDBuffer.getLength()];
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled())
            useBlueFlame = !RobotContainer.coralMode;
        else if (!VisionConstants.cameras.get(0).cameraObject.isConnected()
                || !VisionConstants.cameras.get(1).cameraObject.isConnected()
                || !VisionConstants.cameras.get(2).cameraObject.isConnected()
                || !VisionConstants.cameras.get(3).cameraObject.isConnected()) {
            useBlueFlame = true;
        } 
        else
        {
            int fineCams = 0;
            for (int i = 0; i < 4; i++) {
                List<PhotonPipelineResult> targResults = VisionConstants.cameras.get(i).cameraObject
                        .getAllUnreadResults();
                if (targResults.get(targResults.size() - 1).hasTargets())
                    fineCams++;
            }
            useBlueFlame = fineCams == 0;
        }
    }

    /**
     * Updates the "rising fire" animation each frame.
     * Remember to call addressableLED.setData(addressableLEDBuffer) afterward.
     */
    public void updateFireAnimation() {
        // 1. Cool down every cell a little
        for (int i = 0; i < heat.length; i++) {
            int cooldown = random.nextInt(((COOLING * 10) / addressableLEDBuffer.getLength()) + 2);
            heat[i] = Math.max(0, heat[i] - cooldown);
        }

        // 2. Heat from each cell drifts upward
        for (int k = heat.length - 1; k >= 2; k--) {
            heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
        }

        // 3. Randomly ignite new sparks near the bottom
        if (random.nextInt(255) < SPARKING) {
            int y = random.nextInt(Math.min(7, addressableLEDBuffer.getLength()));
            heat[y] = Math.min(255, heat[y] + random.nextInt(95) + 160);
        }

        // 4. Convert heat to LED colors
        for (int j = 0; j < heat.length; j++) {
            setPixelHeatColor(j, heat[j]);
        }
    }

    /**
     * Converts a "heat" value (0-255) into either a red/orange or bluish-white
     * color,
     * depending on whether we're using the 'useBlueFlame' mode.
     */
    private void setPixelHeatColor(int pixel, int temperature) {
        // Scale 'heat' down from 0-255 to 0-191
        int t192 = (temperature * 191) / 255;

        // calculate ramp up from 0 to 63
        int heatramp = t192 & 0x3F;
        heatramp <<= 2; // scale up to 0-252

        int r, g, b;

        if (!useBlueFlame) {
            // --- Original Red/Orange Flame Palette ---
            if (t192 > 128) {
                // hottest
                r = 255;
                g = 255;
                b = heatramp;
            } else if (t192 > 64) {
                // middle
                r = 255;
                g = heatramp;
                b = 0;
            } else {
                // coolest
                r = heatramp;
                g = 0;
                b = 0;
            }
        } else {
            // --- Bluish-White Flame Palette ---
            // We'll mirror the same "3 zones" logic but shift colors to blues/white
            if (t192 > 128) {
                // hottest => white with a bit of blue
                r = heatramp; // slight red tint
                g = 255; // green at max for white mix
                b = 255; // blue at max
            } else if (t192 > 64) {
                // middle => bright blue, but partial green
                r = 0;
                g = heatramp; // partial green for aqua
                b = 255; // strong blue
            } else {
                // coolest => mostly dim blue
                r = 0;
                g = 0;
                b = heatramp; // ramp in the blue channel
            }
        }

        // Write color to LED buffer
        addressableLEDBuffer.setRGB(pixel, r, g, b);
    }
}