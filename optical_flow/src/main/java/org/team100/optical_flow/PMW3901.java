package org.team100.optical_flow;

import java.util.HexFormat;

import edu.wpi.first.wpilibj.SPI;

/**
 * Optical flow sensor PMW3901
 * 
 * Based on Pimoroni's python code:
 * 
 * https://github.com/pimoroni/pmw3901-python
 *
 * and Bizcraze's Arduino code:
 * 
 * https://github.com/bitcraze/Bitcraze_PMW3901
 * 
 * Sensor:
 * 
 * https://www.pixart.com/products-detail/44/PMW3901MB-TXQT
 * 
 * Datasheet:
 * 
 * https://wiki.bitcraze.io/_media/projects:crazyflie2:expansionboards:pot0189-pmw3901mb-txqt-ds-r1.00-200317_20170331160807_public.pdf
 */
public class PMW3901 {
    public static class Velocities {
        public double x;
        public double y;
    }

    private final SPI m_spi;

    public PMW3901(SPI.Port port) {
        m_spi = new SPI(port);
        init();
    }

    public void ledOn() {
        write("7f14" +
                "6f1c" +
                "7f00");
    }

    public void ledOff() {
        write("7f14" +
                "6f00" +
                "7f00");
    }

    /**
     * This is the "power on setup" thing that the datasheet mentions, without
     * justification.
     */
    private void init() {
        write("7f00" +
                "61ad" +
                "7f03" +
                "4000" +
                "7f05" +
                "41b3" +
                "43f1" +
                "4514" +
                "5b32" +
                "5f34" +
                "7b08" +
                "7f06" +
                "441b" +
                "40bf" +
                "4e3f");
    }

    private void write(String bytesString) {
        byte[] buf = HexFormat.of().parseHex(bytesString);
        int result = m_spi.write(buf, buf.length);
        if (result < 0) {
            throw new RuntimeException("blarg blarg blarg");
        }

    }

    public Velocities getMotion() {
        
        return new Velocities();
    }

}
