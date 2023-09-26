package org.team100.lib.sensors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;

/**
 * The TI ADS1015 is a 12-bit 4-channel ADC with I2C interface.
 * 
 * This is mostly cribbed from SparkFun and Adafruit:
 * 
 * https://github.com/sparkfun/SparkFun_ADS1015_Arduino_Library
 * https://github.com/adafruit/Adafruit_ADS1X15
 * 
 * See datasheet: https://www.ti.com/lit/ds/symlink/ads1015.pdf
 * 
 * Notable from the datasheet: ADS1015 is incapable of clock stretching (section
 * 9.1.1).
 * 
 * The ADS1015 contains a single ADC and a 4-way mux, so reading involves
 * configuring the mux, starting the measurement, waiting for completion, and
 * then reading the result.
 */
public class ADS1015 {
    /**
     * I2C address. 7-bit addr is 0x48, 8-bit addr is 0x91
     */
    private static final byte ADDR = (byte) 0x91;
    /**
     * Conversion register. See datasheet 8.6.2.
     */
    private static final byte CONV_REG = (byte) 0x00;
    /**
     * Config register. See datasheet 8.6.3.
     */
    private static final byte CFG_REG = (byte) 0x01;
    private final I2C m_i2c;

    public ADS1015() {
        this(ADDR);
    }

    private ADS1015(byte i2cAddress) {
        m_i2c = new I2C(I2C.Port.kMXP, i2cAddress >>> 1);
    }

    public int read(int channel) {
        if (channel > 3)
            throw new IllegalArgumentException(String.format("Illegal channel: %d", channel));
        startADCReading();
        while (busy()) {
            // TODO: time this out
        }
        return getLastConversionResult();

    }

    /** Start a single-shot measurement and return immediately. */
    private void startADCReading() {
        short config = 0;
        writeRegister(blarg, config);
    }

    /**
     * Tests the high bit of the config register to see if the device is currently doing a measurement.
     */
    private boolean busy() {
        return (readRegister(CFG_REG) & 0x8000) == 0;
    }

    /**
     * Read the result.
     * 
     * The ADS1015 reads 12 bits into 16 left justified, so this shifts right by 4.
     */
    private int getLastConversionResult() {
        return readRegister(CONV_REG) >>> 4;
    }

    private int readRegister(byte register) {
        ByteBuffer buf = ByteBuffer.allocate(2);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(register, 2, buf);
        return buf.getShort();
    }

    /**
     * Write two bytes to the specified register
     */
    private void writeRegister(byte register, short value) {
        ByteBuffer buf = ByteBuffer.allocate(3);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        buf.put(register);
        buf.putShort(value);
        m_i2c.writeBulk(buf, 3);
    }
}
