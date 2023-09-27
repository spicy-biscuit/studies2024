package org.team100.lib.sensors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.TimeoutException;

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
    private static final short ADS1X15_REG_CONFIG_MODE_SINGLE = (short) 0x0100;
    private static final short ADS1X15_REG_CONFIG_OS_SINGLE = (short) 0x8000;
    // TODO: add the rest in an enum
    private static final short RATE_ADS1015_1600SPS = (short) 0x0080;
    // TODO: add the rest in an enum
    private static final short ADS1X15_REG_CONFIG_PGA_6_144V = (short) 0x0000;
    // TODO: make this an enum
    private static final short ADS1X15_REG_CONFIG_MUX_SINGLE_0 = (short) 0x4000;
    private static final short ADS1X15_REG_CONFIG_MUX_SINGLE_1 = (short) 0x5000;
    private static final short ADS1X15_REG_CONFIG_MUX_SINGLE_2 = (short) 0x6000;
    private static final short ADS1X15_REG_CONFIG_MUX_SINGLE_3 = (short) 0x7000;
    private static final short[] MUX_BY_CHANNEL = {
            ADS1X15_REG_CONFIG_MUX_SINGLE_0, /// < Single-ended AIN0
            ADS1X15_REG_CONFIG_MUX_SINGLE_1, /// < Single-ended AIN1
            ADS1X15_REG_CONFIG_MUX_SINGLE_2, /// < Single-ended AIN2
            ADS1X15_REG_CONFIG_MUX_SINGLE_3 /// < Single-ended AIN3
    };
    private final I2C m_i2c;
    private final short m_dataRate;
    private final short m_gain;

    public ADS1015() {
        this(ADDR);
    }

    private ADS1015(byte i2cAddress) {
        m_i2c = new I2C(I2C.Port.kMXP, i2cAddress >>> 1);
        m_dataRate = RATE_ADS1015_1600SPS;
        m_gain = ADS1X15_REG_CONFIG_PGA_6_144V;
    }

    public int read(int channel) throws TimeoutException, InterruptedException {
        if (channel > 3)
            throw new IllegalArgumentException(String.format("Illegal channel: %d", channel));
        startADCReading(MUX_BY_CHANNEL[channel]);
        // wait for the ADC to finish reading; timeout after awhile.
        // default "data rate" is 1600 samples per sec so wait about 1ms.
        int maxBusy = 5;
        do {
            Thread.sleep(1);
            maxBusy -= 1;
            if (maxBusy < 0) {
                throw new TimeoutException();
            }
        } while (busy());

        return getLastConversionResult();

    }

    /** Start a single-shot measurement and return immediately. */
    private void startADCReading(short mux) {
        short config = 0;

        config |= ADS1X15_REG_CONFIG_MODE_SINGLE;

        // Set PGA/voltage range
        config |= m_gain;

        // Set data rate
        config |= m_dataRate;

        // Set channels
        config |= mux;

        // Set 'start single-conversion' bit
        config |= ADS1X15_REG_CONFIG_OS_SINGLE;

        writeRegister(CFG_REG, config);
    }

    /**
     * Tests the high bit of the config register to see if the device is currently
     * doing a measurement.
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
