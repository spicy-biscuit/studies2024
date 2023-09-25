package org.team100.lib.sensors;

import edu.wpi.first.wpilibj.Timer;

public class AHRS {
    private final LSM6DSOX_I2C m_gyro;
    private double headingNWURad;
    private double previousTimeSec;

    public AHRS(LSM6DSOX_I2C gyro) {
        m_gyro = gyro;
        previousTimeSec = Timer.getFPGATimestamp();
    }

    public double getHeadingNWURad() {
        return headingNWURad;
    }

    /** Call this as often as possible. */
    public void update() {
        double newTimeSec = Timer.getFPGATimestamp();
        double dtSec = newTimeSec - previousTimeSec;
        double yawRateRadS = m_gyro.getYawRateRadS();
        double yawDeltaRad = yawRateRadS * dtSec;
        headingNWURad += yawDeltaRad;
    }

}
