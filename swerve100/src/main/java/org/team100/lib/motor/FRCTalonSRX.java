package org.team100.lib.motor;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/**
 * An abstraction for the Talon SRX for debugging information
 */
public class FRCTalonSRX {

    /**
     * A direct reference to the TalonSRX motor, designed for direct control
     */
    public WPI_TalonSRX motor;

    public SensorCollection m_sensorCollection;

    /**
     * The Can ID of the selected motor
     */
    private int canID;

    /**
     * The inversion of the motor
     *
     * true inverts the motor
     */
    private boolean inverted;

    /**
     * The inversion of the motor
     *
     * Uses CTRE InvertType
     */
    private InvertType invertType;

    /**
     * Determines what mode to use for inversion
     * 
     * true uses InvertType invertType
     * false uses boolean inverted
     */
    private boolean useInvertType;

    /**
     * The feedback port of the motor Default is 0
     */
    private int feedbackPort = 0;

    /**
     * The timeout of the motor in ms
     *
     * Default is 10
     */
    private int timeout = 10;

    /**
     * The sensor phase of the motor
     *
     * true inverts the encoder signal
     */
    private boolean sensorPhase;

    /**
     * The kP value of the motor's PID controller
     */
    private double kP;

    /**
     * The kI value of the motor's PID controller
     */
    private double kI;

    /**
     * The kD value of the motor's PID controller
     */
    private double kD;

    /**
     * The kF value of the motor's PID controller
     */
    private double kF;

    /**
     * The acceptable closed loop error in ticks
     */
    private int allowableClosedLoopError;

    /**
     * The type of status frame
     */
    private StatusFrameEnhanced statusFrameType;

    /**
     * The status frame of the motor
     */
    private int statusFrame;

    /**
     * Is a current limit enabled
     *
     * a currentLimit must be set if this is true
     */
    private boolean currentLimitEnabled;

    /**
     * The current limit set (amps)
     *
     * currentLimitEnabled must be set for this to activate
     */
    private int currentLimit;

    /**
     * The neutral mode of the motor controller
     */
    private NeutralMode neutralMode;


    /**
     * The forward peak output
     */
    private double peakOutputForward;

    /**
     * The reverse peak output
     */
    private double peakOutputReverse;


    /**
     * The measurement period for velocity control
     */
    private SensorVelocityMeasPeriod velocityMeasurementPeriod;

    /**
     * The measurement window for the velocity control
     */
    private int velocityMeasurementWindow;

    /**
     * Is a forward soft limit enabled
     */
    private boolean forwardSoftLimitEnabled;

    /**
     * The forward soft limit set
     */
    private int forwardSoftLimitThreshold;



    private boolean feedbackNotContinuous;



    public int getRawAnalogSensor() {
        return m_sensorCollection.getAnalogInRaw();
    }

    public FRCTalonSRX configure() {
        motor = new WPI_TalonSRX(this.canID);
        m_sensorCollection = motor.getSensorCollection();

        motor.configFactoryDefault();
        motor.setSafetyEnabled(false);

        if (this.inverted || this.invertType != InvertType.None) {
            if (this.useInvertType)
                motor.setInverted(this.invertType);
            else
                motor.setInverted(this.inverted);
            System.out.println("Configuring Inverted");
        }
        if (this.currentLimitEnabled) {
            motor.enableCurrentLimit(this.currentLimitEnabled);
            motor.configContinuousCurrentLimit(this.currentLimit);
            System.out.println("Configuring Current Limit");

        }
        if (this.feedbackNotContinuous) {
            motor.configFeedbackNotContinuous(this.feedbackNotContinuous, this.timeout);
            System.out.println("Configuring Feedback Continuity");
        }



        if (this.neutralMode != null) {
            motor.setNeutralMode(this.neutralMode);
            System.out.println("Setting Neutral Mode");
        }


  

        if (this.peakOutputForward != 0 || this.peakOutputReverse != 0) {
            motor.configPeakOutputForward(this.peakOutputForward);
            motor.configPeakOutputReverse(this.peakOutputReverse);
            System.out.println("Setting Peak Output");
        }


        if (this.sensorPhase) {
            motor.setSensorPhase(this.sensorPhase);
            System.out.println("setting sensor phase");

        }

        if (this.velocityMeasurementPeriod != null || this.velocityMeasurementWindow != 0) {
            motor.configVelocityMeasurementPeriod(this.velocityMeasurementPeriod);
            motor.configVelocityMeasurementWindow(this.velocityMeasurementWindow);
            System.out.println("Setting Velocity Measurement Period");
        }


  
        return this;
    }

    public static final class FRCTalonSRXBuilder {
        private final int canID;
        public boolean inverted = false;
        public boolean useInvertType = false;
        public boolean sensorPhase = false;
        public boolean currentLimitEnabled = false;
        public NeutralMode neutralMode = NeutralMode.Coast;
        private final String smartDashboardPath;
        public double peakOutputForward = 1.0;
        public double peakOutputReverse = -1.0;
        public FRCTalonSRXBuilder(int canID) {
            this.canID = canID;
            this.smartDashboardPath = "TalonSRX_" + canID;
        }

  

        public FRCTalonSRXBuilder withInverted(boolean inverted) {
            this.inverted = inverted;
            this.useInvertType = false;
            return this;
        }

  
  

        public FRCTalonSRXBuilder withSensorPhase(boolean sensorPhase) {
            this.sensorPhase = sensorPhase;
            return this;
        }





        public FRCTalonSRXBuilder withCurrentLimitEnabled(boolean currentLimitEnabled) {
            this.currentLimitEnabled = currentLimitEnabled;
            return this;
        }


        public FRCTalonSRXBuilder withNeutralMode(NeutralMode neutralMode) {
            this.neutralMode = neutralMode;
            return this;
        }



        public FRCTalonSRXBuilder withPeakOutputForward(double peakOutputForward) {
            this.peakOutputForward = peakOutputForward;
            return this;
        }

        public FRCTalonSRXBuilder withPeakOutputReverse(double peakOutputReverse) {
            this.peakOutputReverse = peakOutputReverse;
            return this;
        }

  
        public FRCTalonSRX build() {
            FRCTalonSRX fRCTalonSRX = new FRCTalonSRX();
            fRCTalonSRX.canID = canID;
            if (useInvertType) {
                fRCTalonSRX.invertType = InvertType.None;
                fRCTalonSRX.inverted = false;
                fRCTalonSRX.useInvertType = true;
            } else {
                fRCTalonSRX.inverted = inverted;
                fRCTalonSRX.invertType = InvertType.None;
                fRCTalonSRX.useInvertType = false;
            }
            fRCTalonSRX.feedbackPort = 0;
            fRCTalonSRX.timeout = 10;
            fRCTalonSRX.sensorPhase = sensorPhase;
            fRCTalonSRX.allowableClosedLoopError = 0;
            fRCTalonSRX.statusFrameType = StatusFrameEnhanced.Status_3_Quadrature;
            fRCTalonSRX.statusFrame = 0;
            fRCTalonSRX.currentLimitEnabled = currentLimitEnabled;
            fRCTalonSRX.currentLimit = 0;
            fRCTalonSRX.neutralMode = neutralMode;


            fRCTalonSRX.peakOutputForward = peakOutputForward;
            fRCTalonSRX.peakOutputReverse = peakOutputReverse;

            fRCTalonSRX.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
            fRCTalonSRX.velocityMeasurementWindow = 64;
            fRCTalonSRX.forwardSoftLimitEnabled = false;
            fRCTalonSRX.forwardSoftLimitThreshold = 0;

 

            fRCTalonSRX.feedbackNotContinuous = false;
            fRCTalonSRX.kF = 0.0;
            fRCTalonSRX.kD = 0.0;
            fRCTalonSRX.kI = 0.0;
            fRCTalonSRX.kP = 0.0;
            return fRCTalonSRX.configure();

        }
    }
}