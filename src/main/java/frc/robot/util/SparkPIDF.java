package frc.robot.util;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig;

/**
 * A comprehensive configuration class for SPARK motor controller Closed-Loop
 * parameters.
 * Includes all standard PID gains, the updated kS/kV/kA/kG/kCos feedforward
 * terms,
 * and the new MAXMotion parameters introduced in the REVLib 2025/2026 API.
 */
public class SparkPIDF {
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;

    // Feedforwards
    public double kS = 0.0; // Static Friction Feedforward
    public double kV = 0.0; // Velocity Feedforward
    public double kA = 0.0; // Acceleration Feedforward
    public double kG = 0.0; // Linear Gravity Feedforward (Elevators)

    // Cosine Gravity Feedforward (Arms)
    public double kCos = 0.0;
    public double kCosRatio = 0.0;

    // Integral Constraints
    public double iZone = 0.0;
    public double iMaxAccum = 0.0;

    // Output Constraints
    public double minOutput = -1.0;
    public double maxOutput = 1.0;

    // MAXMotion Parameters
    public double maxMotionCruiseVelocity = 0.0;
    public double maxMotionMaxAcceleration = 0.0;
    public double maxMotionAllowedProfileError = 0.0;

    /**
     * Default constructor with zeroed gains.
     */
    public SparkPIDF() {
    }

    // --- Basic Gain Builder Methods ---

    public SparkPIDF withP(double kP) {
        this.kP = kP;
        return this;
    }

    public SparkPIDF withI(double kI) {
        this.kI = kI;
        return this;
    }

    public SparkPIDF withD(double kD) {
        this.kD = kD;
        return this;
    }

    // --- Feedforward Builder Methods ---

    public SparkPIDF withS(double kS) {
        this.kS = kS;
        return this;
    }

    public SparkPIDF withV(double kV) {
        this.kV = kV;
        return this;
    }

    public SparkPIDF withA(double kA) {
        this.kA = kA;
        return this;
    }

    /**
     * Sets the linear gravity feedforward. (DO NOT use together with kCos)
     */
    public SparkPIDF withG(double kG) {
        this.kG = kG;
        return this;
    }

    /**
     * Sets the cosine gravity feedforward for rotating joints (arms).
     * (DO NOT use together with kG)
     * * @param kCos The gravity feedforward gain (Volts).
     * 
     * @param kCosRatio Relates the encoder position to absolute horizontal.
     */
    public SparkPIDF withCos(double kCos, double kCosRatio) {
        this.kCos = kCos;
        this.kCosRatio = kCosRatio;
        return this;
    }

    // --- Constraints & MAXMotion Builder Methods ---

    public SparkPIDF withIZone(double iZone) {
        this.iZone = iZone;
        return this;
    }

    public SparkPIDF withIMaxAccum(double iMaxAccum) {
        this.iMaxAccum = iMaxAccum;
        return this;
    }

    public SparkPIDF withOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        return this;
    }

    /**
     * Sets up the MAXMotion limits for smooth profiled motion.
     * * @param cruiseVelocity The maximum cruising velocity (RPM or position
     * units/sec).
     * 
     * @param maxAcceleration     The maximum acceleration (RPM/sec or position
     *                            units/sec^2).
     * @param allowedProfileError Permissible error before profile resets.
     */
    public SparkPIDF withMAXMotion(double cruiseVelocity, double maxAcceleration, double allowedProfileError) {
        this.maxMotionCruiseVelocity = cruiseVelocity;
        this.maxMotionMaxAcceleration = maxAcceleration;
        this.maxMotionAllowedProfileError = allowedProfileError;
        return this;
    }

    /**
     * Applies this PIDF configuration directly to a SparkBaseConfig's closed loop
     * slot.
     * * @param config The SparkBaseConfig (SparkMaxConfig or SparkFlexConfig) to
     * apply to.
     * 
     * @param slot The ClosedLoopSlot (e.g., kSlot0) to configure.
     * @return The updated SparkBaseConfig for method chaining.
     */
    public SparkBaseConfig applyTo(SparkBaseConfig config, ClosedLoopSlot slot) {
        config.closedLoop
                .p(kP, slot)
                .i(kI, slot)
                .d(kD, slot)
                .outputRange(minOutput, maxOutput, slot)
                .iZone(iZone, slot)
                .iMaxAccum(iMaxAccum, slot);

        config.closedLoop.feedForward
                .kS(kS, slot)
                .kV(kV)
                .kA(kA)
                .kG(kG, slot)
                .kCos(kCos, slot)
                .kCosRatio(kCosRatio, slot);

        config.closedLoop.maxMotion
                .cruiseVelocity(maxMotionCruiseVelocity, slot)
                .maxAcceleration(maxMotionMaxAcceleration, slot)
                .allowedProfileError(maxMotionAllowedProfileError, slot);

        return config;
    }

    /**
     * Applies this PIDF configuration directly to a SparkBaseConfig's closed loop
     * slot.
     * * @param config The SparkBaseConfig (SparkMaxConfig or SparkFlexConfig) to
     * apply to.
     * 
     * @return The updated SparkBaseConfig for method chaining.
     */
    public SparkBaseConfig applyTo(SparkBaseConfig config) {
        applyTo(config, ClosedLoopSlot.kSlot0);

        return config;
    }
}