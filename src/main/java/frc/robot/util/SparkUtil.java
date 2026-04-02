package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkUtil {
  /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(
      SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        sparkStickyFault = true;
        return;
      }
    }
    consumer.accept(values);
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }

  public static void tryToSetPidfUntillOkTune(SparkBase spark, int maxAttempts, SparkPIDF pidf, ClosedLoopSlot slot) {
    SparkBaseConfig config;

    if (spark instanceof SparkFlex) {
      config = new SparkFlexConfig();
    } else if (spark instanceof SparkMax) {
      config = new SparkMaxConfig();
    } else {
      throw new IllegalArgumentException("Unknown SparkBase type!");
    }

    pidf.applyTo(config, slot);

    tryUntilOk(spark, maxAttempts, () -> spark.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters));
  }

  public static void tryToSetPidfUntillOkTune(SparkBase spark, int maxAttempts, SparkPIDF pidf) {
    tryToSetPidfUntillOkTune(spark, maxAttempts, pidf, ClosedLoopSlot.kSlot0);
  }

  public static void tryToSetPidfUntillOkPermanent(SparkBase spark, int maxAttempts, SparkPIDF pidf,
      ClosedLoopSlot slot) {
    SparkBaseConfig config;

    if (spark instanceof SparkFlex) {
      config = new SparkFlexConfig();
    } else if (spark instanceof SparkMax) {
      config = new SparkMaxConfig();
    } else {
      throw new IllegalArgumentException("Unknown SparkBase type!");
    }

    pidf.applyTo(config, slot);

    tryUntilOk(spark, maxAttempts, () -> spark.configure(
        config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters));
  }

  public static void tryToSetPidfUntillOkPermanent(SparkBase spark, int maxAttempts, SparkPIDF pidf) {
    tryToSetPidfUntillOkPermanent(spark, maxAttempts, pidf, ClosedLoopSlot.kSlot0);
  }
}
