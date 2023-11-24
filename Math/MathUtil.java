package SushiFrcLib.Math;

import com.revrobotics.CANSparkMax;

import SushiFrcLib.Sensors.absoluteEncoder.AbsoluteEncoder;
import SushiFrcLib.SmartDashboard.TunableNumber;

public class MathUtil {
   public static double clamp(double max, double min, double currVal) {
        if (currVal > max) {
            return max;
        } else if (currVal < min) {
            return min;
        }

        return currVal;
   } 

   public static double clamp(double max, double min, TunableNumber currVal) {
    return clamp(max, min, currVal.get());
   } 

   public static double getError(double target, double setpoint) {
    return Math.abs(target - setpoint);
   }

   public static double getError(CANSparkMax target, double setpoint) {
    return getError(target.getEncoder().getPosition(), setpoint);
   }

   public static double getError(CANSparkMax target, TunableNumber setpoint) {
    return getError(target.getEncoder().getPosition(), setpoint.get());
   }

   public static double getError(CANSparkMax target, AbsoluteEncoder setpoint) {
    return getError(target.getEncoder().getPosition(), setpoint.getNormalizedPosition());
   }
}
