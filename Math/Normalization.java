package SushiFrcLib.Math;

import java.lang.Math;

public class Normalization {
  public static double cube(double triggerVal) {
    return Math.pow(triggerVal, 3);
  }
  
  public static double linearDeadzone(double triggerVal, double deadzone) {
    return ((Math.abs(triggerVal) > deadzone) ? triggerVal : 0);
  }
  
  public static double logistic(double triggerVal, double k) {
    return 2/(1 + (Math.pow((Math.E), -k * (triggerVal)))) - 1;
  }
}