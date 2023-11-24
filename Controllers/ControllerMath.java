package SushiFrcLib.Controllers;

import java.lang.Math;

public class ControllerMath {
  public static double cube(double triggerVal) {
    return Math.pow(triggerVal, 3);
  }
  
  public static double applyDeadband(double initalVal, double deadband) {
    return Math.abs(initalVal) <  deadband ? 0 : (
        (initalVal - ((initalVal < 0 ? -1 : 1) * deadband)) 
        / (1 - deadband));
  }
  
  public static double logistic(double triggerVal, double k) {
    return 2/(1 + (Math.pow((Math.E), -k * (triggerVal)))) - 1;
  }
}