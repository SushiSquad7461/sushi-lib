package SushiFrcLib.Control;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import SushiFrcLib.SmartDashboard.PIDTuning;

public class PIDConfig {
   public final double P;
   public final double I;
   public final double D;
   public final double F;
   
   public PIDConfig(double p, double i, double d, double f) {
    this.P = p;
    this.I = i;
    this.D = d;
    this.F = f;
   }

   public PIDConfig(double p, double d, double f) { this(p, 0.0, d, f); }

   public PIDConfig(double p, double f) { this(p, 0.0, 0.0, f); }

   public PIDConfig(double p) { this(p, 0.0, 0.0, 0.0); }

   public static PIDConfig getZeroPid() { return new PIDConfig(0.0, 0.0, 0.0, 0.0); }


   public void setPid(WPI_TalonFX talon) {
    talon.config_kP(0, P);
    talon.config_kI(0, I);
    talon.config_kD(0, D);
    talon.config_kF(0, F);
   }

   public void setPid(CANSparkMax sparkMax) {
    sparkMax.getPIDController().setP(P);
    sparkMax.getPIDController().setI(I);
    sparkMax.getPIDController().setD(D);
    sparkMax.getPIDController().setFF(F);
   }
}
