package SushiFrcLib.Motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import SushiFrcLib.Control.PIDConfig;
import SushiFrcLib.SmartDashboard.PIDTuning;

public class MotorConfig {
   public final PIDConfig pid;
   public final int canId;
   public final String canBus;
   public final int currentLimit; // A currentLimit of bellow zero will not be set on the motor
   public final boolean inversion;
   public final Mode mode;

   public enum Mode {
        COAST(true),
        BRAKE(false);

        private boolean mode;

        private Mode(boolean mode) {
            this.mode = mode;
        }

        public NeutralModeValue getTalonMode() {
            return mode ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        }

        public IdleMode getSparkMaxMode() {
            return mode ? IdleMode.kCoast : IdleMode.kBrake;
        }
   };

   public MotorConfig(int canId, String canBus, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) {
        this.canId = canId;
        this.canBus = canBus;
        this.currentLimit = currentLimit;
        this.inversion = inversion;
        this.pid = pid;
        this.mode = mode;
   }

   public MotorConfig(String canBus, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) { this(-1, canBus, currentLimit, inversion, pid, mode); }

   public MotorConfig(int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) { this(-1, "rio", currentLimit, inversion, pid, mode); }

   public MotorConfig(int canId, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) { this(canId, "rio", currentLimit, inversion, pid, mode); }

   public MotorConfig(int canId) { this(canId, "rio", -1, false, PIDConfig.getZeroPid(), Mode.COAST);}

   public MotorConfig(int canId, String canBus) { this(canId, canBus, -1, false, PIDConfig.getZeroPid(), Mode.COAST);}

   public MotorConfig(int canId, int currentLimit, Boolean inversion, Mode mode) { this(canId, "rio", currentLimit, inversion, PIDConfig.getZeroPid(), mode);}

   public MotorConfig(int canId, String canBus, int currentLimit, Boolean inversion, Mode mode) { this(canId, canBus, currentLimit, inversion, PIDConfig.getZeroPid(), mode);}

   public PIDTuning genPIDTuning(String motorName, boolean tuningMode) {
        return new PIDTuning(motorName, pid, tuningMode);
   }

   public TalonFXConfiguration getTalonConfig() {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        MotorHelper.updateSupplyCurrentLimit(currentLimit, talonConfig);
 
        talonConfig.MotorOutput.NeutralMode = mode.getTalonMode();
        talonConfig.MotorOutput.Inverted = inversion ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

        pid.updatePidConfig(talonConfig);

        return talonConfig;
   }

   public TalonFX createTalon() {
        TalonFX motor = new TalonFX(canId, canBus);
        motor.getConfigurator().apply(getTalonConfig());
        return motor;
   }

   public void setCanSparkMaxConfig(SparkMax motor, MotorType type) {
     SparkMaxConfig config = new SparkMaxConfig();
     config.inverted(inversion);
     config.smartCurrentLimit(currentLimit);
     config.idleMode(mode.getSparkMaxMode());
     config.closedLoop.apply(pid.createSparkMaxConfig());

     motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     if (type == MotorType.kBrushless) {
          motor.getEncoder().setPosition(0);
     }

   }

   public SparkMax createSparkMax() { return createSparkMax(MotorType.kBrushless); }

   public SparkMax createSparkMax(MotorType type) {
        SparkMax motor = new SparkMax(canId, type);
        setCanSparkMaxConfig(motor, type);
        return motor;
   }

   public MotorConfig withCanId(int canId) {
     return new MotorConfig(canId, this.canBus, this.currentLimit, this.inversion, this.pid, this.mode);
   }
}
