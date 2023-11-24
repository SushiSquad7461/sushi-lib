package SushiFrcLib.Motor;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import SushiFrcLib.Control.PIDConfig;

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

        public NeutralMode getTalonMode() {
            return mode ? NeutralMode.Coast : NeutralMode.Brake;
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

   public void setTalonConfig(WPI_TalonFX motor) {
        motor.setInverted(inversion);

        if (currentLimit < 0) {
            motor.configSupplyCurrentLimit(MotorHelper.createCurrentLimt(currentLimit));
        }

        motor.setNeutralMode(mode.getTalonMode());
        motor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

        pid.setPid(motor);
   }

   public WPI_TalonFX createTalon() {
        WPI_TalonFX motor = new WPI_TalonFX(canId, canBus);
        setTalonConfig(motor);
        return motor;
   }

   public void setCanSparkMaxConfig(CANSparkMax motor, CANSparkMaxLowLevel.MotorType type) {
        motor.restoreFactoryDefaults();
        motor.setInverted(inversion);

        if (currentLimit < 0) {
            motor.setSmartCurrentLimit(currentLimit);
        }

        motor.setIdleMode(mode.getSparkMaxMode());

        if (type == CANSparkMaxLowLevel.MotorType.kBrushless) {
            motor.getEncoder().setPosition(0);
        }

        pid.setPid(motor);
   }

   public CANSparkMax createSparkMax() { return createSparkMax(CANSparkMaxLowLevel.MotorType.kBrushless); }

   public CANSparkMax createSparkMax(CANSparkMaxLowLevel.MotorType type) {
        CANSparkMax motor = new CANSparkMax(canId, type);
        setCanSparkMaxConfig(motor, type);
        return motor;
   }
}
