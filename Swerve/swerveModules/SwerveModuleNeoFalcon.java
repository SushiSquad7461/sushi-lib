package SushiFrcLib.Swerve.swerveModules;
import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Swerve.SwerveModuleConstants;
import SushiFrcLib.Swerve.SwerveModulePosition;
import SushiFrcLib.Swerve.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleNeoFalcon extends SwerveModule {
    private final TalonFX driveMotor;
    private final SwerveModuleConstants moduleConstants;
    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkMaxPIDController anglePID;

    private final CANCoder canCoder;

    private double lastAngle;

    public SwerveModuleNeoFalcon(int moduleNumber, SwerveModuleConstants constants) {
        super(moduleNumber, constants);

        this.moduleConstants = constants;

        driveMotor = MotorHelper.createFalconMotor(moduleConstants.driveMotorId, 
                moduleConstants.driveCurrentLimit,
                moduleConstants.driveInversion, moduleConstants.driveNeutralMode, 
                moduleConstants.driveP, moduleConstants.driveI, moduleConstants.driveD,
                moduleConstants.driveF,
                moduleConstants.canivoreName, SensorInitializationStrategy.BootToZero, 
                moduleConstants.openLoopRamp
        );
        driveMotor.setSelectedSensorPosition(0);

        angleMotor = new CANSparkMax(constants.angleMotorId, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();

        canCoder = new CANCoder(constants.cancoderId);

        configureDevices();
        lastAngle = getState().angle.getRadians();
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        // Prevents angle motor from turning further than it needs to. 
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        // state = SwerveModuleState.optimize(state, getState().angle); <-- BROKEN

        double targetAngle = state.angle.getRadians();
        double targetSpeed = state.velocity;

        driveMotor.set(ControlMode.Velocity, Conversion.MPSToFalcon(
            targetSpeed, 
            moduleConstants.wheelCircumference,
            moduleConstants.driveGearRatio
        ));

        double angle = Math.abs(state.velocity) <= moduleConstants.maxSpeed * 0.01
            ? lastAngle
            :  targetAngle;

        anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
        lastAngle = angle;

        SmartDashboard.putNumber("Current Mod Encoder Angle: " + moduleNumber, Rotation2d.fromRadians(angleEncoder.getPosition()).getDegrees());
    }

    public double getAngle() {
        return -getCanCoder().getDegrees() + angleOffset;
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
    }

    @Override
    public SwerveModulePosition getPose() {
        double distance = Conversion.falconToM(
            driveMotor.getSelectedSensorPosition(), moduleConstants.wheelCircumference,
            moduleConstants.driveGearRatio
        );        Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
        return new SwerveModulePosition(distance, rot);
    }

    public SwerveModulePosition getState() {
        double velocity = Conversion.falconToM(
            driveMotor.getSelectedSensorPosition(), moduleConstants.wheelCircumference,
            moduleConstants.driveGearRatio
        );

        Rotation2d angle = Rotation2d.fromRadians(angleEncoder.getPosition());

        return new SwerveModulePosition(velocity, angle);
    }

    

    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(Units.degreesToRadians(getAngle())); 
    }


    private void configureDevices() {

        // Angle motor configuration.
        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(moduleConstants.angleInversion);
        angleMotor.setIdleMode(moduleConstants.angleIdleMode);
        angleMotor.setSmartCurrentLimit(moduleConstants.angleCurrentLimit);

        anglePID.setP(moduleConstants.angleP);
        anglePID.setI(moduleConstants.angleI);
        anglePID.setD(moduleConstants.angleD);
        anglePID.setFF(moduleConstants.angleF);

        anglePID.setPositionPIDWrappingEnabled(true);
        anglePID.setPositionPIDWrappingMaxInput(2 * Math.PI);
        anglePID.setPositionPIDWrappingMinInput(0);

        angleEncoder.setPositionConversionFactor(moduleConstants.angleRotationsToRadians);
        angleEncoder.setVelocityConversionFactor(moduleConstants.angleRPMToRadiansPerSecond);
        resetToAbsolute();

        // CanCoder configuration.
        canCoder.configFactoryDefault();
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        canCoder.configSensorDirection(moduleConstants.cancoderInversion);
        canCoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        );
    }
}