package SushiFrcLib.Swerve.swerveModules;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Swerve.SwerveModuleConstants;
import SushiFrcLib.Swerve.SwerveModulePosition;
import SushiFrcLib.Swerve.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleNeo extends SwerveModule {
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;
    SwerveModuleConstants moduleConstants;

    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkMaxPIDController anglePID;

    private final CANCoder canCoder;

    private double lastAngle;

    public SwerveModuleNeo(int moduleNumber, SwerveModuleConstants constants) {
        super(moduleNumber, constants);

        this.moduleConstants = constants;

        driveMotor = new CANSparkMax(constants.driveMotorId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getPIDController();

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
        // state = SwerveModuleState.optimize(state, getState().angle);

        double targetAngle = state.angle.getRadians();

        // double targetSpeed = state.velocity;

        // double delta = targetAngle - lastAngle;

        // if (Math.abs(delta) > 90) {
        //     targetSpeed = -targetSpeed;
        //     targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);        
        // }

        drivePID.setReference(state.velocity, CANSparkMax.ControlType.kVelocity);

        double angle = Math.abs(state.velocity) <= moduleConstants.maxSpeed * 0.01
            ? lastAngle
            :  targetAngle;

        anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
        lastAngle = angle;

        SmartDashboard.putNumber("Current Mod Encoder Angle: " + moduleNumber, Rotation2d.fromRadians(angleEncoder.getPosition()).getDegrees());
    }

    public SwerveModuleState getState() {
        double velocity = driveEncoder.getVelocity();
        Rotation2d rot = Rotation2d.fromRadians(angleEncoder.getPosition());
        return new SwerveModuleState(velocity, rot);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
    }

    public double getAngle() {
        return -getCanCoder().getDegrees() + angleOffset;
    }

    @Override
    public SwerveModulePosition getPose() {
        double distance = driveEncoder.getPosition();
        Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
        return new SwerveModulePosition(distance, rot);
    }

    private void configureDevices() {
        // Drive motor configuration.
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(moduleConstants.driveInversion);
        driveMotor.setIdleMode(moduleConstants.driveIdleMode);
        driveMotor.setSmartCurrentLimit(moduleConstants.driveCurrentLimit);

        drivePID.setP(moduleConstants.driveP);
        drivePID.setI(moduleConstants.driveI);
        drivePID.setD(moduleConstants.driveD);
        drivePID.setFF(moduleConstants.driveF);

        driveEncoder.setPositionConversionFactor(moduleConstants.driveRotationsToMeters);
        driveEncoder.setVelocityConversionFactor(moduleConstants.driveRPMToMetersPerSecond);
        driveEncoder.setPosition(0);

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

    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(Units.degreesToRadians(getAngle())); 
    }
}
