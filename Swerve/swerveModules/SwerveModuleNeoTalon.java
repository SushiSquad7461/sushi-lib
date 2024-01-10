package SushiFrcLib.Swerve.SwerveModules;
import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Swerve.CTREModuleState;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleNeoTalon extends SwerveModule {
    private final TalonFX driveMotor;
    private final SwerveModuleConstants moduleConstants;
    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkPIDController anglePID;

    private Rotation2d lastAngle;

    private VelocityVoltage drivePID;

    public SwerveModuleNeoTalon(SwerveModuleConstants constants) {
        super(constants);

        this.moduleConstants = constants;

        driveMotor = constants.getDriveFalcon(); 

        angleMotor = constants.getAngleNeo();
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();

        lastAngle = getPose().angle;

        resetToAbsolute();

        drivePID = new VelocityVoltage(0,0,false,0,0,false,false,false); 
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        // Prevents angle motor from turning further than it needs to. 
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        state = CTREModuleState.optimize(state, lastAngle);

        driveMotor.setControl(
            drivePID.withVelocity(
                Conversion.MPSToFalcon(
                    state.speedMetersPerSecond, 
                    SwerveModuleConstants.wheelCircumference,
                    moduleConstants.moduleInfo.driveGearRatio
                )
            )
        );

        double angle = Math.abs(state.speedMetersPerSecond) <= moduleConstants.moduleInfo.driveGearRatio * 0.01
            ? lastAngle.getDegrees()
            :  state.angle.getDegrees();

        anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
        lastAngle = Rotation2d.fromDegrees(angle);

        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Target Drive Velocity: " + swerveModuleConstants.moduleNumber,  Conversion.MPSToFalcon(
                state.speedMetersPerSecond, 
                SwerveModuleConstants.wheelCircumference,
                moduleConstants.moduleInfo.driveGearRatio
            ));
            SmartDashboard.putNumber("Target Relative Encoder Angle " + swerveModuleConstants.moduleNumber, angle);
        }
    }

    @Override
    public SwerveModulePosition getPose() {
        double distance = Conversion.rotationsToM(
            driveMotor.getPosition().getValue(), SwerveModuleConstants.wheelCircumference,
            moduleConstants.moduleInfo.driveGearRatio
        );
        
        Rotation2d rot = Rotation2d.fromDegrees(angleEncoder.getPosition());

        return new SwerveModulePosition(distance, rot);
    }


    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(getCanCoder().getRotations()); 
        lastAngle = getCanCoder();
    }

    @Override
    public void log() {

        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Angle Current " + swerveModuleConstants.moduleNumber, angleMotor.getOutputCurrent());
            SmartDashboard.putNumber("Current Reltaive Encoder Angle " + swerveModuleConstants.moduleNumber, MathUtil.inputModulus(angleEncoder.getPosition(), 0, 360));
            SmartDashboard.putNumber("Current Reltaive Encoder Angle Non Mod " + swerveModuleConstants.moduleNumber, angleEncoder.getPosition());
            SmartDashboard.putNumber("Current Drive Velocity" + swerveModuleConstants.moduleNumber, driveMotor.getVelocity().getValue());
        }
    }
}