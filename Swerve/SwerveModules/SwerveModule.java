package SushiFrcLib.Swerve.SwerveModules;

import com.ctre.phoenix6.hardware.CANcoder;

import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.Swerve.CTREModuleState;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
    public SwerveModuleConstants swerveModuleConstants;
    protected CANcoder angleEncoder;
    protected Rotation2d lastAngle;

    protected PIDTuning angleTuning;
    protected PIDTuning driveTuning;

    public SwerveModule(SwerveModuleConstants moduleConstants) {
        angleEncoder = moduleConstants.getCanCoder();
        this.swerveModuleConstants = moduleConstants;
        lastAngle = Rotation2d.fromDegrees(0);

        angleTuning = new PIDTuning("Angle Motor" + moduleConstants.moduleNumber, moduleConstants.angleConfig.pid, moduleConstants.swerveTuningMode);
        driveTuning = new PIDTuning("Drive Motor" + moduleConstants.moduleNumber, moduleConstants.driveConfig.pid, moduleConstants.swerveTuningMode);
     }

    abstract public void resetToAbsolute();

    public void log() {}

    abstract protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians);
    abstract protected Rotation2d getEncoderAngle();
    abstract protected double getPositionMeter();
    abstract protected double getVelocityMeter();

    abstract public void updatePID();

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public double getAbsoluteAngleDegrees() {
        return MathUtil.inputModulus(getCanCoder().getDegrees(), 0, 360);
    }

    public SwerveModuleState getState() {
         return new SwerveModuleState(getVelocityMeter(), getEncoderAngle());
    }

    public SwerveModulePosition getPose() {
        return new SwerveModulePosition(getPositionMeter(), getEncoderAngle());
    }

    public void setDesiredState(SwerveModuleState state) {
        // Prevents angle motor from turning further than it needs to. 
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        state = CTREModuleState.optimize(state, lastAngle);

        Rotation2d angle = Math.abs(state.speedMetersPerSecond) <= swerveModuleConstants.moduleInfo.driveGearRatio * 0.01
            ? lastAngle
            :  state.angle;

        lastAngle = angle;

        applySwerveModuleState(state.speedMetersPerSecond, angle);
    }
}
