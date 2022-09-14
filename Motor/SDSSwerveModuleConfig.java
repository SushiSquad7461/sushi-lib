package SushiFrcLib.Motor;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import SushiFrcLib.Constants.SushiConstants;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;


public class SDSSwerveModuleConfig {
    public final Mk4SwerveModuleHelper.GearRatio FOUR_GEAR_RATIO;
    public final Mk4iSwerveModuleHelper.GearRatio FOUR_I_GEAR_RATION;
    public final int POWER_ID;
    public final int STEER_ID;
    public final int ENCODER_ID;
    public final double STEER_OFFSET;
    public final String NAME;
    public final int SHIFT;

    public SDSSwerveModuleConfig(int POWER_ID, int STEER_ID, int ENCODER_ID, double STEER_OFFSET, String name, int shift) {
        this.POWER_ID = POWER_ID;
        this.STEER_ID = STEER_ID;
        this.ENCODER_ID = ENCODER_ID;
        this.STEER_OFFSET = STEER_OFFSET;
        this.FOUR_GEAR_RATIO = SushiConstants.SDS_SWERVE.GEAR_RATIO;
        this.FOUR_I_GEAR_RATION = SushiConstants.SDS_SWERVE.FOUR_I_GEAR_RATIO;
        this.NAME = name;
        this.SHIFT = shift;
    }

    public SDSSwerveModuleConfig(int POWER_ID, int STEER_ID, int ENCODER_ID, double STEER_OFFSET, String name, int shift, Mk4SwerveModuleHelper.GearRatio ratio) {
        this.POWER_ID = POWER_ID;
        this.STEER_ID = STEER_ID;
        this.ENCODER_ID = ENCODER_ID;
        this.STEER_OFFSET = STEER_OFFSET;
        this.FOUR_GEAR_RATIO = ratio;
        this.FOUR_I_GEAR_RATION = SushiConstants.SDS_SWERVE.FOUR_I_GEAR_RATIO;
        this.NAME = name;
        this.SHIFT = shift;
    }

    public SDSSwerveModuleConfig(int POWER_ID, int STEER_ID, int ENCODER_ID, double STEER_OFFSET, String name, int shift, Mk4iSwerveModuleHelper.GearRatio ratio) {
        this.POWER_ID = POWER_ID;
        this.STEER_ID = STEER_ID;
        this.ENCODER_ID = ENCODER_ID;
        this.STEER_OFFSET = STEER_OFFSET;
        this.FOUR_GEAR_RATIO = SushiConstants.SDS_SWERVE.GEAR_RATIO;
        this.FOUR_I_GEAR_RATION = ratio;
        this.NAME = name;
        this.SHIFT = shift;
    }

    public SwerveModule createNeoModule() {
        return Mk4SwerveModuleHelper.createNeo(
            FOUR_GEAR_RATIO,
            POWER_ID,
            STEER_ID,
            ENCODER_ID,
            STEER_OFFSET
        );
    }

    public SwerveModule createFourIFalconModule() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        
        return Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout(NAME, BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(SHIFT, 0),
            FOUR_I_GEAR_RATION, 
            POWER_ID, 
            STEER_ID, 
            ENCODER_ID, 
            STEER_OFFSET);
    }
}
