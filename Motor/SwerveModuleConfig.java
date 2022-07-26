// package SushiFrcLib.Motor;

// import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
// import com.swervedrivespecialties.swervelib.SwerveModule;

// public class SwerveModuleConfig {
//     public final Mk4SwerveModuleHelper.GearRatio GEAR_RATIO;
//     public final int POWER_ID;
//     public final int STEER_ID;
//     public final int ENCODER_ID;
//     public final double STEER_OFFSET;

//     public SwerveModuleConfig(int POWER_ID, int STEER_ID, int ENCODER_ID, double STEER_OFFSET, Mk4SwerveModuleHelper.GearRatio GEAR_RATIO) {
//         this.POWER_ID = POWER_ID;
//         this.STEER_ID = STEER_ID;
//         this.ENCODER_ID = ENCODER_ID;
//         this.STEER_OFFSET = STEER_OFFSET;
//         this.GEAR_RATIO = GEAR_RATIO;
//     }

//     // Create a swerve neo module
//     public SwerveModule createNeoModule() {
//         return Mk4SwerveModuleHelper.createNeo(
//             GEAR_RATIO,
//             POWER_ID,
//             STEER_ID,
//             ENCODER_ID,
//             STEER_OFFSET
//         );
//     }

//     // Ceate a swerve falcon module
//     public SwerveModule createFalconModule() {
//         return Mk4SwerveModuleHelper.createFalcon500(
//             GEAR_RATIO,
//             POWER_ID,
//             STEER_ID,
//             ENCODER_ID,
//             STEER_OFFSET
//         );
//     }
// }