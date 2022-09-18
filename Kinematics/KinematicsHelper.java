package SushiFrcLib.Kinematics;

import edu.wpi.first.math.geometry.Translation2d;

public class KinematicsHelper {
    // Args in meters
    public static edu.wpi.first.math.kinematics.SwerveDriveKinematics getKinematics(double trackWidth, double wheelBase) {
        return new edu.wpi.first.math.kinematics.SwerveDriveKinematics(
            // Front left
            new Translation2d(trackWidth / 2.0,
                wheelBase / 2.0),
            // Front right
            new Translation2d(trackWidth/ 2.0,
                -wheelBase / 2.0),
            // Back left
            new Translation2d(-trackWidth / 2.0,
                wheelBase / 2.0),
            // Back right
            new Translation2d(-trackWidth / 2.0,
                -wheelBase / 2.0)
        );
    }
}
