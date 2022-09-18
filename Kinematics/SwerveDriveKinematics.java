package SushiFrcLib.Kinematics;

import java.util.ArrayList;
import java.util.Arrays;

import SushiFrcLib.Math.Rotation2;
import SushiFrcLib.Math.Vector2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDriveKinematics {
    private final ArrayList<Vector2> kModulePositions;
    private final ArrayList<Vector2> kModuleTurnVector;
    private double maxSpeed;

    public SwerveDriveKinematics(double wheelBase, double maxSpeed) { //front right, front left, back right, back left
        this(new ArrayList<Vector2>(Arrays.asList(new Vector2(wheelBase, wheelBase), new Vector2(-wheelBase, wheelBase), new Vector2(wheelBase, -wheelBase), new Vector2(-wheelBase, -wheelBase))), maxSpeed);
    }

    public SwerveDriveKinematics(ArrayList<Vector2> modulePositions, double maxSpeed) {
        if( modulePositions.size() != 4) {
            throw new java.lang.Error("Must have exactly 4 swerve modules");
        }

        kModulePositions = modulePositions;
        kModuleTurnVector = new ArrayList<Vector2>();
        this.maxSpeed = maxSpeed;

        for( var modulePosition : kModulePositions) {
            kModuleTurnVector.add(modulePosition.rotateBy(Rotation2.fromDegrees(90)).normal());
        }
    }

    public ArrayList<SwerveModuleState> calculate(Rotation2 robotOrientation, double x, double y, double angularVelocity) {
        ArrayList<Vector2> res = new ArrayList<Vector2>(4);

        Vector2 desiredTranslation = new Vector2(x, y);

        double maxLength = 0;

        for(int i = 0; i<res.size(); i++) {
            res.set(i, desiredTranslation.add(kModuleTurnVector.get(i).scale(angularVelocity)).rotateBy(robotOrientation.negate()));
            maxLength = Math.max(res.get(i).length, maxLength);
        }

        if (maxLength > maxSpeed) {
            double scaleFactor = maxSpeed / maxLength;
            for ( int i = 0; i<res.size(); i++) {
                res.set(i, res.get(i).scale(scaleFactor));
            }
        }
        return toModuleStates(res);
    }

    public ArrayList<SwerveModuleState> toModuleStates(ArrayList<Vector2> vectors) {
        ArrayList<SwerveModuleState> res = new ArrayList<SwerveModuleState>();
        for (var vector : vectors) {
            res.add(new SwerveModuleState(vector.length, new Rotation2d(vector.getAngle().toRadians())));
        }
        return res;
    }
}
