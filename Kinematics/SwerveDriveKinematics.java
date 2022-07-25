package libraries.SushiFrcLib.Kinematics;

import java.util.ArrayList;

import libraries.SushiFrcLib.Math.Rotation2;
import libraries.SushiFrcLib.Math.Vector2;

public class SwerveDriveKinematics {
    private final ArrayList<Vector2> kModulePositions;
    private final ArrayList<Vector2> kModuleTurnVector;
    private double maxSpeed;

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

    public ArrayList<Vector2> calculate(Rotation2 robotOrientation, ChassisTransformation chassisSpeed) {
        ArrayList<Vector2> res = new ArrayList<Vector2>(4);

        Vector2 desiredTranslation = chassisSpeed.translation;

        double maxLength = 0;

        for(int i = 0; i<res.size(); i++) {
            res.set(i, desiredTranslation.add(kModuleTurnVector.get(i).scale(chassisSpeed.angularVelocity)).rotateBy(robotOrientation.negate()));
            maxLength = Math.max(res.get(i).length, maxLength);
        }

        if (maxLength > maxSpeed) {
            double scaleFactor = maxSpeed / maxLength;
            for ( int i = 0; i<res.size(); i++) {
                res.set(i, res.get(i).scale(scaleFactor));
            }
        }
        return res;
    }
}
