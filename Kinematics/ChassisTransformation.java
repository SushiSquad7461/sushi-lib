package libraries.SushiFrcLib.Kinematics;

import libraries.SushiFrcLib.Math.Vector2;

public class ChassisTransformation {
    Vector2 translation;
    double angularVelocity;

    public ChassisTransformation(double x, double y, double ω) {
        translation = new Vector2(x, y);
        angularVelocity = ω;
    }
}
