package SushiFrcLib.Swerve.SwerveConstants;

public enum SDSModules {
    MK4i((150.0 / 7.0), 6.12, 5.6, 4, 2 * Math.PI, 4),
    MK4(0, 0, 0, 0,0,0);

    public double angleGearRatio;
    public double driveGearRatio;
    public double maxSpeed;
    public double maxAcceleration;
    public double maxAngularVelocity;
    public double maxAngularAcceleration;

    private SDSModules(double angleGearRatio, double driveGearRatio, double maxSpeed, double maxAcceleration, double maxAngularVelocity, double maxAngularAcceleration) {
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
        this.maxAngularAcceleration = maxAngularAcceleration;
    }
}