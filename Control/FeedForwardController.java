package SushiFrcLib.Control;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FeedForwardController {
    private double kF;
    private SimpleMotorFeedforward feedForwardController;
    private final boolean isSimple;

    public FeedForwardController(double f) {
        this.kF = f;
        isSimple = true;
    }

    public FeedForwardController(double A, double S, double V) {
        isSimple = false;
        this.feedForwardController = new SimpleMotorFeedforward(S, V, A);
    }

    public double getF() {
        if (isSimple) {
            return this.kF;
        } else {
            return Double.NaN;
        }
    }

    public double calculate(double velocity) {
        if (isSimple) {
            return velocity * kF;
        } else {
            return feedForwardController.calculate(velocity);
        }
    }
}
