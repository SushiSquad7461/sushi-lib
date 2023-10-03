package SushiFrcLib.Sensors.absoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteEncoder {
    private DutyCycleEncoder encoder;
    private double offset;

    public AbsoluteEncoder(int id, double offset) {
        encoder = new DutyCycleEncoder(id);
        this.offset = offset;
    }

    public double getPosition() {
        return encoder.get() * 360 - offset;
    }

    public double getNormalizedPosition() {
        return MathUtil.inputModulus(getPosition(), -180, 180);
    }
}
