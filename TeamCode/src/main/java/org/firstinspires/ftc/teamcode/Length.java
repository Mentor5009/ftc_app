package org.firstinspires.ftc.teamcode;

public class Length {
    public enum Unit {
        CM(1),
        INCH(2.54);
        public double val;
        Unit(double conv) {
            this.val = conv;
        }
    }

    private double amount;
    private Unit unit;

    public Length(double am, Unit un) {
        amount = am;
        unit = un;
    }

    public Length to(Unit newUnit) {
        return new Length(in(newUnit),newUnit);
    }

    public double in(Unit newUnit) {
        return amount * newUnit.val/unit.val;
    }
}
