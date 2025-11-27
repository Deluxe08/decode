package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
@Configurable

public class Shooter extends SubsystemBase {
   // private Servo f;
    private DcMotorEx shooter;//, r;
    private PIDFController b, s;

    private double t = 0;

    private boolean activated = true;

    public static double bp = 0.00090;
    public static double bd = 0.00003;
    public static double bf = 0.0;

    public static double sp = 0.00050;
    public static double sd = 0.000015;
    public static double sf = 0.0;

    public static double pSwitch = 175;

    public static double close = 1900;
    public static double far = 2600;
    public static double flipUp = 0.3;
    public static double flipDown = 0.71;

    public Shooter(HardwareMap hardwareMap) {
        b = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        //r = hardwareMap.get(DcMotorEx.class, "sr");
        //f = hardwareMap.get(Servo.class, "f");
        //r.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }

    public void setPower(double p) {
        shooter.setPower(p);
        //r.setPower(p);
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public boolean isActivated() {
        return activated;
    }

    public void far() {
        setTarget(far);
        on();
    }

    public void close() {
        setTarget(close);
        on();
    }

    public void setTarget(double velocity) {
        t = velocity;
    }

    @Override
    public void periodic() {
        b.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
        s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

        if (activated) {
            if (Math.abs(getTarget() - getVelocity()) < pSwitch) {
                s.updateError(getTarget() - getVelocity());
                setPower(s.run());
            } else {
                b.updateError(getTarget() - getVelocity());
                setPower(b.run());
            }
        }
    }

    public void up() {
        //f.setPosition(flipUp);
    }

    public void down() {
        //f.setPosition(flipDown);
    }

//    public void flip() {
//        if (f.getPosition() == flipDown)
//            up();
//        else
//            down();
//    }

    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < 50;
    }

    public void forDistance(double distance) {
        //setTarget((6.13992 * distance) + 858.51272);
        setTarget((0.00180088*Math.pow(distance, 2))+(4.14265*distance)+948.97358);
    }

//    public boolean atUp() {
//        //return f.getPosition() == flipUp;
//    }

}