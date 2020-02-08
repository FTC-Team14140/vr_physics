package com.qualcomm.robotcore.hardware;


/**
 * Implementation of the DcMotor interface.
 *
 * A DcMotorPassiveImpl object exists to:
 *   1) Keep track of what power has been requested.
 *   2) Update ticks using a linear equation, based on state info passed in from the bot.
 *   3) Allow encoder to be reset.
 */

public class DcMotorPassiveImpl implements DcMotor {

    public final MotorType motorType;

    private double power = 0;
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private double actualPosition = 0.0;
    private int targetPosition = 0;
    private Direction direction = Direction.FORWARD;

    double basePosition = 0.0;

    double tickIntercept;
    double ticksPerUnit;

    public DcMotorPassiveImpl(double initialUnits, double ticksPerUnit, MotorType mType){
        motorType = mType;
        this.ticksPerUnit = ticksPerUnit;
        this.tickIntercept = -ticksPerUnit * initialUnits;
    }

    public void update(double currentUnits){
        actualPosition = tickIntercept + ticksPerUnit * currentUnits;
    }

    public double getUnitsPerSec(){
        double p;
        if (mode == RunMode.STOP_AND_RESET_ENCODER) p = 0;
        else if (mode == RunMode.RUN_WITHOUT_ENCODER || mode == RunMode.RUN_USING_ENCODER) p = power;
        else{
            double absPower = Math.abs(power);
            p = absPower * (getCurrentPosition() - targetPosition) / (2.0 * motorType.TICKS_PER_ROTATION);
            if (Math.abs(p) > absPower) p = absPower * Math.signum(p);
        }
        double res = p * motorType.MAX_TICKS_PER_SECOND  / ticksPerUnit;
        if (direction == Direction.REVERSE) res *= -1;
        return res;
    }

    @Override
    public void setMode(RunMode mode) {
        this.mode = mode;
    }

    @Override
    public RunMode getMode() {
        return this.mode;
    }

    @Override
    public int getCurrentPosition() {
        int res = (int)(actualPosition - basePosition);
        if (direction == Direction.REVERSE) res *= -1;
        return res;
    }

    @Override
    public void setTargetPosition(int pos) {

    }

    @Override
    public int getTargetPosition() {
        return this.targetPosition;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return this.direction;
    }

    @Override
    public void setPower(double power) {
        this.power = power;
    }

    @Override
    public double getPower() {
        return this.power;
    }
}
