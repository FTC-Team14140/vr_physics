package odefx;

import org.ode4j.ode.DGeom;

public class CBits {

    /**
     * Category Bits
     */

    //Field Parts
    public static final long FLOOR = 0x1;
    public static final long WALLS = 0x2;
    public static final long BRIDGE = 0x4;

    //Game Elements
    public static final long STONES = 0x10;
    public static final long FOUNDATIONS = 0x20;

    //General Bot
    public static final long BOT = 0x100;
    public static final long BOT_BOTTOM = 0x200;

    //BetaBot
    public static final long BOT_HANDS = 0x400;
    public static final long BOT_LEFT_HAND = 0x800;
    public static final long BOT_RIGHT_HAND = 0x1000;
    public static final long BOT_INTAKE = 0x2000;
    public static final long BOT_LEFT_INTAKE = 0x4000;
    public static final long BOT_RIGHT_INTAKE = 0x8000;
    public static final long BOT_INTAKE_ROOF = 0x10000;

    //Two Wheel Bot
    public static final long BOT_FINGERS = 0x20000;
    public static final long BOT_LEFT_FINGER = 0x40000;
    public static final long BOT_RIGHT_FINGER = 0x80000;


    /**
     * Collide Bits
     */


}
