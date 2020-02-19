package virtual_robot.controller.robots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.*;
import javafx.fxml.FXML;
import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import odefx.CBits;
import odefx.FxBody;
import odefx.FxBodyHelper;
import odefx.node_with_geom.BoxWithDGeom;
import odefx.node_with_geom.GroupWithDGeoms;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import util3d.Util3D;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

import static org.ode4j.ode.OdeConstants.*;
import static org.ode4j.ode.OdeConstants.dContactBounce;

/**
 * For internal use only. Represents a robot with two standard wheels, color sensor, four distance sensors,
 * a Gyro Sensor, and a Servo-controlled arm on the back.
 *
 * TwoWheelBot is the controller class for the "two_wheel_bot.fxml" markup file.
 */
@BotConfig(name = "Two Wheel Bot")
public class TwoWheelBot extends VirtualBot {

    private final float TOTAL_MASS = 15000;  //mg
    private final float TOTAL_Z_INERTIA = 5000000f; //gm*cm2
    private final float FIELD_FRICTION_COEFF = 1.0f;
    private final float GRAVITY = 980f; // cm/s2
    //Max possible force (in Robot-X direction) at any wheel (each wheel gets 1/4 of robot weight)
    private final float MAX_WHEEL_FORCE = TOTAL_MASS * GRAVITY * FIELD_FRICTION_COEFF / 2.0f;

    public final MotorType motorType = MotorType.Neverest40;
    private DcMotorImpl leftMotor = null;
    private DcMotorImpl rightMotor = null;
    private DcMotorImpl armExtensionMotor = null;
    private DcMotorImpl armRotationMotor = null;
    BNO055IMUImpl imu = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private ServoImpl fingerServo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    private final double armLength = 12 * 2.54;
    private final double armWidth = 2.54;
    private final double handWidth = 8 * 2.54;
    private final double fingerLength = 6 * 2.54;

    private double wheelCircumference;
    private double interWheelDistance;
    Rotate armRotate = new Rotate(0, 0, -armLength/2 + armWidth/2, 0, new Point3D(1, 0, 0));
    Translate midArmTranslate = new Translate(0, 0, 0);
    Translate foreArmTranslate = new Translate(0, 0, 0);
    Translate leftFingerTranslate = new Translate(0, 0, 0);
    Translate rightFingerTranslate = new Translate(0, 0, 0);

    double armRotation = 0;
    double armExtension = 0;
    double armRotationSpeed = 0;
    double armExtensionSpeed = 0;

    GroupWithDGeoms armGroup = null;

    @Override
    public void init(){
        super.init();
        leftMotor = (DcMotorImpl)hardwareMap.dcMotor.get("left_motor");
        rightMotor = (DcMotorImpl)hardwareMap.dcMotor.get("right_motor");
        armRotationMotor = (DcMotorImpl)hardwareMap.dcMotor.get("arm_rotation_motor");
        armExtensionMotor = (DcMotorImpl)hardwareMap.dcMotor.get("arm_extension_motor");
        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        imu = (BNO055IMUImpl)hardwareMap.get(BNO055IMU.class,"imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        fingerServo = (ServoImpl)hardwareMap.servo.get("finger_servo");
        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelDistance = botWidth * 8.0 / 9.0;
    }

    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        hardwareMap.put("left_motor", new DcMotorImpl(motorType));
        hardwareMap.put("right_motor", new DcMotorImpl(motorType));
        hardwareMap.put("arm_extension_motor", new DcMotorImpl(motorType));
        hardwareMap.put("arm_rotation_motor", new DcMotorImpl(motorType));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("finger_servo", new ServoImpl());
    }

    public void updateSensors(){

        //Get current bot position and orientation
        DMatrix3C currentRot = fxBody.getRotation();
        double headingRadians = Math.atan2(currentRot.get10(), currentRot.get00());
        DVector3C currentPos = fxBody.getPosition();
        double x = currentPos.get0();
        double y = currentPos.get1();

        //Based on current bot position and orientation, update sensors
        imu.updateHeadingRadians(headingRadians);

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;

        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }
    }

    public synchronized void updateState(double millis){

        /**
         * Based on kinematic model, predict change in x, y, heading of robot (dX, dY, dHeading) in world coords
         */
        double deltaLeftPos = leftMotor.update(millis);
        double deltaRightPos = rightMotor.update(millis);
        double leftWheelDist = -deltaLeftPos * wheelCircumference / motorType.TICKS_PER_ROTATION;
        double rightWheelDist = deltaRightPos * wheelCircumference / motorType.TICKS_PER_ROTATION;
        double distTraveled = (leftWheelDist + rightWheelDist) / 2.0;
        double dHeading = (rightWheelDist - leftWheelDist) / interWheelDistance;
        DMatrix3C currentRot = fxBody.getRotation();
        double heading = Math.atan2(currentRot.get10(), currentRot.get00());
        double avgHeading = heading + dHeading / 2.0;
        double sinAvg = Math.sin(avgHeading);
        double cosAvg = Math.cos(avgHeading);
        double dX = -distTraveled * Math.sin(heading + dHeading / 2.0);
        double dY = distTraveled * Math.cos(heading + dHeading / 2.0);

        /**
         * Determine the force and torque that would be required to achieve the predicted dX, dY, dHeading
         */

        double t = millis / 1000.0;
        double tSqr = t * t;
        DVector3 deltaPos = new DVector3(dX, dY, 0);
        DVector3C vel = fxBody.getLinearVel().clone();
        ((DVector3)vel).set2(0);
        DVector3 force = deltaPos.reSub(vel.reScale(t)).reScale(2.0 * TOTAL_MASS / tSqr);
        double angVel = fxBody.getAngularVel().get2();
        float torque = (float)(2.0 * TOTAL_Z_INERTIA * (dHeading - angVel*t)/tSqr);

        //Convert the force to the ROBOT COORDINATE system

        double sinHd = Math.sin(heading);
        double cosHd = Math.cos(heading);

        float fXR = (float)(force.get0()*cosHd + force.get1()*sinHd);
        float fYR = (float)(-force.get0()*sinHd + force.get1()*cosHd);

        //Determine forces needed at left and right wheels to achieve the total torque and force

        float fYLeft = 0.5f * (float)(fYR - 2 * torque / interWheelDistance);
        float fYRight = 0.5f * (float)(fYR + 2 * torque / interWheelDistance);
        float fXLeft = fXR / 2.0f;
        float fXRight = fXR / 2.0f;

        /**
         * If expected force on either wheel is greater than max allowed by friction, reduce its magnitude to
         * the max allowed by friction, while keeping direction the same.
         */

        float fLeftMag = (float)Math.hypot(fXLeft, fYLeft);
        if (fLeftMag > MAX_WHEEL_FORCE) {
            fXLeft *= MAX_WHEEL_FORCE / fLeftMag;
            fYLeft *= MAX_WHEEL_FORCE / fLeftMag;
        }

        float fRightMag = (float)Math.hypot(fXRight, fYRight);
        if (fLeftMag > MAX_WHEEL_FORCE) {
            fXRight *= MAX_WHEEL_FORCE / fRightMag;
            fYRight *= MAX_WHEEL_FORCE / fRightMag;
        }

        /**
         * Using the adjusted forces at left and right wheels, recalculate total force and torque on robot
         */

        fXR = fXLeft + fXRight;
        fYR = fYLeft + fYRight;
        torque = (fYRight - fYLeft) * (float)interWheelDistance / 2.0f;

        //Convert these adjusted forces to WORLD COORDINATES and put into the original force DVector3
        force.set0(fXR*cosHd - fYR*sinHd);
        force.set1(fXR*sinHd + fYR*cosHd);
        force.set2(0);

        //Apply the adjusted force and torque to the bot

        fxBody.addForce(force);
        fxBody.addTorque(new DVector3(0, 0, torque));

        /**
         * Update state of robot accessories
         */

        double newArmRotation = armRotation + 0.05 * armRotationMotor.update(millis);
        double oldArmRotation = armRotation;
        armRotation = Math.max(-20, Math.min(90, newArmRotation));
        armRotationSpeed = 1000 * Math.PI * (armRotation - oldArmRotation) / (millis * 180);

        double newArmExtension = armExtension + 0.01 * armExtensionMotor.update(millis);
        double oldArmExtension = armExtension;
        armExtension = Math.max(0, Math.min(36, newArmExtension));
        armExtensionSpeed = 1000 * (armExtension - oldArmExtension) / millis;
    }

    protected void setUpFxBody(){

        //Create new FxBody object to represent the chassis. This will contain the DBody (for physics sim),
        //a Group object for display, and multiple DGeom objects (for collision handling). It will
        //also have children--these will be other FxBody objects that represent robot components other than
        //the chassis.
        DWorld world = controller.getWorld();
        fxBody = FxBody.newInstance(world, botSpace);
        DBody chassisBody = fxBody.getBody();
        DMass chassisMass = OdeHelper.createMass();
        chassisMass.setMass(TOTAL_MASS);
        chassisMass.setI(new DMatrix3(TOTAL_Z_INERTIA, 0, 0, 0, TOTAL_Z_INERTIA, 0, 0, 0, TOTAL_Z_INERTIA));
        chassisBody.setMass(chassisMass);

        final double chassisWidth = 14 * 2.54;
        final double chassisLength = 18 * 2.54;
        final double chassisHeight = 2 * 2.54;
        final double wheelRadius = 2 * 2.54;
        final double wheelWidth = 2 * 2.54;
        final double wheelTranslateX = 8 * 2.54;
        final double armMountHeight = 6 * 2.54;
        final double armMountLength = 2 * 2.54;


        Group botGroup = new Group();

        Box chassis = new Box(chassisWidth, chassisLength, chassisHeight);
        PhongMaterial chassisMaterial = new PhongMaterial(Color.SIENNA);
        chassisMaterial.setSpecularColor(Color.WHITE);
        chassis.setMaterial(chassisMaterial);

        Cylinder[] wheels = new Cylinder[2];
        PhongMaterial wheelMaterial = new PhongMaterial(Color.BLUE);
        wheelMaterial.setSpecularColor(Color.WHITE);
        for (int i=0; i<2; i++){
            wheels[i] = new Cylinder(wheelRadius,wheelWidth);
            wheels[i].setMaterial(wheelMaterial);
            wheels[i].getTransforms().addAll(new Translate(i==0? -wheelTranslateX : wheelTranslateX, 0,0),
                    new Rotate(90, Rotate.Z_AXIS));
        }

        Box leftArmMount = new Box(armWidth, armMountLength, armMountHeight);
        leftArmMount.setMaterial(chassisMaterial);
        leftArmMount.getTransforms().add(new Translate(-armWidth, -chassisLength/2+armMountLength/2, chassisHeight/2+armMountHeight/2));

        Box rightArmMount = new Box(armWidth, armMountLength, armMountHeight);
        rightArmMount.setMaterial(chassisMaterial);
        rightArmMount.getTransforms().add(new Translate(armWidth, -chassisLength/2+armMountLength/2, chassisHeight/2+armMountHeight/2));

        botGroup.getChildren().addAll(chassis, leftArmMount, rightArmMount);
        botGroup.getChildren().addAll(wheels);

        fxBody.setNode(botGroup, true);
        DTriMeshData botBottomData = FxBodyHelper.getParametricTriMeshData(1, -1, -1, 1, 5, 5,
                false, false, new Util3D.Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return s * (float)chassisWidth/2;
                    }

                    @Override
                    public float y(float s, float t) { return t * (float)chassisLength/2; }

                    @Override
                    public float z(float s, float t) {
                        return 0;
                    }
                });

        DTriMesh botBottomMesh = OdeHelper.createTriMesh(botSpace, botBottomData, null, null, null);
        botBottomMesh.setData("Bot Bottom Mesh");
        fxBody.addGeom(botBottomMesh, 0, 0, -wheelRadius);

        armGroup = new GroupWithDGeoms();

        PhongMaterial armMaterial = new PhongMaterial(Color.FUCHSIA);
        armMaterial.setSpecularColor(Color.WHITE);

        BoxWithDGeom arm = new BoxWithDGeom(armWidth, armLength, armWidth, fxBody);
        arm.setMaterial(armMaterial);
        armGroup.getChildren().add(arm);

        GroupWithDGeoms midArmGroup = new GroupWithDGeoms();
        midArmGroup.getTransforms().addAll(midArmTranslate, new Translate(0, 0, armWidth));
        armGroup.getChildren().add(midArmGroup);
        BoxWithDGeom midArm = new BoxWithDGeom(armWidth, armLength, armWidth, fxBody);
        midArm.setMaterial(armMaterial);
        midArmGroup.getChildren().add(midArm);

        GroupWithDGeoms foreArmGroup = new GroupWithDGeoms();
        midArmGroup.getChildren().add(foreArmGroup);
        foreArmGroup.getTransforms().addAll(foreArmTranslate, new Translate(0, 0, armWidth));
        BoxWithDGeom foreArm = new BoxWithDGeom(armWidth, armLength, armWidth, fxBody);
        foreArm.setMaterial(armMaterial);

        BoxWithDGeom hand = new BoxWithDGeom(handWidth, armWidth, armWidth, fxBody);
        hand.getTransforms().add(new Translate(0, armLength/2 + armWidth/2, 0));
        hand.setMaterial(armMaterial);

        BoxWithDGeom  leftFinger = new BoxWithDGeom(armWidth, fingerLength, 5*armWidth, fxBody);
        leftFinger.setMaterial(armMaterial);
        leftFinger.getTransforms().addAll(leftFingerTranslate, new Translate(-handWidth/2+armWidth/2,
                armLength/2 + fingerLength/2 + armWidth, -2*armWidth));

        BoxWithDGeom  rightFinger = new BoxWithDGeom(armWidth, fingerLength, 5*armWidth, fxBody);
        rightFinger.setMaterial(armMaterial);
        rightFinger.getTransforms().addAll(rightFingerTranslate, new Translate(handWidth/2-armWidth/2,
                armLength/2 + fingerLength/2 + armWidth, -2*armWidth));

        foreArmGroup.getChildren().addAll(foreArm, hand, leftFinger, rightFinger);

        armGroup.getTransforms().addAll(new Translate(0, -chassisLength/2+armMountLength/2+armLength/2-armWidth/2,
                        chassisHeight/2+armMountHeight-armWidth/2), armRotate);

        botGroup.getChildren().add(armGroup);

        zBase = wheelRadius;

        fxBody.setCategoryBits(CBits.BOT);
        fxBody.setCollideBits(0xFFF);
        botBottomMesh.setCategoryBits(CBits.BOT_BOTTOM);
        botBottomMesh.setCollideBits(CBits.FLOOR);
        leftFinger.getDGeom().setCategoryBits(CBits.BOT_LEFT_FINGER);
        rightFinger.getDGeom().setCategoryBits(CBits.BOT_RIGHT_FINGER);
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        armRotate.setAngle(armRotation);
        midArmTranslate.setY(armExtension/2.0);
        foreArmTranslate.setY(armExtension/2.0);
        double fingerMovement = 3.2*fingerServo.getInternalPosition();
        leftFingerTranslate.setX(fingerMovement);
        rightFingerTranslate.setX(-fingerMovement);
        armGroup.updateGeomOffsets();
    }

    public void powerDownAndReset(){
        leftMotor.stopAndReset();
        rightMotor.stopAndReset();
        imu.close();
    }

    @Override
    public void handleContacts(int numContacts, DGeom o1, DGeom o2, DContactBuffer contacts, DJointGroup contactGroup) {
        final double baseArmLength = armLength + fingerLength/2 + armWidth/2;

        long o1CBits = o1.getCategoryBits();
        long o2CBits = o2.getCategoryBits();

        boolean o1RF = (o1CBits & CBits.BOT_RIGHT_FINGER) != 0,
                o2RF = (o2CBits & CBits.BOT_RIGHT_FINGER) != 0,
                o1LF = (o1CBits & CBits.BOT_LEFT_FINGER) != 0,
                o2LF = (o2CBits & CBits.BOT_LEFT_FINGER) != 0,
                o1Block = (o1CBits & CBits.STONES) != 0,
                o2Block = (o2CBits & CBits.STONES) != 0;

        DMatrix3 botRot = (DMatrix3)fxBody.getRotation();
        DMatrix3 armRotRelBot = new DMatrix3();
        DMatrix3 armRotRelWorld = new DMatrix3();
        DRotation.dRFromAxisAndAngle(armRotRelBot, 1, 0, 0, armRotation*Math.PI/180);
        DMatrix.dMultiply0(armRotRelWorld, botRot, armRotRelBot);
        DVector3 armDir = new DVector3();
        DMatrix.dMultiply0(armDir, armRotRelWorld, new DVector3(0, 1, 0));



        for (int i=0; i<numContacts; i++)
        {
            DContact contact = contacts.get(i);

            if ( o1RF && o2Block || o2RF && o1Block || o1LF && o2Block || o2LF && o1Block){


                double horizArmLength = baseArmLength + armExtension;


                contact.fdir1.set(botRot.get10()*Math.cos(armDir.get0()), armDir.get1(), armDir.get2());

                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactFDir1 | dContactMotion1 | dContactApprox1_1
                        | dContactApprox1_2 | dContactApprox1_N | dContactRolling | dContactMotion2 | dContactMu2;
                contact.surface.mu = 10;
                contact.surface.mu2 = 3;
                contact.surface.rhoN = dInfinity;
                contact.surface.soft_cfm = 0.0001;
                contact.surface.soft_erp = 0.4;


//                System.out.println("o1: " + (o1RF? "Rt Finger" : o1LF? "Lt Finger" : o1Block? "Block" : "Unknown"));
//                System.out.println("o2: " + (o2RF? "Rt Finger" : o2LF? "Lt Finger" : o2Block? "Block" : "Unknown"));
//                System.out.println("Normal " + contact.geom.normal);
//                System.out.println("Arm Direction " + armDir);
//                System.out.println("horizArmLength = " + horizArmLength);
//                System.out.println("armRotationSpeed = " + armRotationSpeed);

                if (o1Block) {
                    contact.surface.motion1 = armExtensionSpeed;
                    contact.surface.motion2 = o2LF? horizArmLength * armRotationSpeed : -horizArmLength * armRotationSpeed;
                } else {
                    contact.surface.motion1 = -armExtensionSpeed;
                    contact.surface.motion2 = o1LF? -horizArmLength * armRotationSpeed : horizArmLength * armRotationSpeed;
                }
            }  else {
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
                contact.surface.mu = 0;
                contact.surface.soft_cfm = 0.00000001;
                contact.surface.soft_erp = 0.8;
                contact.surface.bounce = 0.3;
                contact.surface.bounce_vel = 10;
            }
            DJoint c = OdeHelper.createContactJoint (controller.getWorld(),contactGroup,contact);
            c.attach (contact.geom.g1.getBody(), contact.geom.g2.getBody());
        }
    }
}
