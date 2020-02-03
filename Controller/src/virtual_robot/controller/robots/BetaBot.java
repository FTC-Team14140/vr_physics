package virtual_robot.controller.robots;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.*;
import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import odefx.FxBody;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.ode4j.math.*;
import org.ode4j.ode.*;
import util3d.Parts;
import util3d.Util3D;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

/**
 *  Team Beta 8397 specific robot configuration
 *
 */
@BotConfig(name = "Beta Bot")
public class BetaBot extends VirtualBot {

    private final float TOTAL_MASS = 15;  //kg
    private final float TOTAL_Z_INERTIA = 0.5f; //kg*m2
    private final float FIELD_FRICTION_COEFF = 1.0f;
    private final float GRAVITY = 9.8f; // m/s2
    //Max possible force (in Robot-X direction) at any wheel (each wheel gets 1/4 of robot weight)
    private final float MAX_WHEEL_X_FORCE = TOTAL_MASS * GRAVITY * FIELD_FRICTION_COEFF / (4.0f * (float)Math.sqrt(2));

    public final MotorType motorType = MotorType.Neverest40;
    private DcMotorImpl[] motors = null;
    private DcMotorImpl liftMotor = null;
    private BNO055IMUImpl imu = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private ServoImpl handServo = null;
    private CRServoImpl sliderCRServo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    private double wheelCircumference;
    private double interWheelWidth;
    private double interWheelLength;
    private double wlAverage;

    private double[][] tWR; //Transform from wheel motion to robot motion

    GeneralMatrixF  M_ForceWheelToRobot;
    MatrixF M_ForceRobotToWheel;

    Rotate handRotate = new Rotate(0, 0, 0, 2.5, new Point3D(0,1,0));
    Translate sliderTranslate = new Translate(0, 0, 0);
    Translate[] liftTranslates = new Translate[]{new Translate(0, 0, 0), new Translate(0, 0, 0), new Translate(0, 0, 0)};
    Rotate[] wheelRotates = new Rotate[] {new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS)};

    double handRotation = 0;
    double sliderTranslation = 0;
    double liftElevation = 0;
    double[] wheelRotations = new double[]{0,0,0,0};

    private DGeom.DNearCallback dNearCallback = new DGeom.DNearCallback() {
        @Override
        public void call(Object data, DGeom o1, DGeom o2) {
            nearCallback(data, o1, o2);
        }
    };

    public DGeom.DNearCallback getNearCallback() { return dNearCallback; }

    @Override
    public void init(){
        super.init();
        motors = new DcMotorImpl[]{
                (DcMotorImpl)hardwareMap.dcMotor.get("back_left_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("front_left_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("front_right_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("back_right_motor")
        };

        liftMotor = (DcMotorImpl)hardwareMap.dcMotor.get("lift_motor");

        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        handServo = (ServoImpl)hardwareMap.servo.get("hand_servo");
        sliderCRServo = (CRServoImpl)hardwareMap.crservo.get("slider_crservo");
        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelWidth = botWidth * 8.0 / 9.0;
        interWheelLength = botWidth * 7.0 / 9.0;
        wlAverage = (interWheelLength + interWheelWidth) / 2.0;

        tWR = new double[][] {
                {-0.25, 0.25, -0.25, 0.25},
                {0.25, 0.25, 0.25, 0.25},
                {-0.25/ wlAverage, -0.25/ wlAverage, 0.25/ wlAverage, 0.25/ wlAverage},
                {-0.25, 0.25, 0.25, -0.25}
        };

        float RRt2 = 0.5f * (float)Math.sqrt(interWheelLength*interWheelLength + interWheelWidth*interWheelWidth) * (float)Math.sqrt(2.0);
        M_ForceWheelToRobot = new GeneralMatrixF(4, 4, new float[]{
                1, 1, 1, 1,
                -1, 1, -1, 1,
                RRt2, -RRt2, -RRt2, RRt2,
                1, 1, -1, -1});

        M_ForceRobotToWheel = M_ForceWheelToRobot.inverted();

    }

    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorImpl(motorType));
        hardwareMap.put("lift_motor", new DcMotorImpl(motorType, false, false));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("hand_servo", new ServoImpl());
        hardwareMap.put("slider_crservo", new CRServoImpl(720));
    }

    public synchronized void updateSensors(){

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

        //Based on KINEMATIC model, determine expected changes in X,Y,THETA during next interval

        double[] deltaTicks = new double[4];
        double[] w = new double[4];

        for (int i = 0; i < 4; i++) {
            deltaTicks[i] = motors[i].update(millis);
            w[i] = deltaTicks[i] * wheelCircumference / motorType.TICKS_PER_ROTATION;
            double wheelRotationDegrees = 360.0 * deltaTicks[i] / motorType.TICKS_PER_ROTATION;
            if (i < 2) {
                w[i] = -w[i];
                wheelRotationDegrees = -wheelRotationDegrees;
            }
            wheelRotations[i] += Math.min(17, Math.max(-17, wheelRotationDegrees));
        }

        double[] robotDeltaPos = new double[] {0,0,0,0};
        for (int i=0; i<4; i++){
            for (int j = 0; j<4; j++){
                robotDeltaPos[i] += tWR[i][j] * w[j];
            }
        }

        double dxR = robotDeltaPos[0];
        double dyR = robotDeltaPos[1];
        double dHeading = robotDeltaPos[2];
        DMatrix3C currentRot = fxBody.getRotation();
        double heading = Math.atan2(currentRot.get10(), currentRot.get00());
        double avgHeading = heading + dHeading / 2.0;
        double sinAvg = Math.sin(avgHeading);
        double cosAvg = Math.cos(avgHeading);
        double dX = dxR * cosAvg - dyR * sinAvg;
        double dY = dxR * sinAvg + dyR * cosAvg;

        //Determine the force and torque (WORLD COORDS) that would be required to achieve the changes predicted by
        //the kinematic model.

        double t = millis / 1000.0;
        double tSqr = t * t;
        DVector3 deltaPos = new DVector3(dX, dY, 0);
        DVector3C vel = fxBody.getLinearVel();
        ((DVector3)vel).set2(0);
        DVector3 force = deltaPos.reSub(vel.reScale(t)).reScale(2.0 * TOTAL_MASS / tSqr);
        double angVel = fxBody.getAngularVel().get2();
        float torque = (float)(2.0 * TOTAL_Z_INERTIA * (dHeading - angVel*t)/tSqr);

        //Convert the force to the ROBOT COORDINATE system

        double sinHd = Math.sin(heading);
        double cosHd = Math.cos(heading);

        float fXR = (float)(force.get0()*cosHd + force.get1()*sinHd);
        float fYR = (float)(-force.get0()*sinHd + force.get1()*cosHd);

        //Determine the forces that would be required on each of the bot's four wheels to achieve
        //the total force and torque predicted by the kinematic model

        VectorF botForces = new VectorF(fXR, fYR, torque, 0);
        VectorF wheel_X_Forces = M_ForceRobotToWheel.multiplied(botForces);

        //If any of the wheel forces exceeds the product of muStatic*mass*gravity, reduce the magnitude
        //of that force to muKinetic*mass*gravity, keeping the direction the same

        for (int i=0; i<4; i++){
            float f = wheel_X_Forces.get(i);
            if (Math.abs(f) > MAX_WHEEL_X_FORCE) wheel_X_Forces.put(i, MAX_WHEEL_X_FORCE * Math.signum(f));
        }

        //Based on the adjusted forces at each wheel, determine net force and net torque on the bot,
        //Force is in ROBOT COORDINATE system

        botForces = M_ForceWheelToRobot.multiplied(wheel_X_Forces);

        //Convert these adjusted forces to WORLD COORDINATES and put into the original force DVector3
        force.set0(botForces.get(0)*cosHd - botForces.get(1)*sinHd);
        force.set1(botForces.get(0)*sinHd + botForces.get(1)*cosHd);
        force.set2(0);

        //Apply the adjusted force and torque to the bot

        fxBody.addForce(force);
        fxBody.addTorque(new DVector3(0, 0, botForces.get(2)));



        double newHandRotation = handServo.getInternalPosition() * 180.0 - 90.0;
        handRotation = Math.min(30, Math.max(-5, newHandRotation));

        double newSliderTranslation = sliderTranslation + sliderCRServo.updatePositionDegrees(millis) * 4.0 / 360.0;
        double deltaLiftMotor = liftMotor.update(millis);
        double newLiftElevation = liftElevation + deltaLiftMotor * 8.0 / 1120.0;
        double minElev = 0;

        if (newLiftElevation >= 3.95 || newSliderTranslation <=0.95 || newSliderTranslation >= 7.2){
            sliderTranslation = Math.min(11, Math.max(-2, newSliderTranslation));
        } else if (sliderTranslation <=0.95){
            sliderTranslation = Math.min(0.95, Math.max(-2, newSliderTranslation));
        } else if (sliderTranslation >= 7.2) {
            sliderTranslation = Math.min(11, Math.max(7.2, newSliderTranslation));
        } else {
            sliderTranslation = Math.min(11, Math.max(-2, newSliderTranslation));
            minElev = 3.95;
        }

        if (newLiftElevation > 36){
            if (Math.abs(newLiftElevation - liftElevation) < 0.0001) liftMotor.adjustActualPosition(-deltaLiftMotor);
            else {
                liftMotor.adjustActualPosition(-deltaLiftMotor *(newLiftElevation - 36)/(newLiftElevation - liftElevation));
                liftElevation = 36;
            }
        } else if (newLiftElevation < minElev) {
            if (Math.abs(newLiftElevation - liftElevation) < 0.0001) liftMotor.adjustActualPosition(-deltaLiftMotor);
            else {
                liftMotor.adjustActualPosition(-deltaLiftMotor * (newLiftElevation - minElev) / (newLiftElevation - liftElevation));
                liftElevation = minElev;
            }
        } else {
            liftElevation = newLiftElevation;
        }
    }

    protected void setUpFxBody(){

        //Create new FxBody object to represent the chassis. This will contain the DBody (for physics sim),
        //a Group object for display, and multiple DGeom objects (for collision handling). It will
        //also have children--these will be other FxBody objects that represent robot components other than
        //the chassis.

        fxBody = FxBody.newInstance(controller.getWorld(), space);
        DBody chassisBody = fxBody.getBody();
        DMass chassisMass = OdeHelper.createMass();
        chassisMass.setMass(TOTAL_MASS);
        chassisMass.setI(new DMatrix3(TOTAL_Z_INERTIA, 0, 0, 0, TOTAL_Z_INERTIA, 0, 0, 0, TOTAL_Z_INERTIA));
        chassisBody.setMass(chassisMass);



        float tetrixWidth = 1.25f * 0.0254f;
        float sliderLength = 16f * 0.0254f;

        float pltThk = 0.125f * 0.0254f;
        float halfPltHt = 2 * 0.0254f;
        float halfPltLen1 = 8.5f * 0.0254f;
        float halfPltLen2 = 7.5f * 0.0254f;
        float pltXOffset1 = 5.5f * 0.0254f;
        float pltXOffset2 = 8.5f * 0.0254f;
        float pltZOffset = 0.25f * 0.0254f;
        double sideBoxLength = 2.0 * halfPltLen1;
        double sideBoxWidth = pltXOffset2 - pltXOffset1 + pltThk;
        double sideBoxHeight = 2.0 * halfPltHt;
        double sideboxXOffset = 0.5 * (pltXOffset1 + pltXOffset2);

        float shortRailLength = 3.125f * 0.0254f;
        float longRailLength = 17.125f * 0.0254f;
        float shortRailXOffset = 7 * 0.0254f;
        float railYOffset = 7.875f * 0.0254f;
        float wheelDiam = 4 * 0.0254f;
        float wheelWidth = 2 * 0.0254f;
        float wheelXOffset = 7 * 0.0254f;
        float wheelYOffset = 6.5f * 0.0254f;

        float liftThk = 0.6f*0.0254f;
        float liftHt = 13f * 0.0254f;
        float liftXOffset = 5.8625f * 0.0254f;
        float liftBaseYOffset = 2.0f * 0.0254f;
        float liftZOffset = 4.75f * 0.0254f;

        float crossBarZOffset = liftHt/2 - liftThk/2;
        float crossBarLength = 2 * liftXOffset - liftThk;

        //Create Group for display of chassis

        Group chassis = new Group();

        Group[] plates = new Group[4];
        PhongMaterial plateMaterial = new PhongMaterial(Color.color(0.6, 0.3, 0));
        for (int i=0; i<4; i++){
            plates[i] = Util3D.polygonBox(pltThk, new float[]{halfPltHt ,-halfPltLen1, halfPltHt, halfPltLen1, halfPltHt/2,
                            halfPltLen1, -halfPltHt, halfPltLen2, -halfPltHt, -halfPltLen2, halfPltHt/2, -halfPltLen1},
                    plateMaterial);
            float x = (i<2? -1.0f : 1.0f) * (i%2 == 0? pltXOffset2:pltXOffset1);
            plates[i].getTransforms().addAll(new Translate(x, 0, pltZOffset), new Rotate(-90, Rotate.Y_AXIS));
        }

        Group frontLeftRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        frontLeftRail.getTransforms().addAll(new Translate(-shortRailXOffset, railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group frontRightRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        frontRightRail.getTransforms().addAll(new Translate(shortRailXOffset, railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group frontRail = Parts.tetrixBox(longRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        frontRail.getTransforms().addAll(new Translate(0, railYOffset, halfPltHt+pltZOffset+1.5*tetrixWidth));
        Group backLeftRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        backLeftRail.getTransforms().addAll(new Translate(-shortRailXOffset, -railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group backRightRail = Parts.tetrixBox(shortRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        backRightRail.getTransforms().addAll(new Translate(shortRailXOffset, -railYOffset, halfPltHt+pltZOffset+0.5*tetrixWidth));
        Group backRail = Parts.tetrixBox(longRailLength, tetrixWidth, tetrixWidth, tetrixWidth);
        backRail.getTransforms().addAll(new Translate(0, -railYOffset, halfPltHt+pltZOffset+1.5*tetrixWidth));

        chassis.getChildren().addAll(plates);
        chassis.getChildren().addAll(frontLeftRail, frontRightRail, frontRail, backLeftRail, backRightRail, backRail);

        Group[] wheels = new Group[4];
        for (int i=0; i<4; i++){
            wheels[i] = Parts.mecanumWheel(wheelDiam, wheelWidth, i);
            wheels[i].setRotationAxis(new Point3D(0, 0, 1));
            wheels[i].setRotate(90);
            wheels[i].setTranslateX(i<2? -wheelXOffset : wheelXOffset);
            wheels[i].setTranslateY(i==0 || i==3? -wheelYOffset : wheelYOffset);
            wheels[i].getTransforms().add(wheelRotates[i]);
        }


        chassis.getChildren().addAll(wheels);

        PhongMaterial slideMaterial = Util3D.imageMaterial("/virtual_robot/assets/rev_slide.jpg");

        Group leftLiftBase = Util3D.patternBox(liftThk, liftThk, liftHt, liftThk, liftThk, liftThk, slideMaterial);
        leftLiftBase.getTransforms().addAll(new Translate(-liftXOffset, liftBaseYOffset, liftZOffset));
        Group rightLiftBase = Util3D.patternBox(liftThk, liftThk, liftHt, liftThk, liftThk, liftThk, slideMaterial);
        leftLiftBase.getTransforms().addAll(new Translate(liftXOffset, liftBaseYOffset, liftZOffset));
        chassis.getChildren().addAll(leftLiftBase, rightLiftBase);

        //For display purposes, set the Node object of fxBody to the display group

        fxBody.setNode(chassis, false);

        //Generate DGeom objects (they happen to all be boxes) for chassis collision handling

        DBox rightSideBox = OdeHelper.createBox(sideBoxWidth, sideBoxLength, sideBoxHeight);
        DBox leftSideBox = OdeHelper.createBox(sideBoxWidth, sideBoxLength, sideBoxHeight);
        DBox leftFrontRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox rightFrontRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox frontRailBox = OdeHelper.createBox(0.9*longRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox leftBackRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox rightBackRailBox = OdeHelper.createBox(0.9*shortRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox backRailBox = OdeHelper.createBox(0.9*longRailLength, tetrixWidth, 0.9*tetrixWidth);
        DBox leftLiftBaseBox = OdeHelper.createBox(liftThk, liftThk, liftHt);
        DBox rightLiftBaseBox = OdeHelper.createBox(liftThk, liftThk, liftHt);

        //Add the chassis DGeom objects to fxBody, with appropriate offsets

        fxBody.addGeom(rightSideBox, sideboxXOffset, 0, pltZOffset);
        fxBody.addGeom(leftSideBox, -sideboxXOffset, 0, pltZOffset);
        fxBody.addGeom(leftFrontRailBox, -shortRailXOffset, railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(rightFrontRailBox, shortRailXOffset, railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(frontRailBox, 0, railYOffset, pltZOffset+halfPltHt+1.5*tetrixWidth);
        fxBody.addGeom(leftBackRailBox, -shortRailXOffset, -railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(rightBackRailBox, shortRailXOffset, -railYOffset, pltZOffset+halfPltHt+tetrixWidth/2.0);
        fxBody.addGeom(backRailBox, 0, -railYOffset, pltZOffset+halfPltHt+1.5*tetrixWidth);
        fxBody.addGeom(leftLiftBaseBox, -liftXOffset, liftBaseYOffset, liftZOffset);
        fxBody.addGeom(rightLiftBaseBox, liftXOffset, liftBaseYOffset, liftZOffset);

        //Generate an array of 3 Group objects for display of the 3 lift stages (other than the base stage)

        Group[] liftStages = new Group[3];
        for (int i=0; i<3; i++){
            liftStages[i] = new Group();
            Group left = Util3D.patternBox(liftThk, liftHt, liftThk, liftThk, liftThk, liftThk, slideMaterial);
            left.getTransforms().add(new Translate(-liftXOffset, 0,0));
            Group right = Util3D.patternBox(liftThk, liftHt, liftThk, liftThk, liftThk, liftThk, slideMaterial);
            right.getTransforms().add(new Translate(liftXOffset, 0, 0));
            liftStages[i].getChildren().addAll(left, right);
        }


        Group crossBar = Util3D.patternBox(crossBarLength, liftThk, liftThk, liftThk, liftThk, liftThk,
                slideMaterial);
        crossBar.getTransforms().addAll(new Translate(0, 0, crossBarZOffset));
        Box box = new Box(0.07, 0.03, 0.03);
        box.setMaterial(new PhongMaterial(Color.color(0.3, 0.3, 0.3)));
        box.getTransforms().add(new Translate(0, 0, 0.1));
        liftStages[2].getChildren().addAll(crossBar, box);


        Group slideGroup = new Group();
        Group slider = Parts.tetrixBox(tetrixWidth, sliderLength, tetrixWidth, tetrixWidth);
        box = new Box(0.032, 0.064, 0.032);
        box.setMaterial(new PhongMaterial(Color.color(0.3, 0.3, 0.3)));
        box.setTranslateZ(1.775);
        liftStages[4].getChildren().add(box);
        box = new Box(5,5, 0.125);
        box.setMaterial(new PhongMaterial(Color.color(0.6, 0.6, 0, 0.5)));
        box.setTranslateZ(1.0875);
        liftStages[4].getChildren().add(box);
        box = new Box(0.125, 5, 5);
        box.setMaterial(new PhongMaterial(Color.color(0.6, 0.6, 0, 0.5)));
        box.getTransforms().add(new Translate(-2.5, 0, -1.475));
        liftStages[4].getChildren().add(box);
        Box hand = new Box(0.125, 5, 5);
        hand.getTransforms().addAll(new Translate(2.5, 0, -1.475), handRotate);
        hand.setMaterial(new PhongMaterial(Color.color(0.6, 0, 0.6, 0.5)));
        liftStages[4].getChildren().add(hand);
        liftStages[4].getTransforms().add(sliderTranslate);

        Group lift = liftStages[0];
        lift.setTranslateZ(4.75);
        lift.setTranslateY(2.0);


        Group botGroup = new Group();
        botGroup.getChildren().add(chassis);
        botGroup.getChildren().addAll(wheels);
        botGroup.getChildren().add(lift);
        botGroup.setTranslateZ(2);
    }

    private void nearCallback(Object data, DGeom o1, DGeom o2){
        //TO DO: Collision Handling Code
    }



    @Override
    public synchronized void updateDisplay(){
        super.updateDisplay();
        //Probably the only thing needed here is to make the wheels appear to rotate
    }


    public void powerDownAndReset(){
        for (int i=0; i<4; i++) motors[i].stopAndReset();
        imu.close();
    }


}
