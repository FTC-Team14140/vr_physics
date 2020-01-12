package virtual_robot.controller.robots;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.*;
import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
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

    Rotate handRotate = new Rotate(0, 0, 0, 2.5, new Point3D(0,1,0));
    Translate sliderTranslate = new Translate(0, 0, 0);
    Translate[] liftTranslates = new Translate[]{new Translate(0, 0, 0), new Translate(0, 0, 0), new Translate(0, 0, 0)};
    Rotate[] wheelRotates = new Rotate[] {new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS), new Rotate(0, Rotate.Y_AXIS)};

    double handRotation = 0;
    double sliderTranslation = 0;
    double liftElevation = 0;
    double[] wheelRotations = new double[]{0,0,0,0};

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

    public synchronized void updateStateAndSensors(double millis){

        double[] deltaPos = new double[4];
        double[] w = new double[4];

        for (int i = 0; i < 4; i++) {
            deltaPos[i] = motors[i].update(millis);
            w[i] = deltaPos[i] * wheelCircumference / motorType.TICKS_PER_ROTATION;
            double wheelRotationDegrees = 360.0 * deltaPos[i] / motorType.TICKS_PER_ROTATION;
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
        double headingChange = robotDeltaPos[2];
        double avgHeading = headingRadians + headingChange / 2.0;

        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);

        x += dxR * cos - dyR * sin;
        y += dxR * sin + dyR * cos;
        headingRadians += headingChange;

        if (x >  (halfFieldWidth - halfBotWidth)) x = halfFieldWidth - halfBotWidth;
        else if (x < (halfBotWidth - halfFieldWidth)) x = halfBotWidth - halfFieldWidth;
        if (y > (halfFieldWidth - halfBotWidth)) y = halfFieldWidth - halfBotWidth;
        else if (y < (halfBotWidth - halfFieldWidth)) y = halfBotWidth - halfFieldWidth;

        if (headingRadians > Math.PI) headingRadians -= 2.0 * Math.PI;
        else if (headingRadians < -Math.PI) headingRadians += 2.0 * Math.PI;

        imu.updateHeadingRadians(headingRadians);

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;

        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

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

    protected Group getDisplayGroup(){
        Group chassis = new Group();

        Group[] plates = new Group[4];
        PhongMaterial plateMaterial = new PhongMaterial(Color.color(0.6, 0.3, 0));
        for (int i=0; i<4; i++){
            plates[i] = Util3D.polygonBox(0.125f, new float[]{2,-8.5f,2,8.5f,1,8.5f,-2,7.5f,-2,-7.5f,1,-8.5f},
                    plateMaterial);
            float x = (i<2? -1.0f : 1.0f) * (i%2 == 0? 8.5f:5.5f);
            plates[i].getTransforms().addAll(new Translate(x, 0, 0.25), new Rotate(-90, Rotate.Y_AXIS));
        }

        Group frontLeftRail = Parts.tetrixBox(1.25f, 3.125f, 1.25f, 1.25f);
        frontLeftRail.getTransforms().addAll(new Translate(-7, 7.875, 2.875), new Rotate(90, Rotate.Z_AXIS));
        Group frontRightRail = Parts.tetrixBox(1.25f, 3.125f, 1.25f, 1.25f);
        frontRightRail.getTransforms().addAll(new Translate(7, 7.875, 2.875), new Rotate(90, Rotate.Z_AXIS));
        Group frontRail = Parts.tetrixBox(1.25f, 17.125f, 1.25f, 1.25f);
        frontRail.getTransforms().addAll(new Translate(0, 7.875, 4.125f), new Rotate(90, new Point3D(0,0,1)));
        Group backRail = Parts.tetrixBox(1.25f, 17.125f, 2.5f, 1.25f);
        backRail.getTransforms().addAll(new Translate(0, -7.875, 3.5), new Rotate(90, new Point3D(0, 0, 1)));
        Box center = new Box(10, 14, 2);
        center.setMaterial(new PhongMaterial(Color.YELLOW));


        chassis.getChildren().addAll(plates);
        chassis.getChildren().addAll(frontLeftRail, frontRightRail, frontRail, backRail);

        Group[] wheels = new Group[4];
        for (int i=0; i<4; i++){
            wheels[i] = Parts.mecanumWheel(4, 2, i);
            wheels[i].setRotationAxis(new Point3D(0, 0, 1));
            wheels[i].setRotate(90);
            wheels[i].setTranslateX(i<2? -7 : 7);
            wheels[i].setTranslateY(i==0 || i==3? -6.5 : 6.5);
            wheels[i].getTransforms().add(wheelRotates[i]);
        }

        PhongMaterial slideMaterial = Util3D.imageMaterial("/virtual_robot/assets/rev_slide.jpg");
        Group[] liftStages = new Group[6];
        for (int i=0; i<4; i++){
            liftStages[i] = new Group();
            Group left = Util3D.patternBox(0.6f, 13f, 0.6f, 0.6f, 0.6f, 0.6f, slideMaterial);
            left.getTransforms().addAll(new Translate(-5.8625, 0, 0), new Rotate(90, Rotate.X_AXIS));
            Group right = Util3D.patternBox(0.6f, 13f, 0.6f, 0.6f, 0.6f, 0.6f, slideMaterial);
            right.getTransforms().addAll(new Translate(5.8625, 0, 0), new Rotate(90, Rotate.X_AXIS));
            liftStages[i].getChildren().addAll(left, right);
            if (i>0) {
                liftStages[i].setTranslateY(0.6f);
                liftStages[i - 1].getChildren().add(liftStages[i]);
                liftStages[i].getTransforms().add(liftTranslates[i-1]);
            }
        }
        Group crossBar = Util3D.patternBox(0.6f, 11.125f, 0.6f, 0.6f, 0.6f, 0.6f,
                slideMaterial);
        crossBar.getTransforms().addAll(new Translate(0, 0, 5.2), new Rotate(90, Rotate.Z_AXIS));
        Box box = new Box(2.5, 1.25, 1.75);
        box.setMaterial(new PhongMaterial(Color.color(0.3, 0.3, 0.3)));
        box.setTranslateZ(4.025);
        liftStages[3].getChildren().addAll(crossBar, box);
        liftStages[4] = new Group();
        liftStages[3].getChildren().add(liftStages[4]);
        Group slider = Parts.tetrixBox(1.25f, 16, 1.25f, 1.25f);
        slider.setTranslateZ(3.025);
        slider.setTranslateY(-3.8);
        liftStages[4].getChildren().add(slider);
        box = new Box(1.25, 2.5, 1.25);
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
        return botGroup;
    }

    @Override
    public synchronized void updateDisplay(){
        super.updateDisplay();
        handRotate.setAngle(handRotation);
        sliderTranslate.setY(sliderTranslation);
        for (int i=1; i<4; i++){
            liftTranslates[i-1].setZ(liftElevation / 3.0);
        }
        for (int i=0; i<4; i++){
            wheelRotates[i].setAngle(wheelRotations[i]);
        }
    }


    public void powerDownAndReset(){
        for (int i=0; i<4; i++) motors[i].stopAndReset();
        imu.close();
    }


}
