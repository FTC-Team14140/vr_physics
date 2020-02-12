package virtual_robot.controller.robots;

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
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with two standard wheels, color sensor, four distance sensors,
 * a Gyro Sensor, and a Servo-controlled arm on the back.
 *
 * TwoWheelBot is the controller class for the "two_wheel_bot.fxml" markup file.
 */
//@BotConfig(name = "Two Wheel Bot")
public abstract class TwoWheelBot extends VirtualBot {

    public final MotorType motorType = MotorType.Neverest40;
    private DcMotorImpl leftMotor = null;
    private DcMotorImpl rightMotor = null;
    private DcMotorImpl armExtensionMotor = null;
    private DcMotorImpl armRotationMotor = null;
    private GyroSensorImpl gyro = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private ServoImpl fingerServo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    private double wheelCircumference;
    private double interWheelDistance;
    Rotate armRotate = new Rotate(0, 0, -5.5, 0, new Point3D(1, 0, 0));
    Translate midArmTranslate = new Translate(0, 0, 0);
    Translate foreArmTranslate = new Translate(0, 0, 0);
    Translate leftFingerTranslate = new Translate(0, 0, 0);
    Translate rightFingerTranslate = new Translate(0, 0, 0);

    double armRotation = 0;
    double armExtension = 0;



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
        gyro = (GyroSensorImpl)hardwareMap.gyroSensor.get("gyro_sensor");
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
        hardwareMap.put("gyro_sensor", new GyroSensorImpl(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("finger_servo", new ServoImpl());
    }

    public synchronized void updateStateAndSensors(double millis){
        double deltaLeftPos = leftMotor.update(millis);
        double deltaRightPos = rightMotor.update(millis);
        double leftWheelDist = -deltaLeftPos * wheelCircumference / motorType.TICKS_PER_ROTATION;
        double rightWheelDist = deltaRightPos * wheelCircumference / motorType.TICKS_PER_ROTATION;
        double distTraveled = (leftWheelDist + rightWheelDist) / 2.0;
        double headingChange = (rightWheelDist - leftWheelDist) / interWheelDistance;
//        double deltaRobotX = -distTraveled * Math.sin(headingRadians + headingChange / 2.0);
//        double deltaRobotY = distTraveled * Math.cos(headingRadians + headingChange / 2.0);
//        x += deltaRobotX;
//        y += deltaRobotY;
//        if (x >  (halfFieldWidth - halfBotWidth)) x = halfFieldWidth - halfBotWidth;
//        else if (x < (halfBotWidth - halfFieldWidth)) x = halfBotWidth - halfFieldWidth;
//        if (y > (halfFieldWidth - halfBotWidth)) y = halfFieldWidth - halfBotWidth;
//        else if (y < (halfBotWidth - halfFieldWidth)) y = halfBotWidth - halfFieldWidth;
//        headingRadians += headingChange;
//        if (headingRadians > Math.PI) headingRadians -= 2.0 * Math.PI;
//        else if (headingRadians < -Math.PI) headingRadians += 2.0 * Math.PI;
//        gyro.updateHeading(headingRadians * 180.0 / Math.PI);
//        colorSensor.updateColor(x, y);
//        final double piOver2 = Math.PI / 2.0;
//        for (int i = 0; i<4; i++){
//            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
//            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
//                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
//        }
//
//        double newArmRotation = armRotation + 0.05 * armRotationMotor.update(millis);
//        armRotation = Math.max(0, Math.min(90, newArmRotation));
//        double newArmExtension = armExtension + 0.01 * armExtensionMotor.update(millis);
//        armExtension = Math.max(0, Math.min(22, newArmExtension));
    }

    protected Group getDisplayGroup(){
        Box chassis = new Box(14, 18, 2);
        PhongMaterial chassisMaterial = new PhongMaterial(Color.YELLOW);
        chassisMaterial.setSpecularColor(Color.WHITE);
        chassis.setMaterial(chassisMaterial);
        Cylinder[] wheels = new Cylinder[2];
        PhongMaterial wheelMaterial = new PhongMaterial(Color.BLUE);
        wheelMaterial.setSpecularColor(Color.WHITE);
        for (int i=0; i<2; i++){
            wheels[i] = new Cylinder(2,2);
            wheels[i].setMaterial(wheelMaterial);
            wheels[i].setRotationAxis(new Point3D(0, 0, 1));
            wheels[i].setRotate(90);
            wheels[i].setTranslateX(i==0? -8 : 8);
        }

        PhongMaterial armMaterial = new PhongMaterial(Color.FUCHSIA);
        armMaterial.setSpecularColor(Color.WHITE);
        Box arm = new Box(1, 12, 1);
        arm.setMaterial(armMaterial);
        Box midArm = new Box(1, 12, 1);
        midArm.setMaterial(armMaterial);
        Box foreArm = new Box(1, 12, 1);
        foreArm.setMaterial(armMaterial);
        Box hand = new Box(6, 1, 1);
        hand.setTranslateY(6);
        hand.setMaterial(armMaterial);
        Box  leftFinger = new Box(1, 4, 1);
        leftFinger.setTranslateY(8);
        leftFinger.setTranslateX(-2.5);
        leftFinger.setMaterial(armMaterial);
        Box  rightFinger = new Box(1, 4, 1);
        rightFinger.setTranslateY(8);
        rightFinger.setTranslateX(2.5);
        rightFinger.setMaterial(armMaterial);
        leftFinger.getTransforms().add(leftFingerTranslate);
        rightFinger.getTransforms().add(rightFingerTranslate);
        Group foreArmGroup = new Group(foreArm, hand, leftFinger, rightFinger);
        foreArmGroup.setTranslateZ(1);
        foreArmGroup.getTransforms().add(foreArmTranslate);
        Group midArmGroup = new Group(midArm, foreArmGroup);
        midArmGroup.setTranslateZ(1);
        midArmGroup.getTransforms().add(midArmTranslate);
        Group armGroup = new Group(arm, midArmGroup);
        armGroup.setTranslateZ(1.5);
        armGroup.setTranslateY(-3);
        armGroup.getTransforms().add(armRotate);


        Group botGroup = new Group();
        botGroup.getChildren().add(chassis);
        botGroup.getChildren().addAll(wheels);
        botGroup.getChildren().add(armGroup);
        botGroup.setTranslateZ(2);
        return botGroup;
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        armRotate.setAngle(armRotation);
        midArmTranslate.setY(armExtension/2.0);
        foreArmTranslate.setY(armExtension/2.0);
        double fingerMovement = fingerServo.getInternalPosition();
        leftFingerTranslate.setX(fingerMovement);
        rightFingerTranslate.setX(-fingerMovement);
    }

    public void powerDownAndReset(){
        leftMotor.stopAndReset();
        rightMotor.stopAndReset();
        gyro.deinit();
    }


}
