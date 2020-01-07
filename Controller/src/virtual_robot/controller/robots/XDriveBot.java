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
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import javafx.scene.transform.Translate;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with four omni wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a Continuous Rotation Servo-controlled arm on the back.
 *
 * XDriveBot is the controller class for the "xdrive_bot.fxml" markup file.
 *
 */
@BotConfig(name = "XDrive Bot")
public class XDriveBot extends VirtualBot {

    MotorType motorType;
    private DcMotorImpl[] motors = null;
    private DcMotorImpl armExtensionMotor = null;
    private DcMotorImpl armRotationMotor = null;
    private BNO055IMUImpl imu = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private ServoImpl fingerServo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    private double wheelCircumference;
    private double wheelBaseRadius;

    private double[][] tWR; //Transform from wheel motion to robot motion

    Rotate armRotate = new Rotate(0, 0, -5.5, 0, new Point3D(1, 0, 0));
    Translate midArmTranslate = new Translate(0, 0, 0);
    Translate foreArmTranslate = new Translate(0, 0, 0);
    Translate leftFingerTranslate = new Translate(0, 0, 0);
    Translate rightFingerTranslate = new Translate(0, 0, 0);

    double armRotation = 0;
    double armExtension = 0;


    public XDriveBot() {
        super();
        motors = new DcMotorImpl[]{
                (DcMotorImpl)hardwareMap.dcMotor.get("back_left_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("front_left_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("front_right_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("back_right_motor")
        };
        armExtensionMotor = (DcMotorImpl)hardwareMap.dcMotor.get("arm_extension_motor");
        armRotationMotor = (DcMotorImpl)hardwareMap.dcMotor.get("arm_rotation_motor");
        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        //gyro = (VirtualRobotController.GyroSensorImpl)hardwareMap.gyroSensor.get("gyro_sensor");
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        fingerServo = (ServoImpl)hardwareMap.servo.get("finger_servo");
        wheelCircumference = Math.PI * botWidth / 4.5;
        double sqrt2 = Math.sqrt(2);
        wheelBaseRadius = botWidth * (1.0/sqrt2 - 5.0/36.0);

        tWR = new double[][] {
                {-0.25*sqrt2, 0.25*sqrt2, -0.25*sqrt2, 0.25*sqrt2},
                {0.25*sqrt2, 0.25*sqrt2, 0.25*sqrt2, 0.25*sqrt2},
                {-0.25/ wheelBaseRadius, -0.25/ wheelBaseRadius, 0.25/ wheelBaseRadius, 0.25/ wheelBaseRadius},
                {-0.25, 0.25, 0.25, -0.25}
        };
    }

    protected void createHardwareMap(){
        motorType = MotorType.Neverest40;
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[] {"back_left_motor", "front_left_motor", "front_right_motor",
                "back_right_motor", "arm_extension_motor", "arm_rotation_motor"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorImpl(motorType));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("finger_servo", new ServoImpl());
    }

    public synchronized void updateStateAndSensors(double millis){
        double[] deltaPos = new double[4];
        double[] w = new double[4];

        for (int i = 0; i < 4; i++) {
            deltaPos[i] = motors[i].update(millis);
            w[i] = deltaPos[i] * wheelCircumference / motorType.TICKS_PER_ROTATION;
            if (i < 2) w[i] = -w[i];
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
        //gyro.updateHeading(headingRadians * 180.0 / Math.PI);
        imu.updateHeadingRadians(headingRadians);

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;
        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

        double newArmRotation = armRotation + 0.05 * armRotationMotor.update(millis);
        armRotation = Math.max(0, Math.min(90, newArmRotation));
        double newArmExtension = armExtension + 0.01 * armExtensionMotor.update(millis);
        armExtension = Math.max(0, Math.min(22, newArmExtension));

    }

    protected Group getDisplayGroup(){
        Box chassis = new Box(15, 15, 2);
        PhongMaterial chassisMaterial = new PhongMaterial(Color.YELLOW);
        chassisMaterial.setSpecularColor(Color.WHITE);
        chassis.setMaterial(chassisMaterial);
        Cylinder[] wheels = new Cylinder[4];
        PhongMaterial wheelMaterial = new PhongMaterial(Color.BLUE);
        wheelMaterial.setSpecularColor(Color.WHITE);
        for (int i=0; i<4; i++){
            wheels[i] = new Cylinder(2,2);
            wheels[i].setMaterial(wheelMaterial);
            wheels[i].setRotationAxis(new Point3D(0, 0, 1));
            wheels[i].setRotate(i==0 || i==3? -45 : 45);
            wheels[i].setTranslateX(i<2? -7.25 : 7.25);
            wheels[i].setTranslateY(i%2==0? -7.25 : 7.25);
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
        armGroup.setTranslateY(-1.5);
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
        for (int i=0; i<4; i++) motors[i].stopAndReset();
        imu.close();
    }


}
