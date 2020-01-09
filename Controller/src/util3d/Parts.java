package util3d;

import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import virtual_robot.controller.VirtualRobotController;

public class Parts {

    public static final int BACK_LEFT = 0, FRONT_LEFT = 1, FRONT_RIGHT = 0, BACK_RIGHT = 1;

    public static Group mecanumWheel(double diameter, double width, int type){
        PhongMaterial wheelTreadMaterial = type%2 == 0?
                Util3D.imageMaterial("/virtual_robot/assets/mechwheelA_rotated.jpg") :
                Util3D.imageMaterial("/virtual_robot/assets/mechwheelB_rotated.jpg");
        PhongMaterial wheelSideMaterial = new PhongMaterial(Color.color(0.9, 0.9, 0.9));
        wheelSideMaterial.setSpecularColor(Color.color(0, 0, 0, 0));
        Group wheel = Util3D.cylinder((float)diameter/2.0f, (float)width, 10, 1, 1,
                true, wheelTreadMaterial, wheelSideMaterial);
        return wheel;
    }

    public static Group tetrixBox(float length, float height, float depth, float patternWidth){
        PhongMaterial tetrixMaterial = Util3D.imageMaterial("/virtual_robot/assets/tetrix.jpg");
        return Util3D.patternBox(length, height, depth, patternWidth, patternWidth, patternWidth, tetrixMaterial);
    }
    
    public static Group skyStoneBridge(){
        Group group = new Group();

        PhongMaterial blueBridgeMaterial = new PhongMaterial(Color.BLUE);
        Cylinder blueBridge1 = new Cylinder(0.5, 50);
        blueBridge1.setMaterial(blueBridgeMaterial);
        blueBridge1.getTransforms().addAll(
                new Translate(-VirtualRobotController.HALF_FIELD_WIDTH+24, 3.0, 14.5),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder blueBridge2 = new Cylinder(0.5, 50);
        blueBridge2.setMaterial(blueBridgeMaterial);
        blueBridge2.getTransforms().addAll(
                new Translate(-VirtualRobotController.HALF_FIELD_WIDTH+24, -3.0, 14.5),
                new Rotate(90, Rotate.Z_AXIS)
        );
        PhongMaterial redBridgeMaterial = new PhongMaterial(Color.RED);
        Cylinder redBridge1 = new Cylinder(0.5, 50);
        redBridge1.setMaterial(redBridgeMaterial);
        redBridge1.getTransforms().addAll(
                new Translate(VirtualRobotController.HALF_FIELD_WIDTH-24, 3.0, 14.5),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder redBridge2 = new Cylinder(0.5, 50);
        redBridge2.setMaterial(redBridgeMaterial);
        redBridge2.getTransforms().addAll(
                new Translate(VirtualRobotController.HALF_FIELD_WIDTH-24, -3.0, 14.5),
                new Rotate(90, Rotate.Z_AXIS)
        );
        PhongMaterial neutralBridgeMaterial = new PhongMaterial(Color.ORANGE);
        Cylinder neutralBridge1 = new Cylinder(0.5, 47);
        neutralBridge1.setMaterial(neutralBridgeMaterial);
        neutralBridge1.getTransforms().addAll(
                new Translate(0, 3.0, 20.5),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder neutralBridge2 = new Cylinder(0.5, 47);
        neutralBridge2.setMaterial(neutralBridgeMaterial);
        neutralBridge2.getTransforms().addAll(
                new Translate(0, -3.0, 20.5),
                new Rotate(90, Rotate.Z_AXIS)
        );
        PhongMaterial bridgeStandMaterial = new PhongMaterial(Color.CORNSILK);
        Box bridgeStand1 = new Box(1, 8, 16);
        bridgeStand1.setMaterial(bridgeStandMaterial);
        bridgeStand1.getTransforms().add(new Translate(-VirtualRobotController.HALF_FIELD_WIDTH-0.5, 0, 8));
        Box bridgeStand2 = new Box(1, 8, 22);
        bridgeStand2.setMaterial(bridgeStandMaterial);
        bridgeStand2.getTransforms().add(new Translate(-23, 0, 11));
        Box bridgeStand3 = new Box(1, 8, 22);
        bridgeStand3.setMaterial(bridgeStandMaterial);
        bridgeStand3.getTransforms().add(new Translate(23, 0, 11));
        Box bridgeStand4 = new Box(1, 8, 16);
        bridgeStand4.setMaterial(bridgeStandMaterial);
        bridgeStand4.getTransforms().add(new Translate(VirtualRobotController.HALF_FIELD_WIDTH+0.5, 0, 8));

        group.getChildren().addAll(blueBridge1, blueBridge2, redBridge1, redBridge2, neutralBridge1, neutralBridge2,
                bridgeStand1, bridgeStand2, bridgeStand3, bridgeStand4);

        return group;
    }


}
