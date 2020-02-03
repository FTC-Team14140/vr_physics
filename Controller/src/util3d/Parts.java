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

    /**
     *  Create Skystone bridge assembly node
     *  Dimensions now in meters.
     * @return
     */
    public static Group skyStoneBridge(){
        Group group = new Group();

        PhongMaterial blueBridgeMaterial = new PhongMaterial(Color.BLUE);
        Cylinder blueBridge1 = new Cylinder(0.0127, 1.27);
        blueBridge1.setMaterial(blueBridgeMaterial);
        blueBridge1.getTransforms().addAll(
                new Translate(-VirtualRobotController.HALF_FIELD_WIDTH+0.6096, 0.0762, 0.3683),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder blueBridge2 = new Cylinder(0.0127, 1.27);
        blueBridge2.setMaterial(blueBridgeMaterial);
        blueBridge2.getTransforms().addAll(
                new Translate(-VirtualRobotController.HALF_FIELD_WIDTH+0.6096, -0.0762, 0.3683),
                new Rotate(90, Rotate.Z_AXIS)
        );
        PhongMaterial redBridgeMaterial = new PhongMaterial(Color.RED);
        Cylinder redBridge1 = new Cylinder(0.0127, 1.27);
        redBridge1.setMaterial(redBridgeMaterial);
        redBridge1.getTransforms().addAll(
                new Translate(VirtualRobotController.HALF_FIELD_WIDTH-0.6096, 0.0762, 0.3683),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder redBridge2 = new Cylinder(0.0127, 1.27);
        redBridge2.setMaterial(redBridgeMaterial);
        redBridge2.getTransforms().addAll(
                new Translate(VirtualRobotController.HALF_FIELD_WIDTH-0.6096, -0.0762, 0.3683),
                new Rotate(90, Rotate.Z_AXIS)
        );
        PhongMaterial neutralBridgeMaterial = new PhongMaterial(Color.ORANGE);
        Cylinder neutralBridge1 = new Cylinder(0.0127, 1.1938);
        neutralBridge1.setMaterial(neutralBridgeMaterial);
        neutralBridge1.getTransforms().addAll(
                new Translate(0, 0.0762, 0.5207),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder neutralBridge2 = new Cylinder(0.5, 47);
        neutralBridge2.setMaterial(neutralBridgeMaterial);
        neutralBridge2.getTransforms().addAll(
                new Translate(0, -0.0762, 0.5207),
                new Rotate(90, Rotate.Z_AXIS)
        );
        PhongMaterial bridgeStandMaterial = new PhongMaterial(Color.CORNSILK);
        Box bridgeStand1 = new Box(0.0254, 0.2032, 0.4064);
        bridgeStand1.setMaterial(bridgeStandMaterial);
        bridgeStand1.getTransforms().add(new Translate(-VirtualRobotController.HALF_FIELD_WIDTH-0.0127, 0, 0.2032));
        Box bridgeStand2 = new Box(0.0254, 0.2032, 0.5588);
        bridgeStand2.setMaterial(bridgeStandMaterial);
        bridgeStand2.getTransforms().add(new Translate(-0.5842, 0, 0.2794));
        Box bridgeStand3 = new Box(0.0254, 0.2032, 0.5588);
        bridgeStand3.setMaterial(bridgeStandMaterial);
        bridgeStand3.getTransforms().add(new Translate(0.5842, 0, 0.2794));
        Box bridgeStand4 = new Box(0.0254, 0.2032, 0.4064);
        bridgeStand4.setMaterial(bridgeStandMaterial);
        bridgeStand4.getTransforms().add(new Translate(VirtualRobotController.HALF_FIELD_WIDTH+0.0127, 0, 0.2032));

        group.getChildren().addAll(blueBridge1, blueBridge2, redBridge1, redBridge2, neutralBridge1, neutralBridge2,
                bridgeStand1, bridgeStand2, bridgeStand3, bridgeStand4);

        return group;
    }


}
