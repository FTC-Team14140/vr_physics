package util3d;

import javafx.scene.Group;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.OdeHelper;
import virtual_robot.controller.VirtualRobotController;

import java.util.List;

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
     *  Dimensions now in centimeters.
     * @return
     */
    public static Group skyStoneBridge(DSpace space){
        Group group = new Group();

        double tubeRadius = 1.27;
        double tubeHeight = 127;
        double neutralTubeHeight = 119.38;
        double tubeXOffset = 60.96;
        double tubeYOffset = 7.62;
        double tubeZOffset = 36.83;
        double neutralTubeZOffset = 52.07;
        double bridgeStandThickness = 2.54;
        double bridgeStandWidth = 20.32;
        double innerBridgeStandHeight = 55.88;
        double outerBridgeStandHeight = 40.64;
        double innerBridgeStandXOffset = 58.42;

        PhongMaterial blueBridgeMaterial = new PhongMaterial(Color.BLUE);
        Cylinder blueBridge1 = new Cylinder(tubeRadius, tubeHeight);
        blueBridge1.setMaterial(blueBridgeMaterial);
        blueBridge1.getTransforms().addAll(
                new Translate(-VirtualRobotController.HALF_FIELD_WIDTH+tubeXOffset, tubeYOffset, tubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder blueBridge2 = new Cylinder(tubeRadius, tubeHeight);
        blueBridge2.setMaterial(blueBridgeMaterial);
        blueBridge2.getTransforms().addAll(
                new Translate(-VirtualRobotController.HALF_FIELD_WIDTH+tubeXOffset, -tubeYOffset, tubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );

        DGeom blueBridgeGeom = OdeHelper.createBox(space, tubeHeight, 2*tubeYOffset+2*tubeRadius, 2*tubeRadius);
        blueBridgeGeom.setPosition(-VirtualRobotController.HALF_FIELD_WIDTH+tubeXOffset, 0, tubeZOffset);

        PhongMaterial redBridgeMaterial = new PhongMaterial(Color.RED);
        Cylinder redBridge1 = new Cylinder(tubeRadius, tubeHeight);
        redBridge1.setMaterial(redBridgeMaterial);
        redBridge1.getTransforms().addAll(
                new Translate(VirtualRobotController.HALF_FIELD_WIDTH-tubeXOffset, tubeYOffset, tubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder redBridge2 = new Cylinder(tubeRadius, tubeHeight);
        redBridge2.setMaterial(redBridgeMaterial);
        redBridge2.getTransforms().addAll(
                new Translate(VirtualRobotController.HALF_FIELD_WIDTH-tubeXOffset, -tubeYOffset, tubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );

        DGeom redBridgeGeom = OdeHelper.createBox(space, tubeHeight, 2*tubeYOffset+2*tubeRadius, 2*tubeRadius);
        redBridgeGeom.setPosition(VirtualRobotController.HALF_FIELD_WIDTH-tubeXOffset, 0, tubeZOffset);

        PhongMaterial neutralBridgeMaterial = new PhongMaterial(Color.ORANGE);
        Cylinder neutralBridge1 = new Cylinder(tubeRadius, neutralTubeHeight);
        neutralBridge1.setMaterial(neutralBridgeMaterial);
        neutralBridge1.getTransforms().addAll(
                new Translate(0, tubeYOffset, neutralTubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );
        Cylinder neutralBridge2 = new Cylinder(tubeRadius, neutralTubeHeight);
        neutralBridge2.setMaterial(neutralBridgeMaterial);
        neutralBridge2.getTransforms().addAll(
                new Translate(0, -tubeYOffset, neutralTubeZOffset),
                new Rotate(90, Rotate.Z_AXIS)
        );

        DGeom neutralBridgeGeom = OdeHelper.createBox(space, neutralTubeHeight, 2*tubeYOffset+2*tubeRadius, 2*tubeRadius);
        neutralBridgeGeom.setPosition(0, 0, neutralTubeZOffset);

        PhongMaterial bridgeStandMaterial = new PhongMaterial(Color.CORNSILK);

        Box bridgeStand1 = new Box(bridgeStandThickness, bridgeStandWidth, outerBridgeStandHeight);
        bridgeStand1.setMaterial(bridgeStandMaterial);
        bridgeStand1.getTransforms().add(new Translate(-VirtualRobotController.HALF_FIELD_WIDTH-tubeRadius, 0, outerBridgeStandHeight/2.0));

        DGeom bridgeStand1Geom = OdeHelper.createBox(space, bridgeStandThickness, bridgeStandWidth, outerBridgeStandHeight);
        bridgeStand1Geom.setPosition(-VirtualRobotController.HALF_FIELD_WIDTH-tubeRadius, 0, outerBridgeStandHeight/2.0);

        Box bridgeStand2 = new Box(bridgeStandThickness, bridgeStandWidth, innerBridgeStandHeight);
        bridgeStand2.setMaterial(bridgeStandMaterial);
        bridgeStand2.getTransforms().add(new Translate(-innerBridgeStandXOffset, 0, innerBridgeStandHeight/2.0));

        DGeom bridgeStand2Geom = OdeHelper.createBox(space, bridgeStandThickness, bridgeStandWidth, innerBridgeStandHeight);
        bridgeStand2Geom.setPosition(-innerBridgeStandXOffset, 0, innerBridgeStandHeight/2.0);

        Box bridgeStand3 = new Box(bridgeStandThickness, bridgeStandWidth, innerBridgeStandHeight);
        bridgeStand3.setMaterial(bridgeStandMaterial);
        bridgeStand3.getTransforms().add(new Translate(innerBridgeStandXOffset, 0, innerBridgeStandHeight/2.0));

        DGeom bridgeStand3Geom = OdeHelper.createBox(space, bridgeStandThickness, bridgeStandWidth, innerBridgeStandHeight);
        bridgeStand3Geom.setPosition(innerBridgeStandXOffset, 0, innerBridgeStandHeight/2.0);

        Box bridgeStand4 = new Box(bridgeStandThickness, bridgeStandWidth, outerBridgeStandHeight);
        bridgeStand4.setMaterial(bridgeStandMaterial);
        bridgeStand4.getTransforms().add(new Translate(VirtualRobotController.HALF_FIELD_WIDTH+tubeRadius, 0, outerBridgeStandHeight/2.0));

        DGeom bridgeStand4Geom = OdeHelper.createBox(space, bridgeStandThickness, bridgeStandWidth, outerBridgeStandHeight);
        bridgeStand4Geom.setPosition(VirtualRobotController.HALF_FIELD_WIDTH+tubeRadius, 0, outerBridgeStandHeight/2.0);

        group.getChildren().addAll(blueBridge1, blueBridge2, redBridge1, redBridge2, neutralBridge1, neutralBridge2,
                bridgeStand1, bridgeStand2, bridgeStand3, bridgeStand4);

        return group;
    }


}
