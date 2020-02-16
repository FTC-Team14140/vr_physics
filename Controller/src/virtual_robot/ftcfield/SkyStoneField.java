package virtual_robot.ftcfield;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import odefx.CBits;
import odefx.FxBody;
import odefx.FxBodyHelper;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.*;
import util3d.Parts;
import util3d.Util3D;
import virtual_robot.config.Config;
import virtual_robot.controller.VirtualRobotController;

public class SkyStoneField extends FtcField {

    private final double HALF_FIELD_WIDTH;
    private final Image backgroundImage = Config.BACKGROUND;

    FxBody[] stones = new FxBody[12];
    FxBody[] foundations = new FxBody[2];

    public SkyStoneField(Group group, DWorld world, DSpace space){
        super(group, world, space);
        HALF_FIELD_WIDTH = VirtualRobotController.HALF_FIELD_WIDTH;
    }

    @Override
    public void setup() {

        TriangleMesh fieldMesh = Util3D.getParametricMesh(-HALF_FIELD_WIDTH, HALF_FIELD_WIDTH,-HALF_FIELD_WIDTH, HALF_FIELD_WIDTH,
                10, 10, false, false, new Util3D.Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return s;
                    }

                    @Override
                    public float y(float s, float t) {
                        return t;
                    }

                    @Override
                    public float z(float s, float t) {
                        return 0;
                    }
                });
        MeshView fieldView = new MeshView(fieldMesh);
        PhongMaterial fieldMaterial = new PhongMaterial();
        fieldMaterial.setDiffuseColor(Color.gray(0.02));
        fieldMaterial.setDiffuseMap(backgroundImage);
        fieldMaterial.setSelfIlluminationMap(backgroundImage);
        fieldView.setMaterial(fieldMaterial);
        subSceneGroup.getChildren().add(fieldView);

        DPlane fieldPlane = OdeHelper.createPlane(space, 0, 0, 1, 0);
        fieldPlane.setData("Field Plane");
        fieldPlane.setCategoryBits(CBits.FLOOR);

        DSpace bridgeSpace = OdeHelper.createSimpleSpace(space);
        Group bridgeGroup = Parts.skyStoneBridge(bridgeSpace);
        for (DGeom g: bridgeSpace.getGeoms()){
            g.setCategoryBits(CBits.BRIDGE);
        }


        subSceneGroup.getChildren().add(bridgeGroup);

        PhongMaterial wallMaterial = new PhongMaterial(Color.color(0.02, 0.02, 0.02, 0));
        for (int i=0; i<4; i++){
            TriangleMesh wallMesh = Util3D.getParametricMesh(-HALF_FIELD_WIDTH, HALF_FIELD_WIDTH,-15, 15,
                    10, 10, false, false, new Util3D.Param3DEqn() {
                        @Override
                        public float x(float s, float t) {
                            return s;
                        }

                        @Override
                        public float y(float s, float t) {
                            return 0;
                        }

                        @Override
                        public float z(float s, float t) {
                            return t;
                        }
                    });
            MeshView wallView = new MeshView(wallMesh);
            wallView.setMaterial(wallMaterial);
            double dx = i==0? 0 : i==1? -HALF_FIELD_WIDTH-2 : i==2? 0 : HALF_FIELD_WIDTH+2;
            double dy = i==0? HALF_FIELD_WIDTH+2 : i==1? 0 : i==2? -HALF_FIELD_WIDTH-2 : 0;
            double angle = i==0? 0 : i==1? 90 : i==2? 180 : -90;
            wallView.getTransforms().addAll(new Translate(dx, dy, 15), new Rotate(angle, Rotate.Z_AXIS));
            subSceneGroup.getChildren().add(wallView);
            FxBodyHelper.dGeomsFromNode(wallView, space, null);
        }


        DMass testBlockMass = OdeHelper.createMass();
        testBlockMass.setBox(1, 20, 10, 10);
        testBlockMass.setMass(50);
        PhongMaterial yellowMat = new PhongMaterial(Color.YELLOW);
        PhongMaterial blackMat = new PhongMaterial((Color.BLACK));

        for (int i=0; i<12; i++){
            stones[i] = FxBody.newInstance(world, space);
            stones[i].setMass(testBlockMass);
            Node testBlockBox;
            if (i%3 == 0) {
                testBlockBox = Util3D.box(10, 20, 10, 1, 1, 1,
                        yellowMat, yellowMat, blackMat, yellowMat);
            } else {
                testBlockBox = new Box(10, 20, 10);
                ((Box)testBlockBox).setMaterial(yellowMat);
            }
            if (i>5) testBlockBox.getTransforms().add(new Rotate(180, Rotate.Z_AXIS));
            subSceneGroup.getChildren().add(testBlockBox);
            stones[i].setNode(testBlockBox, true);
            stones[i].setPosition(i<6? -HALF_FIELD_WIDTH+125 : HALF_FIELD_WIDTH-125, -HALF_FIELD_WIDTH + 10 + (i%6)*22, 5);
            stones[i].setCategoryBits(CBits.STONES);
            stones[i].setCollideBits(0xFF);
        }


    }

    @Override
    public void reset() {
        for (int i=0; i<12; i++){
            stones[i].setPosition(i<6? -HALF_FIELD_WIDTH+125 : HALF_FIELD_WIDTH-125, -HALF_FIELD_WIDTH + 10 + (i%6)*22, 5);
            DMatrix3 rot = new DMatrix3();
            DRotation.dRFromAxisAndAngle(rot, 0, 0, 1, 0);
            stones[i].setRotation(rot);
        }
    }

    @Override
    public void updateDisplay() {
        for (int i=0; i<stones.length; i++){
            stones[i].updateNodeDisplay();
        }
    }

}
