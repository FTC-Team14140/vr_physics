package virtual_robot.ftcfield;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import odefx.CBits;
import odefx.FxBody;
import odefx.FxBodyHelper;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import util3d.ClosedMeshCreator;
import util3d.Parts;
import util3d.Util3D;
import virtual_robot.config.Config;
import virtual_robot.controller.VirtualRobotController;

import java.util.List;

import static org.ode4j.ode.OdeConstants.*;
import static org.ode4j.ode.OdeConstants.dContactBounce;

public class SkyStoneField extends FtcField {

    private final double HALF_FIELD_WIDTH;
    private final Image backgroundImage = Config.BACKGROUND;

    FxBody[] stones = new FxBody[12];
    FxBody foundationBlue;
    FxBody foundationRed;

    private DMass foundationMass;

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
            List<DGeom> wallGeoms = FxBodyHelper.dGeomsFromNode(wallView, space, null);
            for (DGeom dg: wallGeoms) dg.setCategoryBits(CBits.WALLS);
        }


        DMass testBlockMass = OdeHelper.createMass();
        testBlockMass.setBox(1, 20, 10, 10);
        testBlockMass.setMass(50);
        PhongMaterial yellowMat = new PhongMaterial(Color.YELLOW);
        PhongMaterial blackMat = new PhongMaterial((Color.BLACK));

        StoneMeshCreator stoneMeshCreator = new StoneMeshCreator();

        for (int i=0; i<12; i++){
            stones[i] = FxBody.newInstance(world, space);
            stones[i].setMass(testBlockMass);

            // Stones Created with bumps on top and interlocking depressions in the bottom don't stack well
            // They also get hung up on the sides of the grabber in an irritating way.
//            Mesh stoneMesh = stoneMeshCreator.newMeshInstance();
//            MeshView stoneView = new MeshView(stoneMesh);
//            stoneView.setMaterial(yellowMat);
//            if (i>5) stoneView.getTransforms().add(new Rotate(180, Rotate.Z_AXIS));
//            subSceneGroup.getChildren().add(stoneView);
//            stones[i].setNode(stoneView, true);

            //Stones with Plain Box Geometry
            Node stoneView;
            if (i%3 == 0) {
                stoneView = Util3D.box(10, 20, 10, 1, 1, 1,
                        yellowMat, yellowMat, blackMat, yellowMat);
            } else {
                stoneView = new Box(10, 20, 10);
                ((Box)stoneView).setMaterial(yellowMat);
            }
            if (i>5) stoneView.getTransforms().add(new Rotate(180, Rotate.Z_AXIS));
            subSceneGroup.getChildren().add(stoneView);
            stones[i].setNode(stoneView, false);
            stones[i].addGeom(OdeHelper.createBox(10, 20, 10));

            stones[i].setPosition(i<6? -HALF_FIELD_WIDTH+125 : HALF_FIELD_WIDTH-125, -HALF_FIELD_WIDTH + 10 + (i%6)*22, 5);
            stones[i].setCategoryBits(CBits.STONES);
            stones[i].setCollideBits(0xFF);
        }

        foundationMass = OdeHelper.createMass();
        foundationMass.setBox(1, 46, 88, 6);
        foundationMass.setMass(2000);
        FoundationMeshCreator foundationMeshCreator = new FoundationMeshCreator();
        Mesh foundationMesh1 = foundationMeshCreator.newMeshInstance();
        MeshView foundationMeshView1 = new MeshView(foundationMesh1);
        foundationMeshView1.setMaterial(Util3D.imageMaterial("/virtual_robot/assets/blueFoundation.jpg"));
        Mesh foundationMesh2 = foundationMeshCreator.newMeshInstance();
        MeshView foundationMeshView2 = new MeshView(foundationMesh2);
        foundationMeshView2.setMaterial(Util3D.imageMaterial("/virtual_robot/assets/redFoundation.jpg"));
        subSceneGroup.getChildren().addAll(foundationMeshView1, foundationMeshView2);
        foundationBlue = FxBody.newInstance(world, space);
        foundationRed = FxBody.newInstance(world, space);
        foundationBlue.setMass(foundationMass);
        foundationRed.setMass(foundationMass);
        foundationBlue.setNode(foundationMeshView1, true);
        foundationRed.setNode(foundationMeshView2, true);
        foundationBlue.setPosition(-38, HALF_FIELD_WIDTH-54, 3);
        foundationRed.setPosition(38, HALF_FIELD_WIDTH-54, 3);
        foundationBlue.setCategoryBits(CBits.FOUNDATIONS);
        foundationRed.setCategoryBits(CBits.FOUNDATIONS);
        foundationBlue.setCollideBits(0xFF);
        foundationRed.setCollideBits(0xFF);
        foundationBlue.setDamping(0.05, 0.05);
        foundationRed.setDamping(0.05, 0.05);
    }

    @Override
    public void reset() {
        for (int i=0; i<12; i++){
            stones[i].setPosition(i<6? -HALF_FIELD_WIDTH+125 : HALF_FIELD_WIDTH-125, -HALF_FIELD_WIDTH + 10 + (i%6)*22, 5);
            DMatrix3 rot = new DMatrix3();
            DRotation.dRFromAxisAndAngle(rot, 0, 0, 1, 0);
            stones[i].setRotation(rot);
        }
        foundationBlue.setPosition(-38, HALF_FIELD_WIDTH-54, 3);
        DMatrix3 rotBlue = new DMatrix3();
        DRotation.dRFromAxisAndAngle(rotBlue, 0, 0, 1, 0);
        foundationBlue.setRotation(rotBlue);
        foundationRed.setPosition(38, HALF_FIELD_WIDTH-54, 3);
        DMatrix3 rotRed = new DMatrix3();
        DRotation.dRFromAxisAndAngle(rotRed, 0, 0, 1, 0);
        foundationRed.setRotation(rotRed);
    }

    @Override
    public void updateDisplay() {
        for (int i=0; i<stones.length; i++){
            stones[i].updateNodeDisplay();
        }
        foundationBlue.updateNodeDisplay();
        foundationRed.updateNodeDisplay();
    }

    @Override
    public void handleContacts(int numContacts, DGeom o1, DGeom o2, DContactBuffer contacts, DJointGroup contactGroup) {
        boolean o1Block = (o1.getCategoryBits() & CBits.STONES) != 0;
        boolean o2Block = (o2.getCategoryBits() & CBits.STONES) != 0;
        boolean bothBlocks = o1Block && o2Block;
        boolean o1Foundation = (o1.getCategoryBits() & CBits.FOUNDATIONS) != 0;
        boolean o2Foundation = (o2.getCategoryBits() & CBits.FOUNDATIONS) != 0;
        boolean bothFoundations = o1Foundation && o2Foundation;

        for (int i=0; i<numContacts; i++)
        {
            DContact contact = contacts.get(i);
            if (bothBlocks) {
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;      //Enable bounce
                contact.surface.bounce = 0.1;
                contact.surface.bounce_vel = 4.0;
                contact.surface.mu = 0.3;
                contact.surface.soft_cfm = 0.00002;
                contact.surface.soft_erp = 0.5;
            } else if (bothFoundations) {
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;      //Enable bounce
                contact.surface.mu = 0.5;
                contact.surface.soft_cfm = 0.0000000625;
                contact.surface.soft_erp = 0.5;
            } else{
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;      //Enable bounce
                contact.surface.bounce = 0.5;
                contact.surface.bounce_vel = 2.0;
                contact.surface.mu = 0.5;
                contact.surface.soft_cfm = 0;
                contact.surface.soft_erp = 0.4;
            }
            DJoint c = OdeHelper.createContactJoint(world, contactGroup, contact);
            c.attach(contact.geom.g1.getBody(), contact.geom.g2.getBody());
        }
    }

    @Override
    public void preStepProcess() {
        for (FxBody s: stones){
            DVector3C linVel = s.getLinearVel();
            DVector3C angVel = s.getAngularVel();
            s.addForce(linVel.reScale(-0.1*s.getMass().getMass()));
            s.addTorque(angVel.reScale(-0.1*s.getMass().getI().get22()));
        }
    }

    private class FoundationMeshCreator extends ClosedMeshCreator{

        public FoundationMeshCreator(){
            super(1, 1, 3);
        }

        @Override
        protected float[] createPoints() {
            return new float[]{
                    -22, -43, -3,
                    22, -43, -3,
                    -22, 43, -3,
                    22, 43, -3,

                    -23, -44, 3,
                    23, -44, 3,
                    23, 44, 3,
                    -23, 44, 3,

                    -22, -43, 3,
                    22, -43, 3,
                    22, 43, 3,
                    -22, 43, 3,

                    -21, -42, 0,
                    21, -42, 0,
                    -21, 42, 0,
                    21, 42, 0

            };
        }
    }

    // Stones created with this geometry don't stack well in the simulator, and don't interact well with the
    // fingers of the grabber, although they could still be used for visual effect (but still use a DBox
    // for the DGeom)

    private class StoneMeshCreator extends ClosedMeshCreator{

        public StoneMeshCreator(){ super(5, 9, 1); }

        @Override
        protected float[] createPoints(){
            return new float[]{

                    //Bottom
                    -5, -10, -5,   -4, -10, -5,   -3, -10, -5,   3, -10, -5,   4, -10, -5,   5, -10, -5,
                    -5, -9, -5,   -4, -9, -5,   -3, -9, -5,   3, -9, -5,   4, -9, -5,   5, -9, -5,
                    -5, -8, -5,   -4, -8, -5,   -3, -8, -2,   3, -8, -2,   4, -8, -5,   5, -8, -5,
                    -5, -2, -5,   -4, -2, -5,   -3, -2, -2,   3, -2, -2,   4, -2, -5,   5, -2, -5,
                    -5, -1, -5,   -4, -1, -5,   -3, -1, -5,   3, -1, -5,   4, -1, -5,   5, -1, -5,
                    -5, 1, -5,   -4, 1, -5,   -3, 1, -5,   3, 1, -5,   4, 1, -5,   5, 1, -5,
                    -5, 2, -5,   -4, 2, -5,   -3, 2, -2,   3, 2, -2,   4, 2, -5,   5, 2, -5,
                    -5, 8, -5,   -4, 8, -5,   -3, 8, -2,   3, 8, -2,   4, 8, -5,   5, 8, -5,
                    -5, 9, -5,   -4, 9, -5,   -3, 9, -5,   3, 9, -5,   4, 9, -5,   5, 9, -5,
                    -5, 10, -5,   -4, 10, -5,   -3, 10, -5,   3, 10, -5,   4, 10, -5,   5, 10, -5,

                    //Top
                    -5, -10, 5,   -3, -10, 5,   -2, -10, 5,   2, -10, 5,   3, -10, 5,   5, -10, 5,
                    -5, -8, 5,   -3, -8, 5,   -2, -8, 5,   2, -8, 5,   3, -8, 5,   5, -8, 5,
                    -5, -7, 5,   -3, -7, 5,   -2, -7, 7,   2, -7, 7,   3, -7, 5,   5, -7, 5,
                    -5, -3, 5,   -3, -3, 5,   -2, -3, 7,   2, -3, 7,   3, -3, 5,   5, -3, 5,
                    -5, -2, 5,   -3, -2, 5,   -2, -2, 5,   2, -2, 5,   3, -2, 5,   5, -2, 5,
                    -5, 2, 5,   -3, 2, 5,   -2, 2, 5,   2, 2, 5,   3, 2, 5,   5, 2, 5,
                    -5, 3, 5,   -3, 3, 5,   -2, 3, 7,   2, 3, 7,   3, 3, 5,   5, 3, 5,
                    -5, 7, 5,   -3, 7, 5,   -2, 7, 7,   2, 7, 7,   3, 7, 5,   5, 7, 5,
                    -5, 8, 5,   -3, 8, 5,   -2, 8, 5,   2, 8, 5,   3, 8, 5,   5, 8, 5,
                    -5, 10, 5,   -3, 10, 5,   -2, 10, 5,   2, 10, 5,   3, 10, 5,   5, 10, 5
            };
        }
    }

}
