package odefx;

import javafx.collections.ObservableList;
import javafx.geometry.Point3D;
import javafx.scene.Node;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import org.ode4j.math.*;
import org.ode4j.ode.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

/**
 * Attempting to encapsulate an ODE4J DGeom (possibly attached to a DBody) along with a JavaFX Shape3D or Group,
 * with convenience methods to create such objects and to update display.
 */
public class FxBody {

    private DBody dBody = null;

    private Node node = null;

    private DSpace dSpace = null;

    private HashMap<String, DGeom> geoms = new HashMap<>();

    /**
     * The purpose of children is to allow a group of bodies to be repositioned together, maintaining their
     * original spatial relationships. The only methods that use the children of the FxBody are
     * getChildren(), setPosition(), setRotation(), setQuaternion(), and updateNodeDisplay().
     */
    private List<FxBody> children = new ArrayList<>();

    public DBody getBody(){
        return dBody;
    }

    /**
     * Create new instance of FxBody with new DBody
     * @param world DBody will be placed in this world
     * @return
     */
    public static FxBody newInstance(DWorld world){
        DBody dBody = OdeHelper.createBody(world);
        return new FxBody(dBody);
    }

    /**
     * Create new instance of FxBody with new DBody
     * @param world DBody will be placed in this world
     * @param space DGeoms created/added to this FxBody will be placed in this botSpace
     * @return
     */
    public static FxBody newInstance(DWorld world, DSpace space){
        DBody dBody = OdeHelper.createBody(world);
        return new FxBody(dBody, space);
    };

    public List<FxBody> getChildren() { return children;}

    /**
     * Get the JavaFx Node (should be either a Group or Shape3D) associated with this FxBody
     * @return
     */
    public Node getNode() {
        return node;
    }

    /**
     * Get the DSpace associated with this FxBody.
     * @return
     */
    public DSpace getSpace() {
        return dSpace;
    }

    /**
     * Set the DSpace associated with this FxBody, and add all DGeoms to the DSpace
     * @param dSpace
     */
    public void setSpace(DSpace dSpace) {
        this.dSpace = dSpace;
        DGeom dGeom = dBody.getFirstGeom();
        while (dGeom != null){
            if (dSpace == null) {
                DSpace s = dGeom.getSpace();
                if (s != null) s.remove(dGeom);
            } else {
                dSpace.add(dGeom);
            }
            dGeom = dBody.getNextGeom(dGeom);
        }
    }

    /**
     * Private constructor -- use newInstance
     * @param body
     */
    private FxBody(DBody body){
        dBody = body;
    }

    /**
     * Private constructor -- use newInstance
     * @param body
     * @param space
     */
    private FxBody(DBody body, DSpace space){
        dBody = body;
        dSpace = space;
    }

    /**
     * Set the mass of the DBody
     * @param mass
     */
    public void setMass(DMass mass){
        dBody.setMass(mass);
    }

    /**
     * Add a DGeom to the DBody, and also to the botSpace
     * @param dGeom
     */
    public void addGeom(DGeom dGeom){
        dGeom.setBody(dBody);
        if (dSpace != null && dGeom.getSpace() == null) dSpace.add(dGeom);
        if (dGeom.getData() != null && dGeom.getData() instanceof String) geoms.put((String)dGeom.getData(), dGeom);
    }

    public void addGeom(DGeom dGeom, double x, double y, double z){
        addGeom(dGeom);
        dGeom.setOffsetPosition(x, y, z);
    }

    public Iterable<DGeom> getGeoms(){
        return new Iterable<DGeom>() {
            @Override
            public Iterator<DGeom> iterator() {
                return new Iterator<DGeom>() {
                    DGeom current = null;

                    @Override
                    public boolean hasNext() {
                        if (current == null) return dBody.getFirstGeom() != null;
                        else return dBody.getNextGeom(current) != null;
                    }

                    @Override
                    public DGeom next() {
                        current = current == null? dBody.getFirstGeom() : dBody.getNextGeom(current);
                        return current;
                    }
                };
            }
        };
    }

    /**
     * Add a DGeom to the DBody, and to the botSpace, applying the requested offset position and rotation
     * @param dGeom
     * @param x   x offset
     * @param y   y offset
     * @param z   z offset
     * @param R   Rotation offset
     */
    public void addGeom(DGeom dGeom, double x, double y, double z, DMatrix3C R){
        addGeom(dGeom);
        dGeom.setOffsetPosition(x, y, z);
        dGeom.setOffsetRotation(R);
    }

    public void addGeom(DGeom dGeom, double x, double y, double z, DQuaternion q){
        addGeom(dGeom);
        dGeom.setOffsetPosition(x, y, z);
        dGeom.setOffsetQuaternion(q);
    }

    /**
     * Return dGeom whose "key" is name. Note not all geoms will necessarily have a key.
     * @param name
     * @return
     */
    public DGeom getGeom(String name){
        return geoms.get(name);
    }

    /**
     * Set category bits to the same value for all geoms belonging to this FxBody
     * @param bits
     */
    public void setCategoryBits(long bits){
        for (DGeom g: getGeoms()){
            g.setCategoryBits(bits);
        }
    }

    public void destroy(boolean destroyChildren){
        //First, destroy all of the dgeoms
        List<DGeom> dGeoms = new ArrayList<>();
        for (DGeom g: getGeoms()) dGeoms.add(g);
        for (int i=0; i<dGeoms.size(); i++){
            DGeom g = dGeoms.get(i);
            if (g.getSpace() != null) g.getSpace().remove(g);
            g.destroy();
        }

        //Next, destroy all of the joints
        List<DJoint> joints = new ArrayList<>();
        int numJoints = dBody.getNumJoints();
        for (int i=0; i<numJoints; i++){
            dBody.getJoint(i).destroy();
        }

        //Next, destroy all of the children (if requested)
        if (destroyChildren) {
            for (int i = 0; i < children.size(); i++) children.get(i).destroy(true);
        }

        //Finally, destroy the DBody
        dBody.destroy();
    }

    /**
     * Set collide bits to the same value for all geoms belonging to this FxBody
     * @param bits
     */
    public void setCollideBits(long bits){
        for (DGeom g: getGeoms()){
            g.setCollideBits(bits);
        }
    }


    /**
     * Set category bits for a single geom. If there is no geom with the provided ID, an exception will result
     * @param geomId
     * @param bits
     */
    public void setCategoryBits(String geomId, long bits){
        geoms.get(geomId).setCategoryBits(bits);
    }

    /**
     * Set collide bits for a single geom. If there is no geom with the provided ID, an exception will result
     * @param geomId
     * @param bits
     */
    public void setCollideBits(String geomId, long bits){
        geoms.get(geomId).setCollideBits(bits);
    }


    /**
     * Set the JavaFX node associated with this FxBody and optionally generate a list of DGeoms corresponding to
     * the JavaFX node, and add them to the DBody associated with this FxBody. If the node has transforms,
     * they will be used to generate appropriate Offsets for the generated DGeoms. setNode() will automatically
     * add a Translate transform and a Rotate transform to the beginning of the Transforms property of the
     * node. Those transforms will be used to update the display as the dBody object moves.
     *
     * @param node  The JavaFX Node Object -- should be Group or Shape3D.
     * @param generateGeoms If true, generate DGeom objects and associate them with the DBody.
     */
    public void setNode(Node node, boolean generateGeoms) {
        if (generateGeoms) {
            FxBodyHelper.dGeomsFromNode(node, dSpace, dBody);
        }
        this.node = node;
        this.node.getTransforms().add(0, new Rotate(0));
        this.node.getTransforms().add(0, new Translate(0,0,0));
    }

    /**
     * Update the display of the Node, based on current position and orientation of the DBody.
     *
     * This method should only be called from the Application Thread. This can be accomplished by
     * wrapping the call in a call to Platform.runLater.
     */
    public void updateNodeDisplay(boolean updateChildren){
        if (node == null) return;
        ObservableList<Transform> transforms = node.getTransforms();
        if (transforms.size() < 2) return;
        DVector3C pos;
        DQuaternionC quat;
        if (dBody != null) {
            pos = dBody.getPosition();
            quat = dBody.getQuaternion();
        } else return;
        DVector3 axis = new DVector3(quat.get(1), quat.get(2), quat.get(3));
        double sinThetaOver2 = axis.length();
        double angle = 2.0 * Math.atan2(sinThetaOver2, quat.get(0)) * 180.0 / Math.PI;
        //Normalize the axes[i] to a length of 1 (but not if the angle is zero). If the angle
        //is zero, the axes[i] length will also be zero and this would crash. May not need to
        //normalize at all.
        //if (sinThetaOver2 != 0) axis.normalize();
        ((Rotate)transforms.get(1)).setAxis(new Point3D(axis.get0(), axis.get1(), axis.get2()));
        ((Rotate)transforms.get(1)).setAngle(angle);
        ((Translate)transforms.get(0)).setX(pos.get0());
        ((Translate)transforms.get(0)).setY(pos.get1());
        ((Translate)transforms.get(0)).setZ(pos.get2());

        if (updateChildren){
            for (FxBody fbChild: children){
                fbChild.updateNodeDisplay(true);
            }
        }
    }

    public void updateNodeDisplay(){
        updateNodeDisplay(false);
    }

    /**
     * Set position of the DBody, and automatically update display. Call only from the Application thread.
     * @param x
     * @param y
     * @param z
     */
    public void setPosition(double x, double y, double z, boolean setChildPos) {
        DVector3C oldPos = getPosition().clone();
        dBody.setPosition(x, y, z);
        if (setChildPos) {
            for (FxBody child : children) {
                child.setPosition(((DVector3)child.getPosition()).reAdd(getPosition()).reSub(oldPos), true);
            }
        }
        updateNodeDisplay();
    }

    public void setPosition(double x, double y, double z) { setPosition(x, y, z, true); }


    /**
     * Set the position of the DBody, and automatically update display. Call only from the application thread.
     * @param p
     */
    public void setPosition(DVector3C p, boolean setChildPos) {
        DVector3C oldPos = getPosition().clone();
        dBody.setPosition(p);
        if (setChildPos) {
            for (FxBody child : children) {
                child.setPosition(((DVector3)child.getPosition()).reAdd(getPosition()).reSub(oldPos), true);
            }
        }
        updateNodeDisplay();
    }

    public void setPosition(DVector3C p) { setPosition(p, true); }


    /**
     * Set the Rotation of the DBody, and automatically update display. Call only from the application thread.
     * @param R
     */
    public void setRotation(DMatrix3C R, boolean setChildRot) {
        DMatrix3C oldRot = getRotation().clone();
        dBody.setRotation(R);
        if (setChildRot){
            DMatrix3 invOldRot = new DMatrix3();
            DMatrix.dInvertPDMatrix(oldRot, invOldRot);
            for (FxBody child: children){
                DMatrix3 newRotTimesInvOldRot = new DMatrix3();
                DMatrix.dMultiply0(newRotTimesInvOldRot, R, invOldRot);
                DMatrix3 newChildRot = new DMatrix3();
                DMatrix.dMultiply0(newChildRot, newRotTimesInvOldRot, child.getRotation());
                DVector3 newChildPos = new DVector3();
                DMatrix.dMultiply0(newChildPos, newRotTimesInvOldRot, child.getPosition().reSub(getPosition()));
                newChildPos.add(getPosition());
                child.setPosition(newChildPos, true);
                child.setRotation(newChildRot, true);
            }
        }
        updateNodeDisplay();
    }

    public void setRotation(DMatrix3C R) { setRotation(R, true);}


    /**
     * Set the Quaternion of the DBody, and automatically update display. Call only from the application thread.
     * @param q
     */
    public void setQuaternion(DQuaternionC q, boolean setChildRot) {
        DMatrix3C oldRot = getRotation().clone();
        dBody.setQuaternion(q);
        if (setChildRot){
            DMatrix3 invOldRot = new DMatrix3();
            DMatrix.dInvertPDMatrix(oldRot, invOldRot);
            for (FxBody child: children){
                DMatrix3 newRot = new DMatrix3();
                DRotation.dRfromQ(newRot, q);
                DMatrix3 RotNewTimesInvOldRot = new DMatrix3();
                DMatrix.dMultiply0(RotNewTimesInvOldRot, newRot, invOldRot);
                DMatrix3 newChildRot = new DMatrix3();
                DMatrix.dMultiply0(newChildRot, RotNewTimesInvOldRot, child.getRotation());
                DVector3 newChildPos = new DVector3();
                DMatrix.dMultiply0(newChildPos, RotNewTimesInvOldRot, child.getPosition().reSub(getPosition()));
                newChildPos.add(getPosition());
                child.setPosition(newChildPos, true);
                child.setRotation(newChildRot, true);
            }
        }
        updateNodeDisplay();
    }

    public void setQuaternion(DQuaternionC q) { setQuaternion(q, true);}

    public void setData(Object data) {
        dBody.setData(data);
    }

    public Object getData() {
        return dBody.getData();
    }


    public void setLinearVel(double x, double y, double z) {
        dBody.setLinearVel(x, y, z);
    }


    public void setLinearVel(DVector3C v) {
        dBody.setLinearVel(v);
    }


    public void setAngularVel(double x, double y, double z) {
        dBody.setAngularVel(x, y, z);
    }


    public void setAngularVel(DVector3C v) {
        dBody.setAngularVel(v);
    }


    public DVector3C getPosition() {
        return dBody.getPosition();
    }


    public DMatrix3C getRotation() {
        return dBody.getRotation();
    }


    public DQuaternionC getQuaternion() {
        return dBody.getQuaternion();
    }


    public DVector3C getLinearVel() {
        return dBody.getLinearVel();
    }


    public DVector3C getAngularVel() {
        return dBody.getAngularVel();
    }


    public void setMass(DMassC mass) {
        dBody.setMass(mass);
    }


    public DMassC getMass() {
        return dBody.getMass();
    }


    public DWorld getWorld() {
        return dBody.getWorld();
    }


    public void setAutoDisableLinearThreshold(double threshold) {
        dBody.setAutoDisableLinearThreshold(threshold);
    }


    public double getAutoDisableLinearThreshold() {
        return dBody.getAutoDisableLinearThreshold();
    }


    public void setAutoDisableAngularThreshold(double threshold) {
        dBody.setAutoDisableAngularThreshold(threshold);
    }


    public double getAutoDisableAngularThreshold() {
        return dBody.getAutoDisableAngularThreshold();
    }


    public void setAutoDisableSteps(int steps) {
        dBody.setAutoDisableSteps(steps);
    }


    public int getAutoDisableSteps() {
        return dBody.getAutoDisableSteps();
    }


    public void setAutoDisableTime(double time) {
        dBody.setAutoDisableTime(time);
    }


    public double getAutoDisableTime() {
        return dBody.getAutoDisableTime();
    }


    public void setAutoDisableFlag(boolean do_auto_disable) {
        dBody.setAutoDisableFlag(do_auto_disable);
    }


    public boolean getAutoDisableFlag() {
        return dBody.getAutoDisableFlag();
    }


    public int getAutoDisableAverageSamplesCount() {
        return dBody.getAutoDisableAverageSamplesCount();
    }


    public void setAutoDisableAverageSamplesCount(int average_samples_count) {
        dBody.setAutoDisableAverageSamplesCount(average_samples_count);
    }


    public void setAutoDisableDefaults() {
        dBody.setAutoDisableDefaults();
    }


    public void addForce(double fx, double fy, double fz) {
        dBody.addForce(fx, fy, fz);
    }


    public void addForce(DVector3C f) {
        dBody.addForce(f);
    }


    public void addTorque(double fx, double fy, double fz) {
        dBody.addTorque(fx, fy, fz);
    }


    public void addTorque(DVector3C t) {
        dBody.addTorque(t);
    }


    public void addRelForce(double fx, double fy, double fz) {
        dBody.addRelForce(fx, fy, fz);
    }


    public void addRelForce(DVector3C f) {
        dBody.addRelForce(f);
    }


    public void addRelTorque(double fx, double fy, double fz) {
        dBody.addRelTorque(fx, fy, fz);
    }


    public void addRelTorque(DVector3C t) {
        dBody.addRelTorque(t);
    }


    public void addForceAtPos(double fx, double fy, double fz, double px, double py, double pz) {
        dBody.addForceAtPos(fx, fy, fz, px, py, pz);
    }


    public void addForceAtPos(DVector3C f, DVector3C p) {
        dBody.addForceAtPos(f, p);
    }


    public void addForceAtRelPos(double fx, double fy, double fz, double px, double py, double pz) {
        dBody.addForceAtRelPos(fx, fy, fz, px, py, pz);
    }


    public void addForceAtRelPos(DVector3C f, DVector3C p) {
        dBody.addForceAtRelPos(f, p);
    }


    public void addRelForceAtPos(double fx, double fy, double fz, double px, double py, double pz) {
        dBody.addRelForceAtPos(fx, fy, fz, px, py, pz);
    }


    public void addRelForceAtPos(DVector3C f, DVector3C p) {
        dBody.addRelForceAtPos(f, p);
    }


    public void addRelForceAtRelPos(double fx, double fy, double fz, double px, double py, double pz) {
        dBody.addRelForceAtRelPos(fx, fy, fz, px, py, pz);
    }


    public void addRelForceAtRelPos(DVector3C f, DVector3C p) {
        dBody.addRelForceAtRelPos(f, p);
    }


    public DVector3C getForce() {
        return dBody.getForce();
    }


    public DVector3C getTorque() {
        return dBody.getTorque();
    }


    public void setForce(double x, double y, double z) {
        dBody.setForce(x, y, z);
    }


    public void setForce(DVector3C f) {
        dBody.setForce(f);
    }


    public void setTorque(double x, double y, double z) {
        dBody.setTorque(x, y, z);
    }


    public void setTorque(DVector3C t) {
        dBody.setTorque(t);
    }


    public void getRelPointPos(double px, double py, double pz, DVector3 result) {
        dBody.getRelPointPos(px, py, pz, result);
    }


    public void getRelPointPos(DVector3C p, DVector3 result) {
        dBody.getRelPointPos(p, result);
    }


    public void getRelPointVel(double px, double py, double pz, DVector3 result) {
        dBody.getRelPointVel(px, py, pz, result);
    }


    public void getRelPointVel(DVector3C p, DVector3 result) {
        dBody.getRelPointVel(p, result);
    }


    public void getPointVel(double px, double py, double pz, DVector3 result) {
        dBody.getPointVel(px, py, pz, result);
    }


    public void getPointVel(DVector3C p, DVector3 result) {
        dBody.getPointVel(p, result);
    }


    public void getPosRelPoint(double px, double py, double pz, DVector3 result) {
        dBody.getPointVel(px, py, pz, result);
    }


    public void getPosRelPoint(DVector3C p, DVector3 result) {
        dBody.getPosRelPoint(p, result);
    }


    public void vectorToWorld(double px, double py, double pz, DVector3 result) {
        dBody.vectorToWorld(px, py, pz, result);
    }


    public void vectorToWorld(DVector3C p, DVector3 result) {
        dBody.vectorToWorld(p, result);
    }


    public void vectorFromWorld(double px, double py, double pz, DVector3 result) {
        dBody.vectorFromWorld(px, py, pz, result);
    }


    public void vectorFromWorld(DVector3C p, DVector3 result) {
        dBody.vectorFromWorld(p, result);
    }


    public void setFiniteRotationMode(boolean mode) {
        dBody.setFiniteRotationMode(mode);
    }


    public void setFiniteRotationAxis(double x, double y, double z) {
        dBody.setFiniteRotationAxis(x, y, z);
    }


    public void setFiniteRotationAxis(DVector3C a) {
        dBody.setFiniteRotationAxis(a);
    }


    public boolean getFiniteRotationMode() {
        return dBody.getFiniteRotationMode();
    }


    public void getFiniteRotationAxis(DVector3 result) {
        dBody.getFiniteRotationAxis(result);
    }


    public int getNumJoints() {
        return dBody.getNumJoints();
    }


    public DJoint getJoint(int index) {
        return dBody.getJoint(index);
    }


    public void setDynamic() {
        dBody.setDynamic();
    }


    public void setKinematic() {
        dBody.setKinematic();
    }


    public boolean isKinematic() {
        return dBody.isKinematic();
    }


    public void enable() {
        dBody.enable();
    }


    public void disable() {
        dBody.disable();
    }


    public boolean isEnabled() {
        return dBody.isEnabled();
    }


    public void setGravityMode(boolean mode) {
        dBody.setGravityMode(mode);
    }


    public boolean getGravityMode() {
        return dBody.getGravityMode();
    }


    public boolean isConnectedTo(DBody body) {
        return dBody.isConnectedTo(body);
    }


    public double getLinearDamping() {
        return dBody.getLinearDamping();
    }


    public void setLinearDamping(double scale) {
        dBody.setLinearDamping(scale);
    }


    public double getAngularDamping() {
        return dBody.getAngularDamping();
    }


    public void setAngularDamping(double scale) {
        dBody.setAngularDamping(scale);
    }


    public void setDamping(double linear_scale, double angular_scale) {
        dBody.setDamping(linear_scale, angular_scale);
    }


    public double getLinearDampingThreshold() {
        return dBody.getLinearDampingThreshold();
    }


    public void setLinearDampingThreshold(double threshold) {
        dBody.setLinearDampingThreshold(threshold);
    }


    public double getAngularDampingThreshold() {
        return dBody.getAngularDampingThreshold();
    }


    public void setAngularDampingThreshold(double threshold) {
        dBody.setAngularDampingThreshold(threshold);
    }


    public void setDampingDefaults() {
        dBody.setDampingDefaults();
    }


    public double getMaxAngularSpeed() {
        return dBody.getMaxAngularSpeed();
    }


    public void setMaxAngularSpeed(double max_speed) {
        dBody.setMaxAngularSpeed(max_speed);
    }


    public boolean getGyroscopicMode() {
        return dBody.getGyroscopicMode();
    }


    public void setGyroscopicMode(boolean enabled) {
        dBody.setGyroscopicMode(enabled);
    }


    public void setMovedCallback(DBody.BodyMoveCallBack callback) {
        dBody.setMovedCallback(callback);
    }


    public DGeom getFirstGeom() {
        return dBody.getFirstGeom();
    }


    public DGeom getNextGeom(DGeom geom) {
        return dBody.getNextGeom(geom);
    }
}
