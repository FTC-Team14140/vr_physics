package odefx.node_with_geom;

import javafx.collections.ObservableList;
import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.shape.Cylinder;
import javafx.scene.transform.MatrixType;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.DGeom;

public class GroupWithDGeoms extends Group {

    public GroupWithDGeoms(){
        super();
    }

    public GroupWithDGeoms(String id){
        super();
        this.setId(id);
    }

    public void updateGeomOffsets(Transform preTransform){

        Transform transform = preTransform == null? new Translate(0, 0, 0) : preTransform.clone();
        ObservableList<Transform> transforms = this.getTransforms();
        for (int i=0; i<transforms.size(); i++) transform = transform.createConcatenation(transforms.get(i));

        for (Node node: getChildren()){
            if ( !(node instanceof GroupWithDGeoms) && !(node instanceof Shape3DWithDGeom)) continue;

            if (node instanceof GroupWithDGeoms){
                ((GroupWithDGeoms)node).updateGeomOffsets(transform);
            } else {
                Transform nodeTransform = transform.clone();
                ObservableList<Transform> nodeTransforms = node.getTransforms();
                for (int i=0; i<nodeTransforms.size(); i++) nodeTransform = nodeTransform.createConcatenation(nodeTransforms.get(i));
                Rotate nodeRelGeomRotation = ((Shape3DWithDGeom)node).getRelGeomRotation();
                Translate nodeRelGeomOffset = ((Shape3DWithDGeom)node).getRelGeomOffset();
                if (nodeRelGeomOffset != null) nodeTransform = nodeTransform.createConcatenation(nodeRelGeomOffset);
                if (nodeRelGeomRotation != null) nodeTransform = nodeTransform.createConcatenation(nodeRelGeomRotation);
                DGeom dGeom = ((Shape3DWithDGeom)node).getDGeom();
                if (node instanceof Cylinder) {
                    nodeTransform = nodeTransform.createConcatenation(new Rotate(90, new Point3D(1, 0, 0)));
                }
                double[] tData = nodeTransform.toArray(MatrixType.MT_3D_3x4);
                dGeom.setOffsetPosition(tData[3], tData[7], tData[11]);
                DMatrix3 dRotMatrix = new DMatrix3(tData[0], tData[1], tData[2], tData[4], tData[5], tData[6],
                        tData[8], tData[9], tData[10]);
                dGeom.setOffsetRotation(dRotMatrix);
            }

        }
    }


    public void updateGeomOffsets(){
        updateGeomOffsets(null);
    }

}
