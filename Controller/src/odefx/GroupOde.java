package odefx;

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

public class GroupOde extends Group {

    public GroupOde(){
        super();
    }

    public GroupOde(String id){
        super();
        this.setId(id);
    }

    public void updateGeomOffsets(Transform preTransform){

        Transform transform = preTransform == null? new Translate(0, 0, 0) : preTransform.clone();
        ObservableList<Transform> transforms = this.getTransforms();
        for (int i=0; i<transforms.size(); i++) transform = transform.createConcatenation(transforms.get(i));

        for (Node node: getChildren()){
            if ( !(node instanceof GroupOde) && !(node instanceof HasDGeom)) continue;

            if (node instanceof GroupOde){
                ((GroupOde)node).updateGeomOffsets(transform);
            } else {
                Transform nodeTransform = transform.clone();
                ObservableList<Transform> nodeTransforms = node.getTransforms();
                for (int i=0; i<nodeTransforms.size(); i++) nodeTransform = nodeTransform.createConcatenation(nodeTransforms.get(i));

                DGeom dGeom = ((HasDGeom)node).getDGeom();
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

    public void printGeoms(){

        if (this.getId() != null && !this.getId().isEmpty()){
            System.out.println(this.getId());
        } else {
            System.out.println("GroupOde: NO ID");
        }

        System.out.println();

        for (Node n: this.getChildren()){
            if (n instanceof GroupOde){
                ((GroupOde) n).printGeoms();
            } else if (n instanceof HasDGeom){
                if (n.getId() != null && !n.getId().isEmpty()) System.out.print(n.getId() + ": " + n.getClass().getName() + " " + ((HasDGeom)n).getDGeom().getAABB().toString());
                else System.out.print("No Id: " + n.getClass().getName() + " " + ((HasDGeom)n).getDGeom().getAABB().toString());
                System.out.println();
            }
        }
    }

    public void updateGeomOffsets(){
        updateGeomOffsets(null);
    }

}
