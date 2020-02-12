package odefx;

import javafx.collections.ObservableList;
import javafx.geometry.Point3D;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.shape.*;
import javafx.scene.transform.MatrixType;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import org.ode4j.math.DMatrix3;
import org.ode4j.ode.*;
import util3d.Util3D;

import java.util.ArrayList;
import java.util.List;

public class FxBodyHelper {

    public static DTriMeshData dTriMeshDataFromTriangleMesh(TriangleMesh triangleMesh){
        DTriMeshData data = OdeHelper.createTriMeshData();
        int[] meshFaces = triangleMesh.getFaces().toArray(null);
        int[] meshIndices = new int[meshFaces.length / 2];
        for (int i=0; i<meshIndices.length; i++) meshIndices[i] = meshFaces[2*i];
        data.build( triangleMesh.getPoints().toArray(null), meshIndices);
        return data;
    }

    public static DTriMeshData getParametricTriMeshData(double sMin, double sMax, double tMin, double tMax,
                                                        int numFacetS, int numFacetT, boolean sClosed, boolean tClosed,
                                                        Util3D.Param3DEqn equations){


        int numPtsS = sClosed? numFacetS : numFacetS + 1;
        int numPtsT = tClosed? numFacetT : numFacetT + 1;

        float dS = (float)(sMax - sMin) / numFacetS;
        float dT = (float)(tMax - tMin) / numFacetT;

        float[] vertices = new float[numPtsS * numPtsT * 3];

        for (int i=0; i<numPtsT; i++){
            for (int j=0; j<numPtsS; j++){
                float s = (float)sMin + j * dS;
                float t = (float)tMin + i * dT;
                float x = equations.x(s, t);
                float y = equations.y(s, t);
                float z = equations.z(s, t);
                vertices[3*(numPtsS*i + j)] = x;
                vertices[3*(numPtsS*i + j) + 1] = y;
                vertices[3*(numPtsS*i + j) + 2] = z;
            }
        }

        int[] indices = new int[6 * numFacetS * numFacetT];

        for (int i=0; i<numFacetT; i++){
            int i2 = tClosed && i==(numFacetT-1)? 0 : i+1;
            for (int j=0; j<numFacetS; j++){
                int j2 = sClosed && j==(numFacetS-1)? 0 : j+1;
                int facetBaseIndex = 6 * (numFacetS*i + j);
                indices[facetBaseIndex]  = numPtsS*i + j;
                indices[facetBaseIndex+1]  = numPtsS*i + j2;
                indices[facetBaseIndex+2] = numPtsS*i2 + j2;
                indices[facetBaseIndex+3] = numPtsS*i2 + j2;
                indices[facetBaseIndex+4] = numPtsS*i2 + j;
                indices[facetBaseIndex+5] = numPtsS*i + j;
            }
        }

        DTriMeshData data = OdeHelper.createTriMeshData();
        data.build(vertices, indices);

        return data;

    }

    private static DBox dBoxFromBox(Box box, DSpace space) {
        DBox dBox = OdeHelper.createBox(box.getWidth(), box.getHeight(), box.getDepth());
        if (space != null) space.add(dBox);
        return dBox;
    }

    private static DSphere dSphereFromSphere(Sphere sphere, DSpace space){
        DSphere dSphere = OdeHelper.createSphere(sphere.getRadius());
        if (space != null) space.add(dSphere);
        return dSphere;
    }

    private static DCylinder dCylinderFromCylinder(Cylinder cylinder, DSpace space){
        DCylinder dCylinder = OdeHelper.createCylinder(cylinder.getRadius(), cylinder.getHeight());
        if (space != null) space.add(dCylinder);
        return dCylinder;
    }

    public static DTriMesh dTriMeshFromMeshView(MeshView meshView, DSpace space){
        TriangleMesh triangleMesh = (TriangleMesh)meshView.getMesh();
        DTriMeshData data = dTriMeshDataFromTriangleMesh(triangleMesh);
        DTriMesh dTriMesh = OdeHelper.createTriMesh(space, data, null, null, null);
        return dTriMesh;
    }

    private static DGeom dGeomFromShape3D(Shape3D shape, DSpace space){
        DGeom dGeom;
        if (shape instanceof Box) dGeom = dBoxFromBox((Box)shape, space);
        else if (shape instanceof Sphere) dGeom = dSphereFromSphere((Sphere)shape, space);
        else if (shape instanceof Cylinder) dGeom = dCylinderFromCylinder((Cylinder)shape, space);
        else if (shape instanceof DTriMesh) dGeom = dTriMeshFromMeshView((MeshView)shape, space);
        else return null;
        return dGeom;
    }

    private static List<DGeom> dGeomsFromNode(Node node, Transform preTransform, DSpace space, DBody dBody){

        ArrayList<DGeom> list = new ArrayList<DGeom>();

        Transform transform = preTransform == null? new Translate(0, 0, 0): preTransform.clone();
        ObservableList<Transform> transforms = node.getTransforms();
        for (int i=0; i<transforms.size(); i++) transform = transform.createConcatenation(transforms.get(i));

        if (node instanceof Group){
            for (Node n: ((Group)node).getChildren()){
                List<DGeom> subList = dGeomsFromNode(n, transform, space, dBody);
                for (DGeom dGeom: subList) list.add(dGeom);
            }
        } else if (node instanceof Shape3D){
            DGeom dGeom = dGeomFromShape3D((Shape3D)node, space);
            if (node instanceof Cylinder) {
                transform = transform.createConcatenation(new Rotate(90, new Point3D(1, 0, 0)));
            }
            double[] tData = transform.toArray(MatrixType.MT_3D_3x4);
            if (dBody != null) {
                dGeom.setBody(dBody);
                dGeom.setOffsetPosition(tData[3], tData[7], tData[11]);
                DMatrix3 dRotMatrix = new DMatrix3(tData[0], tData[1], tData[2], tData[4], tData[5], tData[6],
                        tData[8], tData[9], tData[10]);
                dGeom.setOffsetRotation(dRotMatrix);
            } else {
                dGeom.setPosition(tData[3], tData[7], tData[11]);
                DMatrix3 dRotMatrix = new DMatrix3(tData[0], tData[1], tData[2], tData[4], tData[5], tData[6],
                        tData[8], tData[9], tData[10]);
                dGeom.setRotation(dRotMatrix);
            }
        }

        return list;
    }

    public static List<DGeom> dGeomsFromNode(Node node, DSpace space, DBody dBody){
        return dGeomsFromNode(node, null, space, dBody);
    }

}
