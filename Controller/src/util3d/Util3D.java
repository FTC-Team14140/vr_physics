package util3d;

import javafx.scene.Group;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;

public class Util3D {

    public interface Param3DEqn{
        float x(float s, float t);
        float y(float s, float t);
        float z(float s, float t);
    }

    public static TriangleMesh getParametricMesh(double sMin, double sMax, double tMin, double tMax,
                                            int numFacetS, int numFacetT, boolean sClosed, boolean tClosed,
                                            Param3DEqn equations){

        int numPtsS = sClosed? numFacetS : numFacetS + 1;
        int numPtsT = tClosed? numFacetT : numFacetT + 1;

        float dS = (float)(sMax - sMin) / numFacetS;
        float dT = (float)(tMax - tMin) / numFacetT;

        float dU = 1.0f / numFacetS;
        float dV = 1.0f / numFacetT;

        float[] points = new float[numPtsS * numPtsT * 3];
        float[] texCoords = new float[numPtsS * numPtsT * 2];

        for (int i=0; i<numPtsT; i++){
            for (int j=0; j<numPtsS; j++){
                float s = (float)sMin + j * dS;
                float t = (float)tMin + i * dT;
                float x = equations.x(s, t);
                float y = equations.y(s, t);
                float z = equations.z(s, t);
                float u = j * dU;
                float v = 1.0f - i * dV;
                points[3*(numPtsS*i + j)] = x;
                points[3*(numPtsS*i + j) + 1] = y;
                points[3*(numPtsS*i + j) + 2] = z;
                texCoords[2*(numPtsS*i + j)] = u;
                texCoords[2*(numPtsS*i + j) + 1] = v;
            }
        }

        int[] facets = new int[12 * numFacetS * numFacetT];

        for (int i=0; i<numFacetT; i++){
            int i2 = tClosed && i==(numFacetT-1)? 0 : i+1;
            for (int j=0; j<numFacetS; j++){
                int j2 = sClosed && j==(numFacetS-1)? 0 : j+1;
                int facetBaseIndex = 12 * (numFacetS*i + j);
                facets[facetBaseIndex] = facets[facetBaseIndex+1] = numPtsS*i + j;
                facets[facetBaseIndex+2] = facets[facetBaseIndex+3] = numPtsS*i + j2;
                facets[facetBaseIndex+4] = facets[facetBaseIndex+5] = numPtsS*i2 + j2;
                facets[facetBaseIndex+6] = facets[facetBaseIndex+7] = numPtsS*i2 + j2;
                facets[facetBaseIndex+8] = facets[facetBaseIndex+9] = numPtsS*i2 + j;
                facets[facetBaseIndex+10] = facets[facetBaseIndex+11] = numPtsS*i + j;
            }
        }

        TriangleMesh mesh = new TriangleMesh(VertexFormat.POINT_TEXCOORD);

        mesh.getPoints().addAll(points);
        mesh.getTexCoords().addAll(texCoords);
        mesh.getFaces().addAll(facets);

        return mesh;

    }


    public static TriangleMesh getParametricMesh(double sMin, double sMax, double tMin, double tMax,
                                                 int numFacetS, int numFacetT, Param3DEqn equations){
        return getParametricMesh(sMin, sMax, tMin, tMax, numFacetS, numFacetT, false, false, equations);
    }


    public static Group cylinder(float radius, float height, int circumferenceFacets, int heightFacets,
                                 int radiusFacets, boolean endCaps, PhongMaterial... materials){
        Group group = new Group();

        TriangleMesh tubeMesh = getParametricMesh(0, 2.0 * Math.PI, -height / 2.0, height / 2.0, circumferenceFacets,
                heightFacets, new Param3DEqn() {
                    public float x(float s, float t) {
                        return radius * (float) Math.cos(s);
                    }

                    public float y(float s, float t) {
                        return -t;
                    }

                    public float z(float s, float t) {
                        return radius * (float) Math.sin(s);
                    }
                });

        MeshView tubeMeshView = new MeshView(tubeMesh);

        if (materials != null && materials.length >0) tubeMeshView.setMaterial(materials[0]);

        group.getChildren().add(tubeMeshView);

        if (!endCaps) return group;

        TriangleMesh capMesh1 = getParametricMesh(0, 2.0 * Math.PI, 0, radius, circumferenceFacets,
                radiusFacets, new Param3DEqn() {
                    public float x(float s, float t) {
                        return t * (float)Math.sin(s);
                    }

                    public float y(float s, float t) {
                        return -height/2.0f;
                    }

                    public float z(float s, float t) {
                        return t * (float)Math.cos(s);
                    }
                });

        TriangleMesh capMesh2 = getParametricMesh(0, 2.0 * Math.PI, radius, 0, circumferenceFacets,
                radiusFacets, new Param3DEqn() {
                    public float x(float s, float t) {
                        return t * (float)Math.sin(s);
                    }

                    public float y(float s, float t) {
                        return height/2.0f;
                    }

                    public float z(float s, float t) {
                        return t * (float)Math.cos(s);
                    }
                });

        MeshView capMeshView1 = new MeshView(capMesh1);
        MeshView capMeshView2 = new MeshView(capMesh2);

        if (materials != null && materials.length > 0){
            int nMat = materials.length;
            capMeshView1.setMaterial( nMat == 1? materials[0] : materials[1]);
            capMeshView2.setMaterial( nMat == 1? materials[0] : nMat == 2? materials[1] : materials[2]);
        }

        group.getChildren().addAll(capMeshView1, capMeshView2);
        return group;
    }


    public static Group box(float length, float height, float depth, int lengthFacets, int heightFacets,
                            int depthFacets, PhongMaterial... materials){
        Group group = new Group();
        TriangleMesh backMesh = getParametricMesh(-length / 2, length / 2, height / 2, -height / 2, lengthFacets, heightFacets,
                new Param3DEqn() {
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
                        return -depth/2;
                    }
                });

        TriangleMesh frontMesh = getParametricMesh(-length / 2, length / 2, -height / 2, height / 2, lengthFacets, heightFacets,
                new Param3DEqn() {
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
                        return depth/2;
                    }
                });

        TriangleMesh leftMesh = getParametricMesh(-depth / 2, depth / 2, -height / 2, height / 2, depthFacets, heightFacets,
                new Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return -length/2;
                    }

                    @Override
                    public float y(float s, float t) {
                        return t;
                    }

                    @Override
                    public float z(float s, float t) {
                        return s;
                    }
                });

        TriangleMesh rightMesh = getParametricMesh(-depth / 2, depth / 2, height / 2, -height / 2, depthFacets, heightFacets,
                new Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return length/2;
                    }

                    @Override
                    public float y(float s, float t) {
                        return t;
                    }

                    @Override
                    public float z(float s, float t) {
                        return s;
                    }
                });

        TriangleMesh topMesh = getParametricMesh(-length / 2, length / 2, depth / 2, -depth / 2, lengthFacets, depthFacets,
                new Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return s;
                    }

                    @Override
                    public float y(float s, float t) {
                        return height/2;
                    }

                    @Override
                    public float z(float s, float t) {
                        return t;
                    }
                });

        TriangleMesh bottomMesh = getParametricMesh(-length / 2, length / 2, -depth / 2, depth / 2, lengthFacets, depthFacets,
                new Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return s;
                    }

                    @Override
                    public float y(float s, float t) {
                        return -height/2;
                    }

                    @Override
                    public float z(float s, float t) {
                        return t;
                    }
                });

        MeshView frontView = new MeshView(frontMesh);
        MeshView backView = new MeshView(backMesh);
        MeshView leftView = new MeshView(leftMesh);
        MeshView rightView = new MeshView(rightMesh);
        MeshView topView = new MeshView(topMesh);
        MeshView bottomView = new MeshView(bottomMesh);

        group.getChildren().addAll(frontView, backView, leftView, rightView, topView, bottomView);

        if (materials == null || materials.length == 0) return group;

        for (int i=0; i<materials.length; i++){
            ((MeshView)group.getChildren().get(i)).setMaterial(materials[i]);
        }

        for (int i=materials.length; i<6; i++){
            ((MeshView)group.getChildren().get(i)).setMaterial(materials[materials.length-1]);
        }

        return group;

    }

}
