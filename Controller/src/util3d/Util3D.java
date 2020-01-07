package util3d;

import com.sun.javafx.geom.PickRay;
import com.sun.javafx.scene.input.PickResultChooser;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.*;

public class Util3D {

    public interface Param3DEqn{
        float x(float s, float t);
        float y(float s, float t);
        float z(float s, float t);
    }

    public interface Param3DEqnExt extends Param3DEqn {
        float u(float s, float t);
        float v(float s, float t);
    }

    public interface Param2DEqn{
        float x(float s);
        float y(float s);
    }

    public static TriangleMesh getParametricMesh(double sMin, double sMax, double tMin, double tMax,
                                                 int numFacetS, int numFacetT, boolean sClosed, boolean tClosed,
                                                 Param3DEqnExt equations){

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
                float u = equations.u(s, t);
                float v = equations.v(s, t);
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
                                            int numFacetS, int numFacetT, boolean sClosed, boolean tClosed,
                                            Param3DEqn equations){


        Param3DEqnExt extEquations = new Param3DEqnExt() {
            @Override
            public float u(float s, float t) {
                return (float)(s - sMin) / (float)(sMax - sMin);
            }

            @Override
            public float v(float s, float t) {
                return 1.0f - (float)(t - tMin) / (float)(tMax - tMin);
            }

            @Override
            public float x(float s, float t) {
                return equations.x(s, t);
            }

            @Override
            public float y(float s, float t) {
                return equations.y(s, t);
            }

            @Override
            public float z(float s, float t) {
                return equations.z(s, t);
            }
        };

        return getParametricMesh(sMin, sMax, tMin, tMax, numFacetS, numFacetT, sClosed, tClosed, extEquations);

    }


    public static TriangleMesh getParametricMesh(double sMin, double sMax, double tMin, double tMax,
                                                 int numFacetS, int numFacetT, Param3DEqn equations){
        return getParametricMesh(sMin, sMax, tMin, tMax, numFacetS, numFacetT, false, false, equations);
    }

    public static TriangleMesh getParametricMesh(float sMin, float sMax, float tMin, float tMax, float deltaS,
                                                 float deltaT, Param3DEqn equations){
        deltaS = Math.abs(deltaS) * Math.signum(sMax - sMin);
        deltaT = Math.abs(deltaT) * Math.signum(tMax - tMin);
        float facS = (sMax - sMin) / deltaS;
        float facT = (tMax - tMin) / deltaT;
        float fracS = facS - (float)Math.floor(facS);
        float fracT = facT - (float)Math.floor(facT);
        int numFacetS = fracS < 0.001? Math.round((float)Math.floor(facS)) : Math.round((float)Math.floor(facS)) + 1;
        int numFacetT = fracT < 0.001? Math.round((float)Math.floor(facT)) : Math.round((float)Math.floor(facT)) + 1;

        float[] points = new float[numFacetS * numFacetT * 4 * 3];
        float[] texCoords = new float[numFacetS * numFacetT * 4 * 2];
        int[] facets = new int[12 * numFacetS * numFacetT];

        for (int i=0; i<numFacetT; i++){
            for (int j=0; j<numFacetS; j++){
                for (int k=0; k<4; k++){
                    int ii = i + k/2;
                    int jj = j + k%2;
                    float s = jj<numFacetS? sMin + jj * deltaS : sMax;
                    float t = ii<numFacetT? tMin + ii * deltaT : tMax;
                    float x = equations.x(s, t);
                    float y = equations.y(s, t);
                    float z = equations.z(s, t);
                    float u = k%2;
                    float v = k/2;
                    points[12*(numFacetS*i + j)+ 3*k] = x;
                    points[12*(numFacetS*i + j) + 3*k + 1] = y;
                    points[12*(numFacetS*i + j) + 3*k + 2] = z;
                    texCoords[8*(numFacetS*i + j) + 2*k] = u;
                    texCoords[8*(numFacetS*i + j) + 2*k + 1] = v;
                }
            }
        }

        for (int i=0; i<numFacetT; i++){
            for (int j=0; j<numFacetS; j++){
                int facetBaseIndex = 12 * (numFacetS*i + j);
                facets[facetBaseIndex] = facets[facetBaseIndex+1] = 4*(numFacetS*i + j);
                facets[facetBaseIndex+2] = facets[facetBaseIndex+3] = 4*(numFacetS*i + j) + 1;
                facets[facetBaseIndex+4] = facets[facetBaseIndex+5] = 4*(numFacetS*i + j) + 2;
                facets[facetBaseIndex+6] = facets[facetBaseIndex+7] = 4*(numFacetS*i + j) + 1;
                facets[facetBaseIndex+8] = facets[facetBaseIndex+9] = 4*(numFacetS*i + j) + 3;
                facets[facetBaseIndex+10] = facets[facetBaseIndex+11] = 4*(numFacetS*i + j) + 2;
            }
        }

        TriangleMesh mesh = new TriangleMesh(VertexFormat.POINT_TEXCOORD);

        mesh.getPoints().addAll(points);
        mesh.getTexCoords().addAll(texCoords);
        mesh.getFaces().addAll(facets);

        return mesh;

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
                            int depthFacets, boolean hasEndCaps, boolean outsideView, PhongMaterial... materials){

        Group group = new Group();

        double m = outsideView? 1.0 : -1.0;

        TriangleMesh backMesh = getParametricMesh(-length / 2, length / 2, m * height / 2, -m * height / 2, lengthFacets, heightFacets,
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

        TriangleMesh frontMesh = getParametricMesh(-length / 2, length / 2, -m * height / 2, m * height / 2, lengthFacets, heightFacets,
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

        TriangleMesh leftMesh = getParametricMesh(-depth / 2, depth / 2, -m * height / 2, m * height / 2, depthFacets, heightFacets,
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

        TriangleMesh rightMesh = getParametricMesh(-depth / 2, depth / 2, m * height / 2, -m * height / 2, depthFacets, heightFacets,
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

        MeshView frontView = new MeshView(frontMesh);
        MeshView backView = new MeshView(backMesh);
        MeshView leftView = new MeshView(leftMesh);
        MeshView rightView = new MeshView(rightMesh);

        group.getChildren().addAll(frontView, backView, leftView, rightView);

        if (hasEndCaps) {

            TriangleMesh topMesh = getParametricMesh(-length / 2, length / 2, m * depth / 2, -m * depth / 2, lengthFacets, depthFacets,
                    new Param3DEqn() {
                        @Override
                        public float x(float s, float t) {
                            return s;
                        }

                        @Override
                        public float y(float s, float t) {
                            return height / 2;
                        }

                        @Override
                        public float z(float s, float t) {
                            return t;
                        }
                    });

            TriangleMesh bottomMesh = getParametricMesh(-length / 2, length / 2, -m * depth / 2, m * depth / 2, lengthFacets, depthFacets,
                    new Param3DEqn() {
                        @Override
                        public float x(float s, float t) {
                            return s;
                        }

                        @Override
                        public float y(float s, float t) {
                            return -height / 2;
                        }

                        @Override
                        public float z(float s, float t) {
                            return t;
                        }
                    });

            MeshView topView = new MeshView(topMesh);
            MeshView bottomView = new MeshView(bottomMesh);

            group.getChildren().addAll(topView, bottomView);

        }


        if (materials == null || materials.length == 0) return group;

        int numSides = group.getChildren().size();

        for (int i=0; i<materials.length; i++){
            ((MeshView)group.getChildren().get(i)).setMaterial(materials[i]);
        }

        for (int i=materials.length; i<numSides; i++){
            ((MeshView)group.getChildren().get(i)).setMaterial(materials[materials.length-1]);
        }

        return group;

    }

    public static Group box(float length, float height, float depth, int lengthFacets, int heightFacets,
                            int depthFacets, PhongMaterial... materials){
        return box(length, height, depth, lengthFacets, heightFacets, depthFacets, true, true,
                materials);
    }

    public static Group patternBox(float length, float height, float depth, float patternLength, float patternHeight,
                                   float patternDepth, PhongMaterial... materials){

        Group group = new Group();

        TriangleMesh backMesh = getParametricMesh(-length / 2, length / 2, height / 2, -height / 2, patternLength, patternHeight,
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
                    public float z(float s, float t) { return -depth / 2; }
                });

        TriangleMesh frontMesh = getParametricMesh(-length / 2, length / 2, -height / 2, height / 2, patternLength, patternHeight,
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
                        return depth / 2;
                    }
                });

        TriangleMesh leftMesh = getParametricMesh(-depth / 2, depth / 2, -height / 2, height / 2, patternDepth, patternHeight,
                new Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return -length / 2;
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

        TriangleMesh rightMesh = getParametricMesh(-depth / 2, depth / 2, height / 2, -height / 2, patternDepth, patternHeight,
                new Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return length / 2;
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

        TriangleMesh topMesh = getParametricMesh(-length / 2, length / 2, depth / 2, -depth / 2, patternLength, patternDepth,
                new Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return s;
                    }

                    @Override
                    public float y(float s, float t) {
                        return height / 2;
                    }

                    @Override
                    public float z(float s, float t) {
                        return t;
                    }
                });

        TriangleMesh bottomMesh = getParametricMesh(-length / 2, length / 2, -depth / 2, depth / 2, patternLength, patternDepth,
                new Param3DEqn() {
                    @Override
                    public float x(float s, float t) {
                        return s;
                    }

                    @Override
                    public float y(float s, float t) {
                        return -height / 2;
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

        int numSides = group.getChildren().size();

        for (int i = 0; i < materials.length; i++) {
            ((MeshView) group.getChildren().get(i)).setMaterial(materials[i]);
        }

        for (int i = materials.length; i < numSides; i++) {
            ((MeshView) group.getChildren().get(i)).setMaterial(materials[materials.length - 1]);
        }

        return group;

    }

    public static TriangleMesh polygonMesh(boolean reversed, float... xyPoints){
        if (xyPoints.length%2 != 0) return null;
        int numPts = xyPoints.length/2;
        int numFaces = numPts%2 == 0? 2*(numPts/2 - 1) : 2*(numPts/2 - 1) + 1;

        float[] points = new float[3*numPts];
        float[] texCoords = new float[2*numPts];
        int[] faces = new int[6*numFaces];

        float xMax = xyPoints[0], xMin = xyPoints[0], yMax = xyPoints[1], yMin = xyPoints[1];

        for (int i=0; i< numPts; i++){
            if (xyPoints[2*i] > xMax) xMax = xyPoints[2*i];
            if (xyPoints[2*i] < xMin) xMin = xyPoints[2*i];
            if (xyPoints[2*i+1] > yMax) yMax = xyPoints[2*i+1];
            if (xyPoints[2*i+1] < yMin) yMin = xyPoints[2*i+1];
            points[3*i] = xyPoints[2*i];
            points[3*i+1] = xyPoints[2*i+1];
            points[3*i+2] = 0;
        }

        for (int i=0; i<numPts; i++){
            texCoords[2*i] = (xyPoints[2*i] - xMin)/(xMax - xMin);
            texCoords[2*i+1] = (yMax - xyPoints[2*i+1])/(yMax - yMin);
        }

        for (int i=0; i<numPts/2-1; i++){
            faces[i*12] = faces[i*12+1] = i;
            if (!reversed) {
                faces[i * 12 + 2] = faces[i * 12 + 3] = i + 1;
                faces[i * 12 + 4] = faces[i * 12 + 5] = numPts - 1 - i;
            } else {
                faces[i * 12 + 2] = faces[i * 12 + 3] = numPts - 1 - i;
                faces[i * 12 + 4] = faces[i * 12 + 5] = i + 1;
            }
            faces[i*12+6] = faces[i*12+7] = i+1;
            if (!reversed) {
                faces[i * 12 + 8] = faces[i * 12 + 9] = numPts - 2 - i;
                faces[i * 12 + 10] = faces[i * 12 + 11] = numPts - 1 - i;
            } else {
                faces[i * 12 + 8] = faces[i * 12 + 9] = numPts - 1 - i;
                faces[i * 12 + 10] = faces[i * 12 + 11] = numPts - 2 - i;
            }
        }

        TriangleMesh mesh = new TriangleMesh();
        mesh.getPoints().addAll(points);
        mesh.getTexCoords().addAll(texCoords);
        mesh.getFaces().addAll(faces);
        mesh.setVertexFormat(VertexFormat.POINT_TEXCOORD);

        return mesh;
    }

    public static MeshView polygonView(boolean reversed, PhongMaterial material, float... xyPoints){
        TriangleMesh mesh = polygonMesh(reversed, xyPoints);
        MeshView view = new MeshView(mesh);
        view.setMaterial(material);
        return view;
    }

    public static MeshView polygonView(boolean reversed, PhongMaterial material, float sMin, float sMax,
                                       int numPts, Param2DEqn equations){
        float dS = (sMax - sMin) / (numPts-1);

        float[] xyPoints = new float[2*numPts];

        for (int i=0; i<numPts; i++){
            float s = sMin + dS*i;
            xyPoints[2*i] = equations.x(s);
            xyPoints[2*i+1] = equations.y(s);
        }

        return polygonView(reversed, material, xyPoints);
    }

    public static Group polygonBox(float depth, float[] xyPoints, PhongMaterial... materials){

        int numXYPts = xyPoints.length/2;
        int numSidePts = numXYPts + 1;

        float[] sidePts = new float[numSidePts*6];
        float[] sideTexCoords = new float[numSidePts*4];

        float pathLength = 0.0f;
        for (int i=0; i<numXYPts; i++){
            float x0 = xyPoints[2*i];
            float y0 = xyPoints[2*i+1];
            float x1 = i<numXYPts-1 ? xyPoints[2*i+2] : xyPoints[0];
            float y1 = i<numXYPts-1 ? xyPoints[2*i+3] : xyPoints[1];
            pathLength += (float)Math.hypot(x1-x0, y1-y0);
        }

        float path = 0.0f;

        for (int i=0; i<numXYPts; i++){
            sidePts[6*i] = sidePts[6*i+3] = xyPoints[2*i];
            sidePts[6*i+1] = sidePts[6*i+4] = xyPoints[2*i+1];
            sidePts[6*i+2] = depth/2.0f;
            sidePts[6*i+5] = -depth/2.0f;
            sideTexCoords[4*i] = sideTexCoords[4*i+2] = path/pathLength;
            sideTexCoords[4*i+1] = 1;
            sideTexCoords[4*i+3] = 0;
            if (i<numXYPts-1) path += (float)Math.hypot(xyPoints[2*i+2] - xyPoints[2*i], xyPoints[2*i+3] - xyPoints[2*i+1]);
        }

        sidePts[6*numXYPts] = sidePts[6*numXYPts+3] = xyPoints[0];
        sidePts[6*numXYPts+1] = sidePts[6*numXYPts+4] = xyPoints[1];
        sidePts[6*numXYPts+2] = depth/2.0f;
        sidePts[6*numXYPts+5] = -depth/2.0f;
        sideTexCoords[4*numXYPts] = sideTexCoords[4*numXYPts+2] = 1;
        sideTexCoords[4*numXYPts+1] = 1;
        sideTexCoords[4*numXYPts+3] = 0;

        int[] sideFaces = new int[numXYPts * 12];

        for (int i=0; i<numXYPts; i++){
            sideFaces[i*12] = sideFaces[i*12+1] = 2*i;
            sideFaces[i*12+2] = sideFaces[i*12+3] = 2*i+1;
            sideFaces[i*12+4] = sideFaces[i*12+5] = 2*i+3;
            sideFaces[i*12+6] = sideFaces[i*12+7] = 2*i+3;
            sideFaces[i*12+8] = sideFaces[i*12+9] = 2*i+2;
            sideFaces[i*12+10] = sideFaces[i*12+11] = 2*i;
        }

        TriangleMesh sideMesh = new TriangleMesh();
        sideMesh.getPoints().addAll(sidePts);
        sideMesh.getTexCoords().addAll(sideTexCoords);
        sideMesh.getFaces().addAll(sideFaces);
        sideMesh.setVertexFormat(VertexFormat.POINT_TEXCOORD);

        MeshView sideView = new MeshView(sideMesh);

        TriangleMesh topMesh = polygonMesh(false, xyPoints);
        TriangleMesh bottomMesh = polygonMesh(true, xyPoints);

        MeshView topView = new MeshView(topMesh);
        topView.setTranslateZ(depth/2.0);
        MeshView bottomView = new MeshView(bottomMesh);
        bottomView.setTranslateZ(-depth/2.0);

        Group group = new Group();
        group.getChildren().addAll(topView, bottomView, sideView);

        if (materials == null) return group;
        int numMaterials = materials.length;

        if (numMaterials > 0){
            topView.setMaterial(materials[0]);
            if (numMaterials > 1){
                bottomView.setMaterial(materials[1]);
                if (numMaterials > 2){
                    sideView.setMaterial(materials[2]);
                } else {
                    sideView.setMaterial(materials[1]);
                }
            } else {
                bottomView.setMaterial(materials[0]);
                sideView.setMaterial(materials[0]);
            }
        }

        return group;
    }

    public static PhongMaterial imageMaterial(Image image){
        PhongMaterial mat = new PhongMaterial();
        mat.setDiffuseColor(Color.gray(0.02));
        mat.setDiffuseMap(image);
        mat.setSelfIlluminationMap(image);
        return mat;
    }

    public static PhongMaterial imageMaterial(String imageFileUrl){
        Image image = new Image(imageFileUrl);
        return imageMaterial(image);
    }


}
