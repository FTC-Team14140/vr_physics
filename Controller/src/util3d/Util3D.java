package util3d;

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
                System.out.println("s = " + s + "  t = " + t);
                float x = equations.x(s, t);
                float y = equations.y(s, t);
                float z = equations.z(s, t);
                float u = j * dU;
                float v = i * dV;
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

    public TriangleMesh getOpenParametricMesh(double sMin, double sMax, double tMin, double tMax,
                                              int numFacetS, int numFacetT, Param3DEqn equations){
        return getParametricMesh(sMin, sMax, tMin, tMax, numFacetS, numFacetT, false, false, equations);
    }

}
