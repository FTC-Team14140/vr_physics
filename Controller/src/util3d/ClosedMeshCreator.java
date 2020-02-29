package util3d;

import javafx.scene.shape.Mesh;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;

/**
 * Engine for creation of closed TriangleMesh objects.
 *
 * Points comprising the "base" have k=0; i from 0..Ni; j from 0..Nj
 * Points comprising the "top" have k=Nk; i from 0..Ni; j from 0..Nj
 * The "perimeter" includes the perimeter points along the base and top, as well as Nk-1 additional
 *   layers, each of which has 2*(Ni+Nj) points.
 *
 * Indices of points for the "base"  (k=0) range from 0..(Ni+1)*(Nj+1)-1
 *
 * Indices of points for the k-th layer (0<k<Nk) range from (Ni+1)*(Nj+1) + 2*(k-1)*(Ni+Nj) .. (Ni+1)*(Nj+1) + 2*k*(Ni+Nj) - 1
 *
 * Indices of points for the "top" layer (k=Nk) range from (Ni+1)*(Nj+1) + 2*(Nk-1)*(Ni+Nj) .. 2*(Ni+1)*(Nj+1) + 2*(Nk-1)*(Ni+Nj) -1
 *
 * So, total number of points is 2*(Ni+1)*(Nj+1) + 2*(Nk-1)*(Ni+Nj)
 * The points array will have thrice that length, and the texCoords array will have double that length.
 *
 * Indices of "rectangular" (i.e., pairs of triangular) faces are:
 * Base layer: 0 .. Ni*Nj-1
 * k-th layer of perimeter (0<=k<Nk): Ni*Nj + 2*k*(Ni+Nj) .. Ni*Nj + 2*(k+1)*(Ni+Nj) - 1
 * Top layer: Ni*Nj + 2*Nk*(Ni+Nj) .. 2 * (Ni*Nj + Nk*(Ni+Nj)) -1
 */

public abstract class ClosedMeshCreator {

    /**
     * Ni, Nj, Nk = #rectangular faces (i.e. half of the number of triangular faces) in the i, j, k dimensions.
     */

    protected final int Ni, Nj, Nk;
    protected final float[] points;
    protected final float[] texCoords;
    private final int[] faces;

    public ClosedMeshCreator(int ni, int nj, int nk){
        Ni = ni;  Nj = nj;  Nk = nk;
        int pointArrayLength = 6 * ((Ni+1)*(Nj+1) + (Nk-1)*(Ni+Nj));
        int texCoordArrayLength = 2 * (2 * (Ni+1) * (Nj+1) + 2 * (Nk-1) * (Ni + Nj) + Nk+1);
        points = createPoints();
        texCoords = createTexCoords();
        if (points.length != pointArrayLength){
            System.out.println("Warning: Length of Point Array is Invalid in ClosedMeshCreator.");
            System.out.println("Actual length: " + points.length + "   Required length: " + pointArrayLength);
        }
        if (texCoords.length != texCoordArrayLength){
            System.out.println("Warning: Length of TexCoord Array is Invalid in ClosedMeshCreator.");
        }
        faces = new int[12 * (2* (Ni*Nj + Nk*(Ni+Nj)))];
        createFaces();
    }

    protected abstract float[] createPoints();

    protected float[] createTexCoords() {
        int numTexPoints = 2 * (Ni+1) * (Nj+1) + 2 * (Nk-1) * (Ni + Nj) + Nk+1;
        float[] txc = new float[2 * numTexPoints];

        for (int j=0; j<=Nj; j++){
            for (int i=0; i<=Ni; i++){
                int index = 2*(i + j*(Ni+1));
                float u = 0.25f * (float)i/(float)Ni;
                float v = 0.6667f + 0.3333f * (float)(Nj - j)/(float)Nj;
                txc[index] = u;
                txc[index+1] = v;
            }
        }

        for (int k=1; k<Nk; k++) {
            for (int q = 0; q < (2 * (Ni + Nj)); q++) {
                float u = (float) q / (2.0f * (Ni + Nj));
                float v = 0.3333f + 0.3333f * (float) (Nk - k)/ (float) Nk;
                int index = 2 * ((Ni + 1) * (Nj + 1) + 2 * (k - 1) * (Ni + Nj) + q);
                txc[index] = u;
                txc[index+1] = v;
            }
        }

        for (int k=0; k<=Nk; k++){
            int index = 2 * ( 2*(Ni+1)*(Nj+1) + 2*(Nk-1)*(Ni+Nj) + k);
            float v = 0.3333f + 0.3333f * (float)(Nk - k) / (float)Nk;
            txc[index] = 1.0f;
            txc[index+1] = v;
        }

        for (int j=0; j<=Nj; j++){
            for (int i=0; i<=Ni; i++){
                int index = 2 * ( (Ni+1)*(Nj+1) + 2*(Nk-1)*(Ni+Nj) + i + j*(Ni+1) );
                float u = 0.25f * (float)i/(float)Ni;
                float v = 0.3333f * (float)(Nj - j)/(float)Nj;
                txc[index] = u;
                txc[index+1] = v;
            }
        }

        return txc;
    };

    protected void createFaces(){

        int n0F = 0;

        //Base layer
        for (int j=0; j<Nj; j++){
            for (int i=0; i<Ni; i++){
                int nF = 12*(i+j*Ni);
                faces[nF] = faces[nF+1] = i+j*(Ni+1);
                faces[nF+2] = faces[nF+3] = i+(j+1)*(Ni+1);
                faces[nF+4] = faces[nF+5] = i+1+j*(Ni+1);
                faces[nF+6] = faces[nF+7] = i+(j+1)*(Ni+1);
                faces[nF+8] = faces[nF+9] = i+1+(j+1)*(Ni+1);
                faces[nF+10] = faces[nF+11] = i+1+j*(Ni+1);
            }
        }

        //k perimeter layers
        for (int k=0; k<Nk; k++){

            n0F = Ni*Nj + 2*k*(Ni+Nj);
            int numFacets = 2 * (Ni+Nj);
            int endTexIndex = 2*(Ni+1)*(Nj+1) + 2*(Nk-1)*(Ni+Nj) + k;

            for (int q=0; q<numFacets; q++){
                int p1 = getPerimeterIndex(k, q);
                int p2 = getPerimeterIndex(k, q+1);
                int p3 = getPerimeterIndex(k+1, q+1);
                int p4 = getPerimeterIndex(k+1, q);
                int nF = 12 * (n0F + q);
                faces[nF] = faces[nF+1] = p1;
                faces[nF+2] = p2;
                faces[nF+3] = (q+1)==2*(Ni+Nj)? endTexIndex : p2;
                faces[nF+4] = faces[nF+5] = p4;
                faces[nF+6] = p2;
                faces[nF+7] = (q+1)==2*(Ni+Nj)? endTexIndex : p2;
                faces[nF+8] = p3;
                faces[nF+9] = (q+1)==2*(Ni+Nj)? endTexIndex+1 : p3;
                faces[nF+10] = faces[nF+11] = p4;

//                System.out.println();
//                System.out.println("SIDE TEX COORDS");
//                System.out.printf("k=%d  q=%d  Indices: %d %d %d Coords: (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)\n", k, q,
//                        faces[nF+1], faces[nF+3], faces[nF+5],
//                        texCoords[2*faces[nF+1]], texCoords[2*faces[nF+1]+1],
//                        texCoords[2*faces[nF+3]], texCoords[2*faces[nF+3]+1],
//                        texCoords[2*faces[nF+5]], texCoords[2*faces[nF+5]+1]);
//                System.out.printf("k=%d  q=%d  Indices: %d %d %d Coords: (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)\n", k, q,
//                        faces[nF+7], faces[nF+9], faces[nF+11],
//                        texCoords[2*faces[nF+7]], texCoords[2*faces[nF+1]+7],
//                        texCoords[2*faces[nF+9]], texCoords[2*faces[nF+9]+1],
//                        texCoords[2*faces[nF+11]], texCoords[2*faces[nF+11]+1]);
            }
        }

        //Top layer
        n0F = Ni*Nj + 2*Nk*(Ni+Nj);
        int n0P = (Ni+1)*(Nj+1) + 2*(Nk-1)*(Ni+Nj);
        for (int j=0; j<Nj; j++){
            for (int i=0; i<Ni; i++){
                int nF = 12 * (n0F + (i+j*Ni));
                faces[nF] = faces[nF+1] = n0P + i+(j+1)*(Ni+1);
                faces[nF+2] = faces[nF+3] = n0P + i+j*(Ni+1);
                faces[nF+4] = faces[nF+5] = n0P + i+1+j*(Ni+1);
                faces[nF+6] = faces[nF+7] = n0P + i+1+(j+1)*(Ni+1);
                faces[nF+8] = faces[nF+9] = n0P + i+(j+1)*(Ni+1);
                faces[nF+10] = faces[nF+11] = n0P + i+1+j*(Ni+1);
            }
        }

//        System.out.println("FACETS:");
//        int numFacets = 4 * (Ni*Nj) + 4*Nk*(Ni+Nj);
//        for (int n = 0; n < numFacets; n++) {
//            int nf = 6*n;
//            int np0 = 3*faces[nf];
//            int np1 = 3*faces[nf+2];
//            int np2 = 3*faces[nf+4];
//            System.out.printf("%d: Indices: %d %d %d  Points: (%.0f, %.0f, %.0f), (%.0f, %.0f, %.0f), (%.0f, %.0f, %.0f)\n", n,
//                   np0, np1, np2,
//                   points[np0], points[np0+1], points[np0+2],
//                    points[np1], points[np1+1], points[np1+2],
//                    points[np2], points[np2+1], points[np2+2]);
//        }
//        System.out.println();

    }

    /**
     * Return the point (and texture coordinate) index corresponding to the q-th point around base perimeter
     * @param q
     * @return
     */
    private int getBasePerimeterIndex(int q){
        if (q < Ni){
            return q;
        } else if (q < (Ni+Nj)){
            return Ni + (q-Ni) * (Ni+1);
        } else if (q < (2*Ni+Nj)){
            return (Ni+1)*(Nj+1)-1 - (q-(Ni+Nj));
        } else {
            return Nj*(Ni+1) - (q - (2*Ni+Nj))*(Ni+1);
        }
    }

    private int getTopPerimeterIndex(int q){
        int n0 = (Ni+1)*(Nj+1) + 2*(Nk-1)*(Ni+Nj);
        return n0 + getBasePerimeterIndex(q);
    }

    private int getPerimeterIndex(int k, int q){
        if (k == 0){
            return getBasePerimeterIndex(q);
        } else if (k == Nk){
            return getTopPerimeterIndex(q);
        } else {
            int numPerimLayerPoints = 2*(Ni+Nj);
            int n0P = (Ni+1)*(Nj+1) + 2*(k-1)*(Ni+Nj);
            return q < numPerimLayerPoints? n0P + q : n0P;
        }
    }

    public Mesh newMeshInstance(){
        TriangleMesh mesh = new TriangleMesh();
        ((TriangleMesh) mesh).setVertexFormat(VertexFormat.POINT_TEXCOORD);
        mesh.getPoints().addAll(points);
        mesh.getTexCoords().addAll(texCoords);
        mesh.getFaces().addAll(faces);
        return mesh;
    }


}
