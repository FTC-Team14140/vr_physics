package odefx.node_with_geom;


import javafx.scene.shape.Box;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import odefx.FxBody;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.OdeHelper;

public class BoxWithDGeom extends Box implements Shape3DWithDGeom {
    private String name = "BoxWithDGeom";
    private DGeom dGeom;
    private Rotate rotate = null;
    private Translate translate = null;

    public BoxWithDGeom(double lx, double ly, double lz, FxBody fb){
        super(lx, ly, lz);
        dGeom = OdeHelper.createBox(lx, ly, lz);
        fb.addGeom(dGeom);
    }

    public BoxWithDGeom(double lx, double ly, double lz, FxBody fb, String name){
        this(lx, ly, lz, fb);
        this.name = name;
        this.dGeom.setData(name);
    }

    public BoxWithDGeom(double lx, double ly, double lz, FxBody fb, DGeom geom){
        super(lx, ly, lz);
        dGeom = geom;
        fb.addGeom(dGeom);
        this.name = name;
    }

    public void setDGeom(DGeom geom){
        DBody db = dGeom == null? null : dGeom.getBody();
        if (dGeom != null) dGeom.destroy();
        dGeom = geom;
        if (dGeom != null && db != null) dGeom.setBody(db);
    }
    public DGeom getDGeom(){ return dGeom; }

    public void setName(String name){
        this.name = name;
        if (dGeom != null) dGeom.setData(name);
    }

    public String getName(){ return name; }

    public void setRelGeomRotation(Rotate rotate){ this.rotate = rotate; }

    public Rotate getRelGeomRotation(){ return rotate; }

    public void setRelGeomOffset(Translate translate) { this.translate = translate; }

    public Translate getRelGeomOffset () { return translate; }

}
