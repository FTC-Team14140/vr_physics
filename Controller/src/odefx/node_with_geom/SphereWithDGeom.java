package odefx.node_with_geom;


import javafx.scene.shape.Box;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import odefx.FxBody;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.OdeHelper;

public class SphereWithDGeom extends Sphere implements Shape3DWithDGeom {
    private String name = "SphereWithDGeom";
    private DGeom dGeom;
    private Rotate rotate = null;
    private Translate translate = null;

    public SphereWithDGeom(double radius, FxBody fb){
        super(radius);
        dGeom = OdeHelper.createSphere(radius);
        fb.addGeom(dGeom);
    }

    public SphereWithDGeom(double radius, FxBody fb, String name){
        this(radius, fb);
        this.name = name;
        this.dGeom.setData(name);
    }

    public void setDGeom(DGeom geom){ dGeom = geom; }
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
