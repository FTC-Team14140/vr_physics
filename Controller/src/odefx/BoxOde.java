package odefx;


import javafx.scene.shape.Box;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.OdeHelper;

public class BoxOde extends Box implements HasDGeom {
    private DGeom dGeom;

    public BoxOde(double lx, double ly, double lz, FxBody fb){
        super(lx, ly, lz);
        dGeom = OdeHelper.createBox(lx, ly, lz);
        fb.addGeom(dGeom);
    }

    public BoxOde(double lx, double ly, double lz, FxBody fb, String id){
        this(lx, ly, lz, fb);
        this.setId(id);
        this.dGeom.setData(id);
    }

    public DGeom getDGeom(){ return dGeom; }

}
