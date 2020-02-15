package odefx.node_with_geom;

import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DGeom;

public interface Shape3DWithDGeom {

    void setName(String name);
    String getName();
    void setRelGeomOffset(Translate offset);
    Translate getRelGeomOffset();
    void setRelGeomRotation(Rotate rotation);
    Rotate getRelGeomRotation();
    void setDGeom(DGeom dGeom);
    DGeom getDGeom();

}
