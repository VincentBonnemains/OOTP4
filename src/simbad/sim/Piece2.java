package simbad.sim;

import javax.media.j3d.Appearance;
import javax.media.j3d.BoundingBox;
import javax.media.j3d.Bounds;
import javax.media.j3d.Material;
import javax.media.j3d.Node;
import javax.media.j3d.PolygonAttributes;
import javax.media.j3d.QuadArray;
import javax.media.j3d.Shape3D;
import javax.media.j3d.Transform3D;
import javax.media.j3d.TriangleFanArray;
import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Point4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import com.sun.j3d.utils.geometry.Primitive;

public class Piece2 extends SimpleAgent{
	float x,y,z,px,py,pz;
	Color3f c;

	public Piece2(Vector3d pos,Vector3f extent, String name,Color3f c) {
		super(pos, name);
		x = extent.x;
		y = extent.y;
		z = extent.z;
		this.c = c;
	}
	
	/** Create 3D geometry. */
	protected void create3D() {
		int length =  64;
		int tab[]=new int[1];
		tab[0]=length;
		QuadArray quad=new QuadArray(length,QuadArray.COORDINATES|QuadArray.COLOR_3);
		
		Point3f infBasGauche = new Point3f(-(x/2),0,-(z/2));
		Point3f infBasDroit = new Point3f(-(x/2),0,+(z/2));
		Point3f infHautGauche = new Point3f(+(x/2),0,-(z/2));
		Point3f infHautDroit = new Point3f(+(x/2),0,+(z/2));
		
		Point3f supBasGauche = new Point3f(-(x/2),y,-(z/2));
		Point3f supBasDroit = new Point3f(-(x/2),y,+(z/2));
		Point3f supHautGauche = new Point3f(+(x/2),y,-(z/2));
		Point3f supHautDroit = new Point3f(+(x/2),y,+(z/2));
		
		Point3f miinfBasGauche = new Point3f(-(x/2),y/4,-(z/2));
		Point3f miinfBasDroit = new Point3f(-(x/2),y/4,+(z/2));
		Point3f miinfHautGauche = new Point3f(+(x/2),y/4,-(z/2));
		Point3f miinfHautDroit = new Point3f(+(x/2),y/4,+(z/2));
		
		Point3f misupBasGauche = new Point3f(-(x/2),3*y/4,-(z/2));
		Point3f misupBasDroit = new Point3f(-(x/2),3*y/4,+(z/2));
		Point3f misupHautGauche = new Point3f(+(x/2),3*y/4,-(z/2));
		Point3f misupHautDroit = new Point3f(+(x/2),3*y/4,+(z/2));
		
		Point3f interinfBasGauche = new Point3f(-(x/2),y/4,-(z/4));
		Point3f interinfBasDroit = new Point3f(-(x/2),y/4,+(z/4));
		Point3f interinfHautGauche = new Point3f(+(x/2),y/4,-(z/4));
		Point3f interinfHautDroit = new Point3f(+(x/2),y/4,+(z/4));
		
		Point3f intersupBasGauche = new Point3f(-(x/2),3*y/4,-(z/4));
		Point3f intersupBasDroit = new Point3f(-(x/2),3*y/4,+(z/4));
		Point3f intersupHautGauche = new Point3f(+(x/2),3*y/4,-(z/4));
		Point3f intersupHautDroit = new Point3f(+(x/2),3*y/4,+(z/4));
		
		/* face 1              infBasGauche, infBasDroit, infHautDroit, infHautGauche,
		/* face 2              infBasGauche, supBasGauche, supHautGauche, infHautGauche,
		/* face 3              infBasDroit, supBasDroit, supHautDroit, infHautDroit,
		/* face 4              supBasGauche, supBasDroit, supHautDroit, supHautGauche,
		/* face 5 			 infBasGauche, infBasDroit, supBasDroit, supBasGauche,
		/* face 6 			 infHautGauche, infHautDroit, supHautDroit, supHautGauche,
		});*/
		
		quad.setCoordinates(0, new Point3f[] {
				/* face 1*/              infBasGauche, infBasDroit, infHautDroit, infHautGauche,
				/* face 2*/              infBasGauche, miinfBasGauche, miinfHautGauche, infHautGauche,
				/* face 3*/              infBasDroit, miinfBasDroit, miinfHautDroit, infHautDroit,
				/* face 4*/              miinfBasGauche, miinfBasDroit, miinfHautDroit, miinfHautGauche,
				/* face 5*/ 			 infBasGauche, infBasDroit, miinfBasDroit, miinfBasGauche,
				/* face 6*/ 			 infHautGauche, infHautDroit, miinfHautDroit, miinfHautGauche,
				
				/* face 7*/              misupBasGauche, misupBasDroit, misupHautDroit, misupHautGauche,
				/* face 8*/            	 misupBasGauche, supBasGauche, supHautGauche, misupHautGauche,
				/* face 9*/              misupBasDroit, supBasDroit, supHautDroit, misupHautDroit,
				/* face 10*/             supBasGauche, supBasDroit, supHautDroit, supHautGauche,
				/* face 11*/ 			 misupBasGauche, misupBasDroit, supBasDroit, supBasGauche,
				/* face 12*/ 			 misupHautGauche, misupHautDroit, supHautDroit, supHautGauche,
				
				/* face 13*/              interinfBasGauche, interinfBasDroit, intersupHautDroit, intersupHautGauche,
				/* face 14*/           	  interinfBasGauche, intersupBasGauche, intersupHautGauche, interinfHautGauche,
				/* face 15*/              interinfBasDroit, intersupBasDroit, intersupHautDroit, interinfHautDroit,
				/* face 16*/              interinfBasGauche, interinfBasDroit, intersupHautDroit, intersupHautGauche,
				});

		for(int i=1;i<length;i++){ quad.setColor(i,c);}
		

		Appearance app=new Appearance();
        	PolygonAttributes polyAttrib = new PolygonAttributes();
            	polyAttrib.setCullFace(PolygonAttributes.CULL_NONE);
            	polyAttrib.setPolygonMode(PolygonAttributes.POLYGON_FILL);
            	app.setPolygonAttributes(polyAttrib);
	   
		
	 	body = new Shape3D(quad,app);
		//  define the boundsfFor collision  detection
		    BoundingBox bounds = new BoundingBox();
		    bounds.setUpper(x/2,y/2,z/2);
		    bounds.setLower(-x/2,-y/2,-z/2);
		    setBounds(bounds);
        	// we must be able to change the pick flag of the agent
        	body.setCapability(Node.ALLOW_PICKABLE_READ);
        	body.setCapability(Node.ALLOW_PICKABLE_WRITE);
        	body.setPickable(true);
        	body.setCollidable(true);
        	addChild(body);
    	}

}