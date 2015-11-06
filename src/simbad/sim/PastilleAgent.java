
package simbad.sim;

import javax.media.j3d.Appearance;
import javax.media.j3d.BoundingSphere;
import javax.media.j3d.Bounds;

import javax.media.j3d.Material;
import javax.media.j3d.Node;

import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.awt.Color ;

import com.sun.j3d.utils.geometry.Primitive;
import com.sun.j3d.utils.geometry.Sphere;
import com.sun.j3d.utils.geometry.ColorCube;
import javax.media.j3d.*;
import javax.vecmath.Point3f;

import javax.media.j3d.Shape3D;


public class PastilleAgent extends SimpleAgent {

	private Color3f color ;
	private int resolution;
	
	private float radius;

	public PastilleAgent(Vector3d pos, Color3f color, String name, float radius, int resolution){
		super(pos,name);		
		this.color=color;
		this.resolution = resolution;
		this.radius = radius;
	}
	
	/** Create 3D geometry. */
    	protected void create3D() {
       

//le tableau d'entiers figurant comme le dernier paramètre du constructeur permet dans un seul objet ""TriangleStripArray" de définir plusieurs ensembles de triangles distincts. Ainsi, la dimension du tableau correspond au nombre de figures différentes et le contenu de chaque section du tableau donnera le nombre de sommets pour chaque figure. Si l'on passe en paramètre "int tab[3]={5,3,8}", Java3D génèrera alors trois sous-"TriangleStripArray" avec respectivement 5,3 et 8 sommets. Chose importante, le nombre "vertexCount" doit être égal à la somme de toutes les cases du tableau, sans quoi une exception sera générée. Ainsi, "vertexCount" devrait valoir 5+3+8=16.


		int length = resolution + 2;
		int tab[]=new int[1];
		tab[0]=length;

		TriangleFanArray trFan=new TriangleFanArray (length ,TriangleFanArray.COORDINATES|TriangleFanArray.COLOR_3 ,tab);

		Point3f pt0 = new Point3f(radius, 0.0f, 0.0f);
		trFan.setCoordinate(0, pt0);

		Point3f pt = new Point3f();
		for (int i = 1; i < resolution; i++)
		{
   			pt.x = (float)(radius * Math.cos(2*i * Math.PI / resolution));
   			pt.z = (float)(radius * Math.sin(2*i * Math.PI / resolution));
   			pt.y = 0.0f;
   			trFan.setCoordinate(i, pt);
		}
		trFan.setCoordinate(resolution, pt0);

		trFan.setColor(0,color);

		for(int i=1;i<resolution;i++) trFan.setColor(i,color); 

		Appearance app=new Appearance();
        	PolygonAttributes polyAttrib = new PolygonAttributes();
            	polyAttrib.setCullFace(PolygonAttributes.CULL_NONE);
            	polyAttrib.setPolygonMode(PolygonAttributes.POLYGON_FILL);
            	app.setPolygonAttributes(polyAttrib);
	   
		
	 	body = new Shape3D(trFan,app);

	
        	// we must be able to change the pick flag of the agent
        	body.setCapability(Node.ALLOW_PICKABLE_READ);
        	body.setCapability(Node.ALLOW_PICKABLE_WRITE);
        	body.setPickable(true);
        	body.setCollidable(true);
        	addChild(body);
    	}
}

