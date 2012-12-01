/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package mp_extsimp;

/**
 *
 * @author freeExec
 */
public class XYZ {

    public double x, y, z;

    //Convert (lat,lon) to (x,y,z) on reference ellipsoid
    public static XYZ latLonToXYZ(double lat, double lon) {
        XYZ result = new XYZ();
        double r = 0;
        r = Bbox.DATUM_R_EQUAT * Math.cos(lat * Bbox.DEGTORAD);
        result.z = Bbox.DATUM_R_POLAR * Math.sin(lat * Bbox.DEGTORAD);
        result.x = r * Math.sin(lon * Bbox.DEGTORAD);
        result.y = r * Math.cos(lon * Bbox.DEGTORAD);
        return result;
    }
}
