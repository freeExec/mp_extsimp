/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package mp_extsimp;

/**
 *
 * @author freeExec
 */
public class Highway {
    // Consts

    //OSM highway main types
    public static final int HIGHWAY_MOTORWAY = 0;
    public static final int HIGHWAY_MOTORWAY_LINK = 1;
    public static final int HIGHWAY_TRUNK = 2;
    public static final int HIGHWAY_TRUNK_LINK = 3;
    public static final int HIGHWAY_PRIMARY = 4;
    public static final int HIGHWAY_PRIMARY_LINK = 5;
    public static final int HIGHWAY_SECONDARY = 6;
    public static final int HIGHWAY_SECONDARY_LINK = 7;
    public static final int HIGHWAY_TERTIARY = 8;
    public static final int HIGHWAY_TERTIARY_LINK = 9;

    //OSM highway minor types (should not occur)
    public static final int HIGHWAY_LIVING_STREET = 10;
    public static final int HIGHWAY_RESIDENTIAL = 12;
    public static final int HIGHWAY_UNCLASSIFIED = 14;
    public static final int HIGHWAY_SERVICE = 16;
    public static final int HIGHWAY_TRACK = 18;
    public static final int HIGHWAY_OTHER = 20;
    public static final int HIGHWAY_UNKNOWN = 22;
    public static final int HIGHWAY_UNSPECIFIED = 24;

    //Masks
    //all links
    public static final int HIGHWAY_MASK_LINK = 1;
    //get main type (removes _link)
    public static final int HIGHWAY_MASK_MAIN = 254;

    
        //Parse OSM highway class to our own constants
    public static byte getHighwayType(String text) {
        byte _rtn = 0;
        // предположительно trim лишний, т.к. еще обрезаеться до входа
        switch (text.trim().toLowerCase()) {
            case  "primary":
                _rtn = HIGHWAY_PRIMARY;
                break;
            case  "primary_link":
                _rtn = HIGHWAY_PRIMARY_LINK;
                break;
            case  "secondary":
                _rtn = HIGHWAY_SECONDARY;
                break;
            case  "secondary_link":
                _rtn = HIGHWAY_SECONDARY_LINK;
                break;
            case  "tertiary":
                _rtn = HIGHWAY_TERTIARY;
                break;
            case  "tertiary_link":
                _rtn = HIGHWAY_TERTIARY_LINK;
                break;
            case  "motorway":
                _rtn = HIGHWAY_MOTORWAY;
                break;
            case  "motorway_link":
                _rtn = HIGHWAY_MOTORWAY_LINK;
                break;
            case  "trunk":
                _rtn = HIGHWAY_TRUNK;
                break;
            case  "trunk_link":
                _rtn = HIGHWAY_TRUNK_LINK;
                break;
            case  "living_street":
                _rtn = HIGHWAY_LIVING_STREET;
                break;
            case  "residential":
                _rtn = HIGHWAY_RESIDENTIAL;
                break;
            case  "unclassified":
                _rtn = HIGHWAY_UNCLASSIFIED;
                break;
            case  "service":
                _rtn = HIGHWAY_SERVICE;
                break;
            case  "track":
                _rtn = HIGHWAY_TRACK;
                break;
            case  "road":
                _rtn = HIGHWAY_UNKNOWN;
                break;
            default:
                _rtn = HIGHWAY_OTHER;
                break;
        }
        return _rtn;
    }
 
}
