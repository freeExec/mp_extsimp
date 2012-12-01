/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package mp_extsimp;

/**
 *
 * @author freeExec
 */
public class Node {

    // Consts
    //Specific marking of edges/nodes for algorithms
    //public static final int MARK_JUNCTION = 1;
    //public static final int MARK_COLLAPSING = 2;
    //public static final int MARK_AIMING = 4;
    //public static final int MARK_DISTCHECK = 8;
    //public static final int MARK_SIDE1CHECK = 8;
    //public static final int MARK_SIDE2CHECK = 16;
    //public static final int MARK_SIDESCHECK = 24;
    //public static final int MARK_WAVEPASSED = 16;
    //public static final int MARK_NODE_BORDER = -2;
    //public static final int MARK_NODE_OF_JUNCTION = -3;
    public static final int MARK_NODEID_DELETED = -2;

    public double lat;      //Latitude
    public double lon;      //Longitude
    public int nodeID;     //NodeID from source .mp, -1 - not set, -2 - node killed
    public int[] edge;   //all edges (values - indexes in Edges array)
    public int Edges;      //number of edges, -1 means "not counted"
    public int mark;       //internal marker for all-network algo-s
    public double temp_dist;//internal value for for all-network algo-s
    
    public Node() {
        Init();        
    }
    
    public Node(int nodeID) {
        this.nodeID = nodeID;
        Init();
    }
    
    public void addEdge(int Id) {
        this.edge[Edges++] = Id;
    }
    
    private void Init() {
        this.edge = new int[20];
    }

    //Delete node with all connected edges
    public void delNode() {
        while (this.Edges > 0) {
            Mp_extsimp.delEdge(this.edge[0]);
        }
        this.Edges = 0;
        //'mark node as deleted
        this.nodeID = MARK_NODEID_DELETED;
    }

    //Calc distance from node3 to interval (not just line) from node1 to node2 in metres
    //Calc by Heron's formula from sides of triangle   (180/-180 safe)
    public static double distanceToSegment(Node node1, Node node2, Node node3) {
        double _rtn = 0;
        double a = 0;
        double b = 0;
        double c = 0;
        double s2 = 0;

        //'Calc squares of triangle sides
        a = distanceSquare(node1, node2);
        b = distanceSquare(node1, node3);
        c = distanceSquare(node2, node3);
        if (a == 0) {
            _rtn = Math.sqrt(b);
            //'node1=node2
            DistanceToSegment_last_case = 0;
            return _rtn;
        }
        else if (b > (a + c)) {
            _rtn = Math.sqrt(c);
            //'node1 is closest point to node3
            DistanceToSegment_last_case = 1;
            return _rtn;
        }
        else if (c > (a + b)) {
            _rtn = Math.sqrt(b);
            //'node2 is closest point to node3
            DistanceToSegment_last_case = 2;
            return _rtn;
        }
        else {
            //'Calc sides lengths from squares
            a = Math.sqrt(a);
            b = Math.sqrt(b);
            c = Math.sqrt(c);
            s2 = 0.5 * Math.sqrt((a + b + c) * (a + b - c) * (a + c - b) * (b + c - a));
            _rtn = s2 / a;
            //'closest point is inside interval
            DistanceToSegment_last_case = 3;
        }
        return _rtn;
    }

    //Calc distance square from node1 to node2 in metres
    //metric distance of ellipsoid chord (not arc)
    public static double distanceSquare(Node node1, Node node2) {
        //double x1, y1, z1;
        //double x2, y2, z2;
        XYZ first = new XYZ();
        XYZ second = new XYZ();
        first = XYZ.latLonToXYZ(node1.lat, node1.lon);
        second = XYZ.latLonToXYZ(node2.lat, node2.lon);
        return (first.x - second.x) * (first.x - second.x) + (first.y - second.y) * (first.y - second.y) + (first.z - second.z) * (first.z - second.z);
    }

}
