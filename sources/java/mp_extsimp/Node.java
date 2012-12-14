/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package mp_extsimp;

import java.util.ArrayList;
import java.util.Iterator;

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

    // Костыль, т.к. влом создавать псевдо клас для возврата двух значений, может оно потом и не пригодится
    // Есть проверка только на = 3
    public static int DistanceToSegment_last_case = 0;

    public double lat;      //Latitude
    public double lon;      //Longitude
    public int nodeID;     //NodeID from source .mp, -1 - not set, -2 - node killed
    public ArrayList<Edge> edgeL;   //all edges (values - indexes in Edges array)
    //public int Edges;      //number of edges, -1 means "not counted"
    public Node mark;       //internal marker for all-network algo-s
    public double temp_dist;//internal value for for all-network algo-s
    
    public Node() {
        Init();        
    }
    
    public Node(int nodeID) {
        this.nodeID = nodeID;
        Init();
    }
/*    
    public void addEdge(int Id) {
        this.edge[Edges++] = Id;
    }
*/    
    public void addEdge(Edge edgeIn) {
        this.edgeL.add(edgeIn);
    }
        
    
    private void Init() {
        this.edgeL = new ArrayList<Edge>();//new int[20];
    }

    //Delete node with all connected edges
    public void delNode() {
        while (this.edgeL.size() > 0) {
            //Mp_extsimp.delEdge(this.edgeL[0]);
            this.edgeL.get(0).delEdge();
        }
        //this.Edges = 0;
        //mark node as deleted
        this.nodeID = MARK_NODEID_DELETED;
    }

    //Calc distance from node3 to interval (not just line) from node1 to node2 in metres
    //Calc by Heron's formula from sides of triangle   (180/-180 safe)
    public static double distanceToSegment(Node node1, Node node2, Node node3) {
        double _rtn;
        double a;
        double b;
        double c;
        double s2;

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
        XYZ first;
        XYZ second;
        first = XYZ.latLonToXYZ(node1.lat, node1.lon);
        second = XYZ.latLonToXYZ(node2.lat, node2.lon);
        return (first.x - second.x) * (first.x - second.x) + (first.y - second.y) * (first.y - second.y) + (first.z - second.z) * (first.z - second.z);
    }

    //Calc distance from node1 to node2 in metres
    public static double distance(Node node1, Node node2) {
        return Math.sqrt(distanceSquare(node1, node2));
    }    

    //Merge node2 to node1 with relink of all edges
    public static void mergeNodes(Node node1, Node node2) {
        mergeNodes(node1, node2, 0);
    }
    
    //flag: 1 - ignore node2 coords (0 - move node1 to average coordinates)
    public static void mergeNodes(Node node1, Node node2, int flag) {
        int k, i;
        Edge edgeJ;

        //relink edges from node2 to node1
        //p = Nodes.get(node1).edgeL.size();
        //Node node1N = Nodes.get(node1);
        //Node node2N = Nodes.get(node2);
        k = node2.edgeL.size();
        
        for (i = 0; i <= k - 1; i++) {
            //j = node2N.edgeL[i];
            edgeJ = node2.edgeL.get(i);

            if (edgeJ.node1 == node2) {
                //edge goes from node2 to X
                edgeJ.node1 = node1;
            }
            if (edgeJ.node2 == node2) {
                //edge goes from X to node2
                edgeJ.node2 = node1;
            }
            //Nodes.get(node1).edgeL[p] = j;
            //node1N.edgeL.set(p, edgeJ);
            node1.edgeL.add(edgeJ);
            //p = p + 1;
        }
        //Nodes.get(node1).Edges = p;
        //Nodes.get(node2).Edges = 0;
        node2.edgeL.clear();

        //kill all void edges right now
        /*
        i = 0;
        p = node1.edgeL.size();
        while (i < p) {     //Nodes[node1].Edges) {
            //j = Nodes.get(node1).edgeL[i];
            edgeJ = node1.edgeL.get(i);
            if (edgeJ.node1 == edgeJ.node2) { edgeJ.delEdge(); }
            i = i + 1;
        }
        */
        for(Iterator<Edge> iEdge = node1.edgeL.iterator(); iEdge.hasNext();) {
            if (iEdge.next().node1 == iEdge.next().node2) { iEdge.next().delEdge(); }
        }

        if (flag == 1) {
            //Calc average coordinates
            //TODO: fix (not safe to 180/-180 edge)
            node1.lat = 0.5 * (node1.lat + node2.lat);
            node1.lon = 0.5 * (node1.lon + node2.lon);
        }

        //delNode(node2);
        node2.delNode();
    }

}
