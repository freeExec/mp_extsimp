/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package mp_extsimp;

/**
 *
 * @author freeExec
 */
public class Edge {
    //public int node1;      //first node (index in Nodes array)
    //public int node2;      //second node
    public Node node1;
    public Node node2;
    public byte roadtype;   //roadtype, see HIGHWAY_ consts
    public byte oneway;     //0 - no, 1 - yes ( goes from node1 to node2 )
    public int mark;    //internal marker for all-network algo-s
    public byte speed;      //speed class (in .mp terms)
    public String label;    //label of road (only ref= values currently, not name= )    

    public Edge() {
    }

    public Edge(Node node1, Node node2) {
        this.node1 = node1;
        this.node2 = node2;
    }

    public Edge(Edge edge) {
        this.node1 = edge.node1;
        this.node2 = edge.node2;
        this.roadtype = edge.roadtype;
        this.oneway = edge.oneway;
        this.mark = edge.mark;
        this.speed = edge.speed;
        this.label = edge.label;
    }

    public String toString() {
        return String.format("Num1=%d, Num2=%d", this.node1.VBNum, this.node2.VBNum);
    }

    //Delete edge and remove all references to it from both nodes
    public void delEdge() { //Node node1) {
        
        //find this edge among edges of node1
        //edge already deleted        
        if (this.node1 == null) { return; }
        this.node1.edgeL.remove(this);
        this.node1 = null;
        this.node2.edgeL.remove(this);
        this.node2 = null;
    }

        //Get bounding box of edge
    public static Bbox getEdgeBbox(Node node1, Node node2) {
        Bbox result = new Bbox();
        result.lat_min = node1.lat;
        result.lat_max = node1.lat;
        result.lon_min = node1.lon;
        result.lon_max = node1.lon;
        if (result.lat_min > node2.lat) {
            result.lat_min = node2.lat;
        }
        if (result.lat_max < node2.lat) {
            result.lat_max = node2.lat;
        }
        if (result.lon_min > node2.lon) {
            result.lon_min = node2.lon;
        }
        if (result.lon_max < node2.lon) {
            result.lon_max = node2.lon;
        }
        return result;
    }

    //Join two nodes by new edge
    //node1 - start node, node2 - end node
    //return: index of new edge
    public static Edge joinByEdge(Node node1, Node node2) {
        //int _rtn = 0;
        //int k = 0;
        //Edges[EdgesNum].node1 = node1;
        //Edges[EdgesNum].node2 = node2;
        Edge result = new Edge(node1, node2);
        //Edges.add(result);
        
        //add edge to both nodes
        /*
        k = Nodes[node1].Edges;
        Nodes[node1].edge[k] = EdgesNum;
        Nodes[node1].Edges = Nodes[node1].Edges + 1;
        k = Nodes[node2].Edges;
        Nodes[node2].edge[k] = EdgesNum;
        Nodes[node2].Edges = Nodes[node2].Edges + 1;
        */
        node1.addEdge(result);
        node2.addEdge(result);
        
        //_rtn = EdgesNum;
        //addEdge();
        return result; //EdgesNum; //_rtn;
    }

    //Calc cosine of angle betweeen two edges
    //(calc via vectors on reference ellipsoid, 180/-180 safe)
    public static double cosAngleBetweenEdges(Edge edge1, Edge edge2) {
        double _rtn = 0;
        //double x1, y1, z1;
        //double x2, y2, z2;
        //double x3, y3, z3;
        //double x4, y4, z4;
        //int node1, node2;
        double len1, len2;

        XYZ first;
        XYZ second;
        XYZ third;
        XYZ fourth;

        //XYZ
        //node1 = edge1.node1;
        //.node2 = edge1.node2;
        first = XYZ.latLonToXYZ(edge1.node1.lat, edge1.node1.lon);
        second = XYZ.latLonToXYZ(edge1.node2.lat, edge1.node2.lon);
        //node1 = Edges[edge2].node1;
        //.node2 = Edges[edge2]..node2;
        third = XYZ.latLonToXYZ(edge2.node1.lat, edge2.node1.lon);
        fourth = XYZ.latLonToXYZ(edge2.node2.lat, edge2.node2.lon);

        //vectors
        //x2 = second.x - first.x;
        //y2 = second.y - first.y;
        //z2 = second.z - first.z;
        second.Sub(first);
        //x4 = x4 - x3;
        //y4 = y4 - y3;
        //z4 = z4 - z3;
        fourth.Sub(third);

        //vector lengths
        //len1 = Sqr(x2 * x2 + y2 * y2 + z2 * z2);
        len1 = Math.sqrt(second.x * second.x + second.y * second.y + second.z * second.z);
        //len2 = Sqr(x4 * x4 + y4 * y4 + z4 * z4);
        len2 = Math.sqrt(fourth.x * fourth.x + fourth.y * fourth.y + fourth.z * fourth.z);

        if (len1 == 0  || len2 == 0) {
            //one of vectors is void
            _rtn = 0;
        }
        else {
            //Cosine of angle is scalar multiply divided by lengths
            _rtn = (second.x * fourth.x + second.y * fourth.y + second.z * fourth.z) / (len1 * len2);
        }

        return _rtn;
    }
    
    //Calc distance between not crossing edges (edge1 and edge2)
    //(180/-180 safe)
    public static double distanceBetweenSegments(Edge edge1, Edge edge2) {
        double d1, d2;
        
        //Just minimum of 4 distances (each ends to each other edge)
        d1 = Node.distanceToSegment(edge1.node1, edge1.node2, edge2.node1);
        d2 = Node.distanceToSegment(edge1.node1, edge1.node2, edge2.node2);
        if (d2 < d1) { d1 = d2; }
        d2 = Node.distanceToSegment(edge2.node1, edge2.node2, edge1.node1);
        if (d2 < d1) { d1 = d2; }
        d2 = Node.distanceToSegment(edge2.node1, edge2.node2, edge1.node2);
        if (d2 < d1) { d1 = d2; }
        return d1;
    }
}
