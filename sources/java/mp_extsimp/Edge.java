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
    public int node1;      //first node (index in Nodes array)
    public int node2;      //second node
    public byte roadtype;   //roadtype, see HIGHWAY_ consts
    public byte oneway;     //0 - no, 1 - yes ( goes from node1 to node2 )
    public int mark;    //internal marker for all-network algo-s
    public byte speed;      //speed class (in .mp terms)
    public String label;    //label of road (only ref= values currently, not name= )    

    public Edge() {
    }

    public Edge(int node1, int node2) {
        this.node1 = node1;
        this.node2 = node2;
    }

    //Delete edge and remove all references to it from both nodes
    public void delEdge(int edge1, Node node1, Node node2) {
        int i;
        int k;

        //find this edge among edges of node1
        i = this.node1; //Edges.get(edge1).node1;
        //edge already deleted
        if (i == -1) { return; }
        for (k = 0; k <= node1.Edges - 1; k++) {
            if (node1.edge[k] == edge1) {
                //remove edge from edges of node1
                node1.edge[k] = node1.edge[node1.Edges - 1];
                node1.Edges --;
                //*TODO:** goto found: GoTo lFound1;
                break;
            }
        }
        //*TODO:** label found: lFound1:;
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
}
