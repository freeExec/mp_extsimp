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

    //Delete edge and remove all references to it from both nodes
    public void delEdge() { //Node node1) {
        
        //find this edge among edges of node1
        //i = this.node1; //Edges.get(edge1).node1;
        //edge already deleted        
        if (this.node1 == null) { return; }
        /*
        p = node1.edgeL.size();
        for (k = 0; k < p; k++) {
            if (node1.edgeL.get(k).equals(this)) {
                //remove edge from edges of node1
                //node1.edgeL[k] = node1.edgeL[node1.Edges - 1];                
                //node1.Edges --;
                node1.edgeL.remove(this);
                //*TODO:** goto found: GoTo lFound1;
                break;
            }
        }
        */
        //*TODO:** label found: lFound1:;
        //node1.edgeL.remove(this);
        this.node1.edgeL.remove(this);
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

}
