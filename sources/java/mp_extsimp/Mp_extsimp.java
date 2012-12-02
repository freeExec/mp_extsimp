/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package mp_extsimp;

import java.io.*;
import java.util.*;
//import com.sun.corba.se.spi.orbutil.fsm.Input;
//import java.io.BufferedReader;
//import java.io.FileInputStream;
//import java.io.InputStreamReader;

/**
 *
 * @author freeExec
 */
public class Mp_extsimp {

    // Consts
    //Cluster size in degrees for ClusterIndex build and search
    public static final double CLUSTER_SIZE = 0.05;
    // Fields

    public static String MPheader = "";
    
    //All nodes
    //private static node[] Nodes;
    private static ArrayList<Node> Nodes;
    //private static int NodesAlloc = 0;
    //private static int NodesNum = 0;
    
    //All road edges
    //private static edge[] Edges;
    private static ArrayList<Edge> Edges;
    //private static int EdgesAlloc = 0;
    private static int EdgesNum = 0;
    
    //Array for building chain of indexes
    private static int[] Chain;
    private static int ChainAlloc = 0;
    private static int ChainNum = 0;
    
    //Aim edges
    private static aimedge[] AimEdges;
    private static int AimEdgesAlloc = 0;
    private static int AimEdgesNum = 0;    
    
    //Label statistics, for estimate labels of joined roads
    private static LabelStat[] LabelStats;
    private static int LabelStatsNum = 0;
    private static int LabelStatsAlloc = 0;

    //Indexes of forward and backward ways during two ways joining
    private static int[] TWforw;
    private static int[] TWback;
    private static int TWalloc = 0;
    private static int TWforwNum = 0;
    private static int TWbackNum = 0;

    //max found NodeID
    public static int NodeIDMax = 0;

    //'case of calc distance during last call of DistanceToSegment()
    public static int DistanceToSegment_last_case = 0;
    
//Cluster index
    //'min lat-border of clusters
    public static double ClustersLat0 = 0;
    //'min lon-border of clusters
    public static double ClustersLon0 = 0;
    //'index of first node of cluster (X*Y)
    public static int[] ClustersFirst;
    //'chain of nodes (NodesNum)
    public static int[] ClustersChain;
    //'num of cluster by lat = X
    public static int ClustersLatNum = 0;
    //'num of cluster by lon = Y
    public static int ClustersLonNum = 0;
    //'num of indexed nodes (for continuing BuildNodeClusterIndex)
    public static int ClustersIndexedNodes = 0;
    //'index of last node of cluster - for building index (X*Y)
    public static int[] ClustersLast;
    //last bbox
    public static Bbox ClustersFindLastBbox;
    //'last index of cluster
    public static int ClustersFindLastCluster = 0;
    //'
    public static int ClustersFindLastNode = 0;

    
    public static void main(String[] args) {
        optimizeRouting(args[0]);
    }
    
    private static void optimizeRouting(String inputFile) {
        String outFile = "";
        String outFile2 = "";
        long time1 = 0;

        //nothing to do
        if (inputFile.isEmpty()) { return; }

        //output file
        outFile = inputFile + "_opt.mp";
        outFile2 = inputFile + "_p.mp";  //output2 - for intermediate results
        
        //start measure time
        time1 = System.currentTimeMillis();
        
        //Init module (all arrays)
        InitArrays();

        //Load data from file
        load_MP(inputFile);

        //Join nodes by NodeID
        joinNodesByID();

        //Join two way roads into bidirectional ways
        joinDirections3(70, -0.996, -0.95, 100, 2);
        //70 metres between directions (Ex: Universitetskii pr, Moscow - 68m)
        //-0.996 -> (175, 180) degrees for start contradirectional check
        //-0.95 -> (161.8, 180) degrees for further contradirectional checks
        //100 metres min two way road
        //2 metres for joining nodes into one
    }

    //Init module (all arrays)
    private static void InitArrays() {
        //NodesAlloc = 1000;
        //Nodes = new node[NodesAlloc];
        Nodes = new ArrayList<Node>();
        //NodesNum = 0;   -> Nodes.size();

        //EdgesAlloc = 1000;
        //Edges = new edge[EdgesAlloc];
        Edges = new ArrayList<Edge>();
        //EdgesNum = 0;

        ChainAlloc = 1000;
        Chain = new int[ChainAlloc];
        ChainNum = 0;

        AimEdgesAlloc = 50;
        AimEdges = new aimedge[AimEdgesAlloc];
        AimEdgesNum = 0;

        LabelStatsAlloc = 30;
        LabelStats = new LabelStat[LabelStatsAlloc];
        LabelStatsNum = 0;

        TWalloc = 100;
        TWforw = new int[TWalloc];
        TWback = new int[TWalloc];
        TWbackNum = 0;
        TWforwNum = 0;     
    }

    //Load .mp file
    //(loader is basic and rather stupid, uses relocation on file to read section info without internal buffering)
    public static void load_MP(String filename) {
        //int logOptimization = 0;
        String sLine = "";
        double fLat = 0;
        double fLon = 0;
        //String sWay = "";
        //'0 - none, 1 - header, 2 - polyline, 3 - polygon
        int sectionType = 0;
        //'Phase of reading polyline: 0 - general part, 1 - scan for routeparam, 2 - scan for geometry, 3 - scan for routing (nodeid e.t.c)
        int iPhase = 0;
        long iStartLine = 0;
        long iPrevLine = 0;
        long fileLen = 0;
        int lastPercent = 0;
        //String sPrefix = "";
        int dataLineNum = 0;
        int k, k2, k3;
        long p;
        //int i, j;
        int thisLineNodes = 0;
        int nodeID = 0;
        byte wayClass = 0;
        byte waySpeed = 0;
        byte wayOneway = 0;
        String[] routep;
        byte lastCommentHighway = 0;
        String label = "";
        
        int originalLenLine = 0;

        //no nodeid yet
        NodeIDMax = -1;

        //Open(filename For Input As #1);
        //fileLen = LOF(1);
        BufferedReader br;
        FileInputStream fis;
        try {
            fis = new FileInputStream(filename);
            br = new BufferedReader(new InputStreamReader(fis, "CP1251"));
            
            fileLen = fis.getChannel().size();
            
            sectionType = 0;
            wayClass = -1;
            waySpeed = -1;
            iPhase = 0;
            label = "";
            //iStartLine = 0;
            iPrevLine = 0;
            MPheader = "";
            lastCommentHighway = Highway.HIGHWAY_UNSPECIFIED;

            while(br.ready()) {
    //*TODO:** label found: lNextLine:;
                //get current position in file            //iPrevLine = br.Seek(1);
                iPrevLine += originalLenLine; //fis.getChannel().position();
                //Line(Input #1, sLine);
                sLine = br.readLine();
                originalLenLine = sLine.length() + 2; // 2 = перевод строки
                sLine = sLine.trim();   // один раз отрезать все лишнее

                //check for section start
                if (sLine.startsWith("[IMG ID]")) {
                    //header section
                    sectionType = 1;
                    iPhase = 0;
                }
                if (sLine.startsWith("[POLYGON]")) {
                    //polygon
                    sectionType = 3;
                    //*TODO:** goto found: GoTo lStartPoly;
                }
                if (sLine.startsWith("[POLYLINE]")) {
                    //polyline
                    sectionType = 2;
                }
    //*TODO:** label found: lStartPoly:;
                if (sectionType == 3 || sectionType == 2) {
                    if ((iPrevLine / 1023) > lastPercent) {
                        //display progress
                        //Form1.Caption = "Load: " + CStr(iPrevLine) + " / " + CStr(fileLen): Form1.Refresh;
                        lastPercent = (int)(iPrevLine / 1023);
                        System.out.printf("Load: (%3$d%%) %1$d / %2$d\n", iPrevLine, fileLen, lastPercent);
                    }
                    dataLineNum = 0;
                    if (iPhase == 0) {
                        //first pass of section? start scanning
                        wayClass = -1;
                        waySpeed = -1;
                        iPhase = 1;
                        //remember current pos (where to go after ending pass)
                        iStartLine = iPrevLine - originalLenLine;
                        br.mark(40*1024);   // резерв буфера на 40 КБ
                        //System.out.printf("Mark: %1$d / %2$d\n", iPrevLine , originalLenLine);
                    }
                }

                if (sLine.startsWith("[END")) {
                    //section ended

                    if (sectionType == 1) {
                        //add ending of section into saved header
                        MPheader = MPheader + sLine + "\n";
                    }

                    //no routing params found in 1st pass - skip way completely
                    if (iPhase == 1  && (waySpeed == -1)) { iPhase = 0; }

                    if (iPhase > 0 && iPhase < 3) {
                        //not last pass of section -> goto start of it
                        //relocate in file

                        //Seek(1, iStartLine);
                        //fis.getChannel().position(iStartLine);
                        //System.out.printf("Reset: %1$d / %2$d\n", iPrevLine , originalLenLine);
                        iPrevLine = iStartLine;
                        br.reset();
                        iPhase = iPhase + 1;
    //*TODO:** goto found: GoTo lNextLine;
                        continue;
                    }

                    //if no osm2mp info yet found
                    lastCommentHighway = Highway.HIGHWAY_UNSPECIFIED;
                    label = "";
                    iPhase = 0;
                    sectionType = 0;
                }

                switch (iPhase) {
                    case  0:
                        if (sLine.startsWith("; highway")) {
                            //comment, produced by osm2mp
                            lastCommentHighway = Highway.getHighwayType(sLine.substring(12).trim());
                        }
                        if (sectionType == 1) {
                            //line of header section
                            MPheader = MPheader + sLine + "\n";
                        }
                        break;
                    //scan for routing param
                    case  1:
                        if (sLine.startsWith("RouteParam")) {
                            //'skip ext
                            if (sLine.startsWith("RouteParamExt")) { break; } //{ goto: GoTo lNoData; // skipp ext}
                            k2 = sLine.indexOf("=") + 1;
                            //split by "," delimiter
                            routep = sLine.substring(k2).split(",");
                            //direct copy of speed
                            waySpeed = Byte.parseByte(routep[0]);
                            //and oneway
                            wayOneway = Byte.parseByte(routep[2]);
                            if (lastCommentHighway == Highway.HIGHWAY_UNSPECIFIED) {
                                //default class
                                wayClass = 3;
                                //TODO: should be detected by Type and WayClass
                            }
                            else {
                                //get class from osm2mp comment
                                wayClass = lastCommentHighway;
                            }
                        }
                        if (sLine.startsWith("Label")) {
                            //label
                            k2 = sLine.indexOf("=") + 1;
                            label = sLine.substring(k2).trim();
                            if (label.startsWith("~[0x")) {
                                //use only special codes
                                k2 = label.indexOf("]") + 1;
                                label = label.substring(k2).trim();
                            }
                            else {
                                //ignore others
                                label = "";
                            }

                        }
                        break;
                    case  3:
                        //scan for node routing info:
                        if (sLine.startsWith("Nod")) {
                            //Nod

                            k2 = sLine.indexOf("=") + 1;
                            if (k2 <= 0) { break; } //*TODO:** goto found: GoTo lSkipRoadNode; }
                            routep = sLine.substring(k2).split(",");
                            k = Integer.parseInt(routep[0]);

                            if (k > Nodes.size()) { //NodesAlloc) {
                                //error: too big node index: " + sLine
                                //*TODO:** goto found: GoTo lSkipRoadNode;
                                break;
                            }

                            //k3 = sLine.indexOf(",", k2);
                            //if (k3 < 0) {
                            if (routep.length < 2) {
                                //error: bad NodeID
                                //*TODO:** goto found: GoTo lSkipRoadNode;
                                break;
                            }
                            nodeID = Integer.parseInt(routep[1]);

                            //update max NodeID
                            if (nodeID > NodeIDMax) { NodeIDMax = nodeID; }

                            //store nodeid
                            //Nodes[NodesNum - thisLineNodes + k].nodeID = nodeID;
                            Nodes.add(new Node(nodeID));
                            
//*TODO:** label found: lSkipRoadNode:;
                        }
                        break;
                    case  2:
                        if (sLine.startsWith("Data")) {
                            //geometry
                            // вроде как дубль пред. условия - выкинуть
                            //k = sLine.indexOf("Data");
                            //if (k < 0) { break; } //*TODO:** goto found: GoTo lNoData; }

                            //sWay = "";
                            //"Data" + next char + "="
                            //sPrefix = sLine.substring(0, k + 4) + "=";

                            dataLineNum = dataLineNum + 1;

                            thisLineNodes = 0;

//*TODO:** label found: lNextPoint:;
                            k3 = 0;
                            while (true) {
                                Node addedNode = new Node(-1);
                                //get lat-lon coords from line
                                k = sLine.indexOf("(", k3);//k.toLowerCase().indexOf(sLine.toLowerCase());
                                k2 = sLine.indexOf(",", k);
                                k3 = sLine.indexOf(")", k2);
                                if (k < 0 || k2 < 0 || k3 < 0) { break; } //*TODO:** goto found: GoTo lEndData; }
                                fLat = Double.parseDouble(sLine.substring(k + 1, k2));

                                //k3 = sLine.indexOf(",", k2);
                                //if (k3 < 0) { break; }//*TODO:** goto found: GoTo lEndData; }
                                fLon = Double.parseDouble(sLine.substring(k2 + 1, k3));

                                //fill node info
                                /*
                                Nodes[NodesNum].lat = fLat;
                                Nodes[NodesNum].lon = fLon;
                                //Nodes[NodesNum].Edges = 0;
                                Nodes[NodesNum].nodeID = -1;*/
                                addedNode.lat = fLat;
                                addedNode.lon = fLon;
                                //addedNode.nodeID = -1;
                                Nodes.add(addedNode);

                                if (thisLineNodes > 0) {
                                    //not the first node of way -> create edge
                                    Edge jEdge = joinByEdge(Nodes.size() - 2, Nodes.size() - 1);
                                    //oneway edges is always -> by geometry
                                    /*Edges[j].oneway = wayOneway;
                                    Edges[j].roadtype = wayClass;
                                    Edges[j].label = label;*/
                                    jEdge.oneway = wayOneway;
                                    jEdge.roadtype = wayClass;
                                    jEdge.label = label;
                                    if (waySpeed >= 0) {
                                        //Edges[j].speed = waySpeed;
                                        jEdge.speed = waySpeed;
                                    }
                                    else {
                                        //were not specified
                                        //56km/h
                                        //Edges[j].speed = 3;
                                        jEdge.speed = 3;
                                    }
                                }
                                thisLineNodes ++;

                                //'finish node creation
                                //addNode();

                                //k = k3;
                                //*TODO:** goto found: GoTo lNextPoint;
                            }
//*TODO:** label found: lEndData:;

                            p = 0;

//*TODO:** label found: lNoData:;
                        }
                        break;
                }
                //if (!EOF(1)) { //*TODO:** goto found: GoTo lNextLine; }
            }
            //Close(#1);
            br.close();
        } catch (FileNotFoundException ex) {
            System.err.println(ex.getLocalizedMessage() + " : " + filename);
        } catch (IOException ex) {
            System.err.println(ex.getLocalizedMessage());
        }

    }
   
    //Join two nodes by new edge
    //node1 - start node, node2 - end node
    //return: index of new edge
    public static Edge joinByEdge(int node1, int node2) {
        //int _rtn = 0;
        //int k = 0;
        //Edges[EdgesNum].node1 = node1;
        //Edges[EdgesNum].node2 = node2;
        Edge result = new Edge(node1, node2);
        Edges.add(result);
        
        //add edge to both nodes
        /*
        k = Nodes[node1].Edges;
        Nodes[node1].edge[k] = EdgesNum;
        Nodes[node1].Edges = Nodes[node1].Edges + 1;
        k = Nodes[node2].Edges;
        Nodes[node2].edge[k] = EdgesNum;
        Nodes[node2].Edges = Nodes[node2].Edges + 1;
        */
        Nodes.get(node1).addEdge(Edges.size());
        Nodes.get(node2).addEdge(Edges.size());
        
        //_rtn = EdgesNum;
        //addEdge();
        return result; //EdgesNum; //_rtn;
    }
    
    /*
    // Использую списки, провке размера не нужна
    //Add one edge to dynamic array
    //Assumed, Edges(EdgesNum) filled with required data prior to call
    public static void addEdge() {
        if (EdgesNum >= EdgesAlloc) {
            //realloc if needed
            EdgesAlloc = EdgesAlloc * 2;
            G.redimPreserve(Edges, EdgesAlloc);
        }
        EdgesNum = EdgesNum + 1;
    } */

    //Merge loaded nodes from diffrent ways by NodeID
    private static void joinNodesByID() {
        int i, j;
        int k = 0;
        int mapNum = 0;
        int[] iDmap;
        int[] nodeMap;

        int NodesNum = Nodes.size();

        //if NodeID indexes are too big, we could not use direct mapping
        //max number for direct mapping should be selected with respect to available RAM
        //need more than 40M
        if (NodeIDMax < 10000000L) {

            //SOFT WAY, via direct map from NodeID  (~ O(n) )

            //IDmap(NodeID) = index in Nodes array
            //G.redim(iDmap, NodeIDMax);
            iDmap = new int[NodeIDMax+1];

            for (i = 0; i < NodeIDMax+1; i++) {  // <= ?
                iDmap[i] = -1;
            }

            for (i = 0; i < NodesNum ; i++) {   // <= - 1?
                k = Nodes.get(i).nodeID;

                //without NodeID - not mergable
                if (k >= 0) { //if (k < 0) *TODO:** goto found: GoTo lSkip; }

                    if (iDmap[k] < 0) {
                        //first occurence of NodeID
                        iDmap[k] = i;
                    }
                    else {
                        //hould join
                        mergeNodes(iDmap[k], i);
                    }
                }
//*TODO:** label found: lSkip:;

                if ((i % 80) == 0) {  //8191
                    //display progress
                    //Form1.Caption = "Join soft " + CStr(i) + " / " + CStr(NodesNum): Form1.Refresh;
                    System.out.printf("Join soft %1$d / %2$d\n", i, NodesNum);
                }

            }
        //*TODO:** goto found: GoTo lExit;

//*TODO:** label found: lHardWay:;
        } else {
            //HARD WAY, via bubble search (~ O(n^2))

            // IDmap(a) = NodeID
            //G.redim(iDmap, NodesNum);
            iDmap = new int[NodesNum];
            // NodeMap(a) = index of node in Nodes() array
            //G.redim(nodeMap, NodesNum);
            nodeMap = new int[NodesNum];
            mapNum = 0;

            for (i = 0; i <= NodesNum - 1; i++) {
                k = Nodes.get(i).nodeID;
                if (k >= 0) {
                    for (j = 0; j <= i - 1; j++) {
                        if (iDmap[j] == k) {
                            //found - not first occurence - should join
                            mergeNodes(nodeMap[j], i);
                            //*TODO:** goto found: GoTo lFound;
                            break;
                        }
                    }
                    if (j > i - 1) {        // добавли из-за брейка в цикле
                        //not found - first occurence of NodeID
                        nodeMap[mapNum] = i;
                        iDmap[mapNum] = k;
                        mapNum = mapNum + 1;
                    }
                }
//*TODO:** label found: lFound:;

                if ((i & 8191) == 0) {
                    //display progress
                    //Form1.Caption = "Join soft " + CStr(i) + " / " + CStr(NodesNum): Form1.Refresh;
                    System.out.printf("Join hard %1$d / %2$d\n", i, NodesNum);
                }
            }
        }
//*TODO:** label found: lExit:;
    }

    //Merge node2 to node1 with relink of all edges
    private static void mergeNodes(int node1, int node2) {
        mergeNodes(node1, node2, 0);
    }
    //flag: 1 - ignore node2 coords (0 - move node1 to average coordinates)
    private static void mergeNodes(int node1, int node2, int flag) {
        int k, i, j;
        int p = 0;

        //relink edges from node2 to node1
        p = Nodes.get(node1).Edges;
        k = Nodes.get(node2).Edges;
        for (i = 0; i <= k - 1; i++) {
            j = Nodes.get(node2).edge[i];

            if (Edges.get(j).node1 == node2) {
                //edge goes from node2 to X
                Edges.get(j).node1 = node1;
            }
            if (Edges.get(j).node2 == node2) {
                //edge goes from X to node2
                Edges.get(j).node2 = node1;
            }
            Nodes.get(node1).edge[p] = j;
            p = p + 1;
        }
        Nodes.get(node1).Edges = p;
        Nodes.get(node2).Edges = 0;

        //kill all void edges right now
        i = 0;
        while (i < p) {     //Nodes[node1].Edges) {
            j = Nodes.get(node1).edge[i];
            if (Edges.get(j).node1 == Edges.get(j).node2) { delEdge(j); }
            i = i + 1;
        }

        if (flag == 1) {
            //Calc average coordinates
            //TODO: fix (not safe to 180/-180 edge)
            Nodes.get(node1).lat = 0.5 * (Nodes.get(node1).lat + Nodes.get(node2).lat);
            Nodes.get(node1).lon = 0.5 * (Nodes.get(node1).lon + Nodes.get(node2).lon);
        }

        //delNode(node2);
        Nodes.get(node2).delNode();
    }

    //Delete edge and remove all references to it from both nodes
    public static void delEdge(int edge1) {
        //find this edge among edges of node1

        Edge dEdge = Edges.get(edge1);
        dEdge.delEdge(edge1, Nodes.get(dEdge.node1), Nodes.get(dEdge.node2));
/*
        //find this edge among edges of node2
        i = Edges[edge1]..node2;
        for (k = 0; k <= Nodes[i].Edges - 1; k++) {
            if (Nodes[i]..edge(k) == edge1) {
                //remove edge from edges of node2
                Nodes[i]..edge(k) = Nodes[i]..edge(Nodes[i].Edges - 1);
                Nodes[i].Edges = Nodes[i].Edges - 1;
                //*TODO:** goto found: GoTo lFound2;
            }
        }
        //*TODO:** label found: lFound2:;
        //'mark node as deleted
        Edges[edge1]..node1 = -1;
        */
    }

    //Join two directions of road way
    //MaxCosine - cosine of max angle between start edges, -0.996 means (175,180) degrees - contradirectional edges or close
    //MaxCosine2 - cosine of max angle between other edges, during going-by-two-ways
    //MinChainLen - length of min two-way road to join
    public static void joinDirections3(double joinDistance, double maxCosine, double maxCosine2, double minChainLen, double combineDistance) {
        int i,  j, k, mode1;
        int e, d, q;
        double dist1, dist2;
        Bbox bbox_edge;
        double angl = 0;
        double min_dist;
        int min_dist_edge;
        int roadtype = 0;
        int speednew = 0;

        //'chain of forward edges
        int[] edgesForw;
        //'chain of backward edges
        int[] edgesBack;
        //'1 if road is circled
        int loopChain = 0;
        //'len of half of road
        int halfChain = 0;

        //Algorithm will check all non-link oneway edges for presence of contradirectional edge in the vicinity
        //All found pairs of edges will be checked in both directions by GoByTwoWays function
        //for presence of continuous road of one type
        //During this check will be created new chain of nodes, which is projection of joining nodes into middle line
        //Then both found ways will be joined into one bidirectional way, consist from new nodes
        //All related roads will reconnected to new way and old edges were deleted

        int NodesNum = Nodes.size();
        //mark all nodes as not checked
        for (i = 0; i <= NodesNum - 1; i++) {
            //'not moved
            Nodes.get(i).mark = -1;
        }
        int EdgesNum = Edges.size();
        for (i = 0; i <= EdgesNum - 1; i++) {
            Edges.get(i).mark = 1;
            
            //'skip deleted and 2-ways edges
            if (Edges.get(i).node1 == -1 || Edges.get(i).oneway == 0) {
                //*TODO:** goto found: GoTo lFinMarkEdge;
                continue;
            }
            //'skip links
            if ((Edges.get(i).roadtype & Highway.HIGHWAY_MASK_LINK) > 0) {
                //*TODO:** goto found: GoTo lFinMarkEdge;
                continue;
            }
            //'skip edges between complex connections
            if (Nodes.get(Edges.get(i).node1).Edges != 2  && Nodes.get(Edges.get(i).node2).Edges != 2) { 
                //*TODO:** goto found: GoTo lFinMarkEdge;
                continue;
            }
            Edges.get(i).mark = 0;
//*TODO:** label found: lFinMarkEdge:;
        }

        //rebuild cluster-index from 0
        buildNodeClusterIndex(0);

        for (i = 0; i <= EdgesNum - 1; i++) {
            //'skip marked edge or deleted
            if (Edges.get(i).mark > 0 || Edges.get(i).node1 < 0) {
            //*TODO:** goto found: GoTo lSkipEdge;
            }
            //'get bbox
            //bbox_edge = getEdgeBbox(i);
            bbox_edge = Edge.getEdgeBbox(Nodes.get(Edges.get(i).node1), Nodes.get(Edges.get(i).node2));
            //'expand it
            Bbox.expandBbox(bbox_edge, joinDistance);
            min_dist = joinDistance;
            min_dist_edge = -1;

            //'first
            mode1 = 0;

            while (true) {
//*TODO:** label found: lSkipNode2:;
                k = getNodeInBboxByCluster(bbox_edge, mode1);
                //'next (next time)
                mode1 = 1;
                //'no more nodes
                if (k == -1) {
                    //*TODO:** goto found: GoTo lAllNodes;
                    break;
                }

                //'skip nodes of same edge, deleted and complex nodes
                if (k == Edges.get(i).node1  || k == Edges.get(i).node2  || Nodes.get(k).nodeID == Mark.MARK_NODEID_DELETED  || Nodes.get(k).Edges != 2) {
                    //*TODO:** goto found: GoTo lSkipNode2;
                    continue;
                }

                //'calc dist from found node to our edge
                dist1 = Node.distanceToSegment(Nodes.get(Edges.get(i).node1), Nodes.get(Edges.get(i).node2), Nodes.get(k));
                DistanceToSegment_last_case = Node.DistanceToSegment_last_case;
                //'too far, skip
                if (dist1 > min_dist) {
                    //*TODO:** goto found: GoTo lSkipNode2;
                    continue;
                }

                //node is on join distance, check all (2) edges
                for (d = 0; d <= 1; d++) {
                    q = Nodes.get(k).edge[d];
                    //'deleted or 2-way edge or other road class
                    if (Edges.get(q).node1 == -1 || Edges.get(q).oneway == 0 || Edges.get(q).roadtype != Edges.get(i).roadtype) {
                    //*TODO:** goto found: GoTo lSkipEdge2;
                        continue;
                    }
                    angl = Node.cosAngleBetweenEdges(Nodes.get(Edges.get(q).node1), Nodes.get(Edges.get(q).node2),
                            Nodes.get(Edges.get(i).node1), Nodes.get(Edges.get(i).node2));
                    if (angl < maxCosine) {
                        //contradirectional edge or close

                        dist1 = Node.distanceBetweenSegments(Nodes.get(Edges.get(i).node1), Nodes.get(Edges.get(i).node2),
                                Nodes.get(Edges.get(q).node1), Nodes.get(Edges.get(q).node2));
                        //'found edge close enough
                        if (dist1 < min_dist) {
                            min_dist = dist1;
                            min_dist_edge = q;
                        }
                    }
    //*TODO:** label found: lSkipEdge2:;
                }
            }

            //*TODO:** goto found: GoTo lSkipNode2:;

//*TODO:** label found: lAllNodes:;
            //all nodes in bbox check

            if (min_dist_edge > -1) {
                //found edge close enough
                //now - trace two ways in both directions
                //in the process we will fill Chain array with all nodes of joining ways
                //sequence of nodes in Chain will correspond to sequence of nodes on combined way
                //index of new node, where old node should join, will in .mark field of old nodes
                //also will be created two lists of deleting edges separated to two directions - arrays TWforw and TWback

                roadtype = Edges.get(i).roadtype;
                loopChain = 0;
                ChainNum = 0;
                TWforwNum = 0;
                TWbackNum = 0;

                //first pass, in direction of edge i
                goByTwoWays(i, min_dist_edge, joinDistance, combineDistance, maxCosine2, 0);

                //reverse of TWforw and TWback removed, as order of edges have no major effect

                //reverse Chain
                reverseArray(Chain, ChainNum);

                //second pass, in direction of min_dist_edge
                goByTwoWays(min_dist_edge, i, joinDistance, combineDistance, maxCosine2, 1);

                //'first and last nodes coincide - this is loop road
                if (Chain[0] == Chain[ChainNum - 1]) { loopChain = 1; }

                //'half len of road in nodes
                halfChain = ChainNum / 2;
                //'will "kill" halfchain limit for very short loops
                if (halfChain < 10) { halfChain = ChainNum + 1; }

                //call metric length of found road
                dist1 = 0;
                for (j = 1; j <= ChainNum - 1; j++) {
                    dist1 = dist1 + distance(Nodes.get(Chain[j - 1]).mark, Nodes.get(Chain[j]).mark);
                }

                if (dist1 < minChainLen) {
                    //road is too short -> unmark all edges and not delete anything
                    for (j = 0; j <= ChainNum - 1; j++) {
                        for (k = 0; k <= Nodes.get(Chain[j]).Edges - 1; k++) {
                            if (Edges.get(Nodes.get(Chain[j]).edge[k]).mark == 2) {
                                Edges.get(Nodes.get(Chain[j]).edge[k]).mark = 1;
                            }
                        }
                    }
                    //*TODO:** goto found: GoTo lSkipDel;
                }


                //process both directions edges list
                //to build index of edges, which joins between each pair of nodes in Chain

                //note: is some rare cases chain of nodes have pleats, where nodes of one directions
                //in Chain swaps position due to non uniform projecting of nodes to middle-line
                //In this cases one or more edges joins to bidirectional road in backward direction
                //to the original direction of this one-way line
                //These edges could be ignored during combining parameter of bidirectional road
                //(as they are usually very short)
                //also at least two other edges will overlap in at least one interval between nodes
                //only one of them will be counted during combining parameter (last in TW* array)
                //this is considired acceptable, as they are near edges of very same road

                //G.redim(edgesForw, ChainNum);
                edgesForw = new int[ChainNum];
                //G.redim(edgesBack, ChainNum);
                edgesBack = new int[ChainNum];
                for (j = 0; j <= ChainNum - 1; j++) {
                    edgesForw[j] = -1;
                    edgesBack[j] = -1;
                }

                //process forward direction
                for (j = 0; j <= TWforwNum - 1; j++) {
                    e = Edges.get(TWforw[j]).node1;
                    d = Edges.get(TWforw[j]).node2;
                    //get indexes of nodes inside Chain
                    e = findInChain(e);
                    d = findInChain(d);
                    if (e == -1 || d == -1) {
                        //(should not happen)
                        //edge with nodes not in chain - skip
                        //*TODO:** goto found: GoTo lSkip1;
                        continue;
                    }

                    if (e < d) {
                        //normal forward edge (or pleat crossing 0 of chain)
                        // ... e ---> d .....
                        //skip too long edges on loop chains as it could be wrong (i.e. pleat edge which cross 0 of chain)
                        if (loopChain == 1  && (d - e) > halfChain) { 
                        //*TODO:** label found: //*TODO:** goto found: GoTo lSkip1:;
                        }
                        for (q = e; q <= d - 1; q++) {
                            //in forward direction between q and q+1 node is edge TWforw(j)
                            edgesForw[q] = TWforw[j];
                        }
                    }
                    else {
                        //pleat edge (or normal crossing 0 of chain)
                        // ---.---> d .... ... .... e --->
                        //'on straight chains forward edge could not go backward without pleat
                        if (loopChain == 0) {
                            //*TODO:** goto found: GoTo lSkip1;
                            continue;
                        }
                        if ((e - d) > halfChain) {
                            //e and d is close to ends of chain
                            //-> this is really forward edge crossing 0 of chain in a loop road
                            for (q = 0; q <= d - 1; q++) {
                                edgesForw[q] = TWforw[j];
                            }
                            for (q = e; q <= ChainNum - 1; q++) {
                                edgesForw[q] = TWforw[j];
                            }
                        }
                    }
//*TODO:** label found: lSkip1:;
                }

                //process backward direction
                for (j = 0; j <= TWbackNum - 1; j++) {
                    e = Edges.get(TWback[j]).node1;
                    d = Edges.get(TWback[j]).node2;
                    //'get indexes of nodes inside Chain
                    e = findInChain(e);
                    d = findInChain(d);
                    if (e == -1  || d == -1) {
                        //(should not happen)
                        //edge with nodes not in chain - skip
                        //*TODO:** goto found: GoTo lSkip2;
                        continue;
                    }

                    if (d < e) {
                        //normal backward edge (or pleat crossing 0 of chain)
                        // ... d <--- e .....
                        //'skip too long edges on loop chains as it could be wrong (i.e. pleat edge which cross 0 of chain)
                        if (loopChain == 1  && (e - d) > halfChain) {
                            //*TODO:** label found: //*TODO:** goto found: GoTo lSkip2:;
                            continue;
                        }
                        for (q = d; q <= e - 1; q++) {
                            edgesBack[q] = TWback[j];
                        }
                    }
                    else {
                        //pleat edge (or normal crossing 0 of chain)
                        // <-.-- e ... ... .... ... d <--.---.---
                        //'on straight chains backward edge could not go forward without pleat
                        if (loopChain == 0) {
                            //*TODO:** goto found: GoTo lSkip2;
                            continue;
                        }
                        if ((d - e) > halfChain) {
                            //e and d is close to ends of chain
                            //-> this is really backward edge crossing 0 of chain in a loop road
                            for (q = 0; q <= e - 1; q++) {
                                edgesBack[q] = TWback[j];
                            }
                            for (q = d; q <= ChainNum - 1; q++) {
                                edgesBack[q] = TWback[j];
                            }
                        }
                    }
//*TODO:** label found: lSkip2:;
                }

                for (j = 1; j <= ChainNum - 1; j++) {
                    d = Nodes.get(Chain[j - 1]).mark;
                    e = Nodes.get(Chain[j]).mark;
                    if (d != e) {
                        /*k = joinByEdge(Nodes.get(Chain[j - 1]).mark, Nodes.get(Chain[j]).mark);
                        Edges.get(k).roadtype = roadtype;
                        Edges.get(k).oneway = 0;
                        Edges.get(k).mark = 1;
                        */
                        Edge edg = joinByEdge(Nodes.get(Chain[j - 1]).mark, Nodes.get(Chain[j]).mark);
                        // TODO: не будет ли потери при байте
                        edg.roadtype = (byte)roadtype;
                        edg.oneway = 0;
                        edg.mark = 1;

                        if ((edgesForw[j - 1] == -1) && (edgesBack[j - 1] == -1)) {
                            //no edges for this interval between nodes
                            //(should never happens)
                            //'default value
                            /*Edges.get(k).speed = 3;
                            Edges.get(k).label = "";*/
                            edg.speed = 3;
                            edg.label = "";
                        }
                        else {
                            //get minimal speed class of both edges
                            speednew = 10;
                            resetLabelStats();
                            if (edgesForw[j - 1] != -1) {
                                //forward edge present
                                speednew = Edges.get(edgesForw[j - 1]).speed;
                                addLabelStat0(Edges.get(edgesForw[j - 1]).label);
                            }
                            if (edgesBack[j - 1] != -1) {
                                //backward edge present
                                if (speednew > Edges.get(edgesBack[j - 1]).speed) { speednew = Edges.get(edgesBack[j - 1]).speed; }
                                addLabelStat0(Edges.get(edgesBack[j - 1]).label);
                            }
                            /*Edges.get(k).speed = speednew;
                            Edges.get(k).label = getLabelByStats(0);*/
                            // TODO (byte)
                            edg.speed = (byte)speednew;
                            edg.label = getLabelByStats(0);
                        }


                        //ends of chain could be oneway if only one edge (or even part is joining there
                        //ex:     * ------> * --------> * ----------> *
                        //             * <-------- * <--------- * <---------- *
                        //joins into:
                        //        *--->*----*------*----*-------*-----*<------*

                        if (edgesBack[j - 1] == -1) {
                            //no backward edge - result in one-way
                            //Edges.get(k).oneway = 1;
                            edg.oneway = 1;
                        }
                        else if (edgesForw[j - 1] == -1) {
                            //no forward edge - result in one-way, backward to other road
                            /*Edges.get(k).oneway = 1;
                            Edges.get(k).node1 = Nodes[Chain[j]]..mark;
                            Edges.get(k).node2 = Nodes[Chain[j - 1]]..mark;*/
                            edg.oneway = 1;
                            edg.node1 = Nodes.get(Chain[j]).mark;
                            edg.node1 = Nodes.get(Chain[j - 1]).mark;
                        }
                    }
                }

                //delete all old edges
                for (j = 0; j <= TWforwNum - 1; j++) {
                    delEdge(TWforw[j]);
                }
                for (j = 0; j <= TWbackNum - 1; j++) {
                    delEdge(TWback[j]);
                }

                //merge all old nodes into new ones
                for (j = 0; j <= ChainNum - 1; j++) {
                    mergeNodes(Nodes.get(Chain[j]).mark, Chain[j], 1);
                }

                //update cluster index to include only newly created nodes (i.e. nodes of joined road)
                buildNodeClusterIndex(1);

//*TODO:** label found: lSkipDel:;
            }

            //'mark edge as checked
            Edges.get(i).mark = 1;

//*TODO:** label found: lSkipEdge:;

            if ((i & 8191) == 0) {
                //show progress
                //Form1.Caption = "JD3: " + CStr(i) + " / " + CStr(EdgesNum): Form1.Refresh;
                System.out.printf("JDS: %1$d / %2$d", i, EdgesNum);
            }
        }


    }


   //Build cluster index
    //Cluster index allow to quickly find nodes in specific bbox
    //Cluster index is collections of nodes chains, where starts can be selected from coordinates
    //and continuation - by indexes in chains
    //Flags: 1 - only update from ClustersIndexedNodes to NodesNum (0 - full re/build)
    public static void buildNodeClusterIndex(int flags) {
        Long, i = null; j As Long, k As Long
        int x = 0;
        int y = 0;

        if ((flags && 1) != 0) {
            //Only update
            //TODO(?): remove chain from deleted nodes
            G.redimPreserve(ClustersChain, NodesNum);
            //*TODO:** goto found: GoTo lClustering;
        }

        //calc overall bbox
        Bbox wholeBbox = null;
        wholeBbox..lat_max = -360;
        wholeBbox..lat_min = 360;
        wholeBbox..lon_max = -360;
        wholeBbox..lon_min = 360;
        for (i = 0; i <= NodesNum - 1; i++) {
            //'skip deleted nodes
            if (Nodes.get(i).nodeID != MARK_NODEID_DELETED) {
                if (Nodes.get(i).lat < wholeBbox..lat_min) { wholeBbox..lat_min = Nodes.get(i).lat; }
                if (Nodes.get(i).lat > wholeBbox..lat_max) { wholeBbox..lat_max = Nodes.get(i).lat; }
                if (Nodes.get(i).lon < wholeBbox..lon_min) { wholeBbox..lon_min = Nodes.get(i).lon; }
                if (Nodes.get(i).lon > wholeBbox..lon_max) { wholeBbox..lon_max = Nodes.get(i).lon; }
            }
        }

        ClustersIndexedNodes = 0;
        //'no nodes at all or something wrong
        if (wholeBbox..lat_max < wholeBbox..lat_min || wholeBbox..lon_max < wholeBbox..lon_min) { return; }

        //calc number of clusters
        ClustersLatNum = 1 + (wholeBbox..lat_max - wholeBbox..lat_min) / CLUSTER_SIZE;
        ClustersLonNum = 1 + (wholeBbox..lon_max - wholeBbox..lon_min) / CLUSTER_SIZE;

        //'starts of chains
        G.redim(ClustersFirst, ClustersLatNum * ClustersLonNum);
        //'ends of chains (for updating)
        G.redim(ClustersLast, ClustersLatNum * ClustersLonNum);
        //'whole chain
        G.redim(ClustersChain, NodesNum);

        //'edge of overall bbox
        ClustersLat0 = wholeBbox..lat_min;
        ClustersLon0 = wholeBbox..lon_min;

        for (i = 0; i <= ClustersLatNum * ClustersLonNum - 1; i++) {
            //'no nodes in cluster yet
            ClustersFirst[i] = -1;
            ClustersLast[i] = -1;
        }

        ClustersIndexedNodes = 0;

        //*TODO:** label found: lClustering:;
        for (i = ClustersIndexedNodes; i <= NodesNum - 1; i++) {
            if (Nodes.get(i).nodeID != MARK_NODEID_DELETED) {
                //get cluster from lat/lon
                x = (Nodes.get(i).lat - ClustersLat0) / CLUSTER_SIZE;
                y = (Nodes.get(i).lon - ClustersLon0) / CLUSTER_SIZE;
                j = x + y * ClustersLatNum;

                k = ClustersLast[j];
                if (k == -1) {
                    //first index in chain of this cluster
                    ClustersFirst[j] = i;
                }
                else {
                    //continuing chain
                    ClustersChain[k] = i;
                }
                //'this is last node in chain
                ClustersChain[i] = -1;
                ClustersLast[j] = i;
            }
        }

        //'last node in cluster index
        ClustersIndexedNodes = NodesNum;

    }

    //Find node in bbox by using cluster index
    //Flags: 1 - next (0 - first)
    public static int getNodeInBboxByCluster(Bbox box1, int flags) {
        int _rtn = 0;
        int i, j, k;
        int x, y;
        int x1, x2, y1, y2;

        if ((flags & 1) == 0) {
            //first node needed

            //get coordinates of all needed clusters
            x1 = (int)((box1.lat_min - ClustersLat0) / CLUSTER_SIZE);
            x2 = (int)((box1.lat_max - ClustersLat0) / CLUSTER_SIZE);
            y1 = (int)((box1.lon_min - ClustersLon0) / CLUSTER_SIZE);
            y2 = (int)((box1.lon_max - ClustersLon0) / CLUSTER_SIZE);

            //'store bbox for next searches
            ClustersFindLastBbox = box1;
            x = x1;
            y = y1;
            //*TODO:** goto found: GoTo lCheckFirst;
        } else {

            if (ClustersFindLastNode == -1) {
                //Last time nothing found - nothing to do further
                _rtn = -1;
                //*TODO:** goto found: GoTo lExit;
            }

            //get coordinates of all needed clusters
            x1 = (int)((ClustersFindLastBbox.lat_min - ClustersLat0) / CLUSTER_SIZE);
            x2 = (int)((ClustersFindLastBbox.lat_max - ClustersLat0) / CLUSTER_SIZE);
            y1 = (int)((ClustersFindLastBbox.lon_min - ClustersLon0) / CLUSTER_SIZE);
            y2 = (int)((ClustersFindLastBbox.lon_max - ClustersLon0) / CLUSTER_SIZE);

            //get coordinates of last used cluster
            x = ClustersFindLastCluster % ClustersLatNum;   // Mod
            y = (int)((ClustersFindLastCluster - x) / ClustersLatNum);  // было целое деление или как-то так "\"

//*TODO:** label found: lNextNode:;
            //'get node from chain
            //k = ClustersChain[ClustersFindLastNode];
            // TODO: не уверен что так можно
            while ((k = ClustersChain[ClustersFindLastNode]) != -1) {
            //if (k != -1) {

                //not end of chain
//*TODO:** label found: lCheckNode:;
                //'keep as last result
                ClustersFindLastNode = k;
                //'deleted node - find next
                if (Nodes.get(k).nodeID == Node.MARK_NODEID_DELETED) {
                //*TODO:** goto found: GoTo lNextNode;
                    continue;
                }
                //'node outside desired bbox - find next
                if (Nodes.get(k).lat < ClustersFindLastBbox.lat_min || Nodes.get(k).lat > ClustersFindLastBbox.lat_max) {
                    //*TODO:** goto found: GoTo lNextNode;
                    continue;
                }
                if (Nodes.get(k).lon < ClustersFindLastBbox.lon_min || Nodes.get(k).lon > ClustersFindLastBbox.lon_max) {
                    //*TODO:** goto found: GoTo lNextNode;
                    continue;
                }
                //OK, found
                //_rtn = k;
                //*TODO:** goto found: GoTo lExit;
                return k;
            }

            //end of chain -> last node in cluster

//*TODO:** label found: lNextCluster:;
            //proceed to next cluster

            x ++;
            //next line of cluster
            if (x > x2) {
                y ++;
                x = x1;
            }
            if (y > y2) {
                //last cluster - no nodes
                //'nothing found
                _rtn = -1;
                //'nothing will be found next time
                ClustersFindLastNode = -1;
                ClustersFindLastCluster = -1;
                //*TODO:** goto found: GoTo lExit;
                return _rtn;
            }
        }
//*TODO:** label found: lCheckFirst:;
        //get first node of cluster

        j = x + y * ClustersLatNum;
        k = ClustersFirst[j];
        //no first node - skip cluster
        if (k == -1) { 
            //*TODO:** goto found: GoTo lNextCluster;
        }
        ClustersFindLastCluster = j;
        //'there is first node - check it
        //*TODO:** goto found: GoTo lCheckNode;

//*TODO:** label found: lExit:;

        return _rtn;
    }

    //Find edges of two way road
    //Algorithm goes by finding next edge on side, which is not leading
    //Found new node (end of found edge) is projected to local middle line
    //Array Chain is filled by found nodes
    //Arrays TWforw and TWback is filled by found edges
    //
    //edge1,edge2 - start edges
    //JoinDistance - distance limit between two ways
    //CombineDistance - distance to join two nodes into one (on middle line)
    //MaxCosine2 - angle limit between edges
    //Params: 0 - first pass (chain empty, go by edge1 direction)
    //        1 - second pass (chain contains all 4 nodes of edges at the end, go by edge2 direction)
    public static void goByTwoWays(int edge1, int edge2, double joinDistance, double combineDistance, double maxCosine2, int params) {
        Long, i = null; j As Long, k As Long

        //'arrow-head edges
        int edge_side1 = 0;
        int edge_side2 = 0;
        //'arrow-head nodes
        Long, side1i = null; side1j As Long
        Long, side2i = null; side2j As Long
        //'flags of circle on each side
        int side1circled = 0;
        int side2circled = 0;

        int[4] side(4) = null;
        double[4] dist(4) = null;
        double dist_t = 0;
        double dx = 0;
        double dy = 0;
        Double, px = null; py As Double
        double dd = 0;
        int roadtype = 0;
        double angl = 0;
        int calc_side = 0;
        Double, angl_min = null; angl_min_edge As Long
        int checkchain = 0;
        int passNumber = 0;

        //'keep road type for comparing
        roadtype = Edges[edge1].roadtype;

        //'mark edges as participating in joining
        Edges[edge1]..mark = 2;
        Edges[edge2]..mark = 2;

        //'arrow-head of finding chains
        edge_side1 = edge1;
        edge_side2 = edge2;

        //i node is back, j is front of arrow-head - on both sides
        side1i = Edges[edge1]..node1;
        side1j = Edges[edge1]..node2;
        side2i = Edges[edge2]..node2;
        side2j = Edges[edge2]..node1;

        //'circles not yet found
        side1circled = 0;
        side2circled = 0;

        passNumber = 0;
        if ((params && 1)) { passNumber = 1; }

        if (passNumber == 1) {
            //second pass
            //skip initial part, as it is already done in first pass
            //*TODO:** goto found: GoTo lKeepGoing;
        }

        //middle line projection vector
        //TODO: fix (not safe to 180/-180 edge)
        //'sum of two edges
        dx = (Nodes[side1j]..lat - Nodes[side1i]..lat) + (Nodes[side2j]..lat - Nodes[side2i]..lat);
        dy = (Nodes[side1j]..lon - Nodes[side1i]..lon) + (Nodes[side2j]..lon - Nodes[side2i]..lon);
        //'start point - average of two starts
        px = (Nodes[side1i]..lat + Nodes[side2i]..lat) * 0.5;
        py = (Nodes[side1i]..lon + Nodes[side2i]..lon) * 0.5;

        side[0] = side1i;
        side[1] = side1j;
        side[2] = side2i;
        side[3] = side2j;

        //calc relative positions of projections of all 4 noes to edge1
        dd = 1 / (dx * dx + dy * dy);
        for (i = 0; i <= 3; i++) {
            dist[i] = (Nodes[side[i]]..lat - px) * dx + (Nodes[side[i]]..lon - py) * dy;
        }

        //Sort dist() and side() by dist() by bubble sort
        for (i = 0; i <= 3; i++) {
            for (j = i + 1; j <= 3; j++) {
                if (dist[j] < dist[i]) {
                    dist_t = dist[j]: dist[j] == dist[i]: dist[i] == dist_t;
                    k = side[j]: side[j] == side[i]: side[i] == k;
                }
            }
        }

        //Add nodes to chain in sorted order
        for (i = 0; i <= 3; i++) {
            addChain(side[i]);
            Nodes[NodesNum].Edges = 0;
            Nodes[NodesNum]..nodeID = -1;
            Nodes[NodesNum]..mark = -1;
            //'info that old node will collapse to this new one
            Nodes[side[i]]..mark = NodesNum;
            //'projected coordinates
            Nodes[NodesNum]..lat = px + dist[i] * dx * dd;
            Nodes[NodesNum]..lon = py + dist[i] * dy * dd;
            addNode();
        }

//*TODO:** label found: lKeepGoing:;

        angl_min = MaxCosine2: angl_min_edge == -1;

        if (Chain[ChainNum - 1] == side1j) {
            //side1 is leading, side2 should be prolonged
            calc_side = 2;
        }
        else {
            //side2 is leading, side1 should be prolonged
            calc_side = 1;
        }

        if (calc_side == 2) {
            //search edge from side2j which is most opposite to edge_side1
            for (i = 0; i <= Nodes[side2j].Edges - 1; i++) {
                j = Nodes[side2j]..edge(i);
                if (j == edge_side2  || Edges[j]..node1 < 0 || Edges[j]..oneway == 0  || Edges[j].roadtype != roadtype  || Edges[j]..node2 != side2j) { 
                    //*TODO:** goto found: GoTo lSkipEdgeSide2;
                }
                //skip same edge_side2, deleted, 2-ways, other road types and directed from this node outside
                dist_t = distanceBetweenSegments(j, edge_side1);
                //'skip too far edges
                if (dist_t > joinDistance) { //*TODO:** goto found: GoTo lSkipEdgeSide2; }
                angl = cosAngleBetweenEdges(j, edge_side1);
                //'remember edge with min angle
                if (angl < angl_min) { angl_min = angl: angl_min_edge == j; }
                //*TODO:** label found: lSkipEdgeSide2:;
            }

            //'mark edge as participating in joining
            Edges[edge_side2]..mark = 2;
            //'add edge to chain (depending on pass number)
            addTW(edge_side2, passNumber);

            if (angl_min_edge == -1) {
                //no edge found - end of chain
                //'mark last edge of side1
                Edges[edge_side1]..mark = 2;
                //'and add it to chain
                addTW(edge_side1, 1 - passNumber);
                //*TODO:** goto found: GoTo lChainEnds;
            }

            edge_side2 = angl_min_edge;
            //'update i and j nodes of side
            side2i = side2j;
            side2j = Edges[edge_side2]..node1;

            if (Edges[edge_side2]..mark == 2) {
                //found marked edge, this means that we found cycle
                side2circled = 1;
            }

            if (side2j == side1j) {
                //found joining of two directions, should end chain
                //'mark both last edges as participating in joining
                Edges[edge_side2]..mark = 2;
                Edges[edge_side1]..mark = 2;
                //'add them to chains
                addTW(edge_side2, passNumber);
                addTW(edge_side1, 1 - passNumber);
                //*TODO:** goto found: GoTo lChainEnds;
            }

        }
        else {
            //search edge from side1j which is most opposite to edge_side2
            for (i = 0; i <= Nodes[side1j].Edges - 1; i++) {
                j = Nodes[side1j]..edge(i);
                if (j == edge_side1  || Edges[j]..oneway == 0  || Edges[j].roadtype != roadtype  || Edges[j]..node1 != side1j) { //*TODO:** goto found: GoTo lSkipEdgeSide1; }
                //skip same edge_side1, 2-ways, other road types and directed from this node outside
                dist_t = distanceBetweenSegments(j, edge_side2);
                //'skip too far edges
                if (dist_t > joinDistance) { 
                    //*TODO:** goto found: GoTo lSkipEdgeSide1;
                }
                angl = cosAngleBetweenEdges(j, edge_side2);
                //'remember edge with min angle
                if (angl < angl_min) { angl_min = angl: angl_min_edge == j; }
                //*TODO:** label found: lSkipEdgeSide1:;
            }

            //'mark edge as participating in joining
            Edges[edge_side1]..mark = 2;
            //'add edge to chain (depending on pass number)
            addTW(edge_side1, 1 - passNumber);

            if (angl_min_edge == -1) {
                //no edge found - end of chain
                //'mark last edge of side2
                Edges[edge_side2]..mark = 2;
                //'and add it to chain
                addTW(edge_side2, passNumber);
                //*TODO:** goto found: GoTo lChainEnds;
            }

            edge_side1 = angl_min_edge;
            //'update i and j nodes of side
            side1i = side1j;
            side1j = Edges[edge_side1]..node2;

            if (Edges[edge_side1]..mark == 2) {
                //found marked edge, means, that we found cycle
                side1circled = 1;
            }

            if (side2j == side1j) {
                //found marked edge, this means that we found cycle
                //'mark both last edges as participating in joining
                Edges[edge_side2]..mark = 2;
                Edges[edge_side1]..mark = 2;
                //'add them to chains
                addTW(edge_side2, passNumber);
                addTW(edge_side1, 1 - passNumber);
                //*TODO:** goto found: GoTo lChainEnds;
            }
        }

        //middle line projection vector
        //TODO: fix (not safe to 180/-180 edge)
        dx = Nodes[side1j]..lat - Nodes[side1i]..lat + Nodes[side2j]..lat - Nodes[side2i]..lat;
        dy = Nodes[side1j]..lon - Nodes[side1i]..lon + Nodes[side2j]..lon - Nodes[side2i]..lon;
        px = (Nodes[side1i]..lat + Nodes[side2i]..lat) * 0.5;
        py = (Nodes[side1i]..lon + Nodes[side2i]..lon) * 0.5;
        dd = 1 / (dx * dx + dy * dy);

        //'remember current chain len
        checkchain = ChainNum;

        if (calc_side == 2) {
            //project j node from side2 to middle line
            dist_t = (Nodes[side2j]..lat - px) * dx + (Nodes[side2j]..lon - py) * dy;
            addChain(side2j);
            //'old node will collapse to this new one
            Nodes[side2j]..mark = NodesNum;
        }
        else {
            //project j node from side1 to middle line
            dist_t = (Nodes[side1j]..lat - px) * dx + (Nodes[side1j]..lon - py) * dy;
            addChain(side1j);
            //'old node will collapse to this new one
            Nodes[side1j]..mark = NodesNum;
        }

        //create new node
        Nodes[NodesNum].Edges = 0;
        Nodes[NodesNum]..nodeID = -1;
        Nodes[NodesNum]..mark = -1;
        Nodes[NodesNum]..lat = px + dist_t * dx * dd;
        Nodes[NodesNum]..lon = py + dist_t * dy * dd;

        //reproject prev node into current middle line ("ChainNum - 2" because ChainNum were updated above by AddChain)
        j = Nodes[Chain[ChainNum - 2]]..mark;
        dist_t = (Nodes[j]..lat - px) * dx + (Nodes[j]..lon - py) * dy;
        Nodes[j]..lat = px + dist_t * dx * dd;
        Nodes[j]..lon = py + dist_t * dy * dd;

        if (distance(j, NodesNum) < combineDistance) {
            //Distance from new node to prev-one is too small, collapse node with prev-one
            //TODO(?): averaging coordinates?
            if (calc_side == 2) {
                Nodes[side2j]..mark = j;
            }
            else {
                Nodes[side1j]..mark = j;
            }
            //do not call AddNode -> new node will die
        }
        else {
            addNode();
            //'fix order of nodes in chain
            fixChainOrder(checkchain);
        }

        //'both sides circled - whole road is a loop
        if (side1circled > 0 && side2circled > 0) { 
            //*TODO:** goto found: GoTo lFoundCycle;
        }

        //'proceed to searching next edge
        //*TODO:** goto found: GoTo lKeepGoing;

//*TODO:** label found: lChainEnds:;

        //Node there is chance, that circular way will be not closed from one of sides
        //Algorithm does not handle this case, it should collapse during juctions collapsing

        return;

//*TODO:** label found: lFoundCycle:;
        //handle cycle road

        //find all nodes from end of chain which is present in chain two times
        //remove all of them, except last one
        //in good cases last node should be same as first node
        //TODO: what if not?
        for (i = ChainNum - 1; i <= 0; i--) {
            for (j = 0; j <= i - 1; j++) {
                if (Chain[i] == Chain[j]) { 
                    //*TODO:** goto found: GoTo lFound; 
                }
            }
            //not found
            //'keep this node (which is one time in chain) and next one (which is two times)
            ChainNum = i + 2;
            //*TODO:** goto found: GoTo lExit;
//*TODO:** label found: lFound:;
        }

//*TODO:** label found: lExit:;

    }

    //Reverse array into backward direction
    public static void reverseArray(int[] arr, int num) { // TODO: Use of ByRef founded
        int i = 0;
        int j = 0;
        int t = 0;
        //'half of len
        j = num / 2;
        for (i = 0; i <= j - 1; i++) {
            //swap elements from first and second halfs
            t = arr[i];
            arr[i] = arr[num - 1 - i];
            arr[num - 1 - i] = t;
        }
    }

}
