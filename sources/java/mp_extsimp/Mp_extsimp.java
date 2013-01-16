/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package mp_extsimp;

import java.io.*;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
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
    //private static int EdgesNum = 0;
    
    //Array for building chain of indexes
    private static ArrayList<Node> Chain;
    /* private static int[] Chain;
    private static int ChainAlloc = 0;
    private static int ChainNum = 0;*/
    
    //Aim edges
    private static aimedge[] AimEdges;
    private static int AimEdgesAlloc = 0;
    private static int AimEdgesNum = 0;    
    
    //Label statistics, for estimate labels of joined roads
    private static ArrayList<LabelStat> LabelStats;
    //private static int LabelStatsNum = 0;
    //private static int LabelStatsAlloc = 0;

    //Indexes of forward and backward ways during two ways joining
    private static ArrayList<Edge> TWforw;
    private static ArrayList<Edge> TWback;
    /*private static int[] TWforw;
    private static int[] TWback;
    private static int TWalloc = 0;
    private static int TWforwNum = 0;
    private static int TWbackNum = 0;*/

    //max found NodeID
    public static int NodeIDMax = 0;

    //edge, on which GoByChain() function have just passed from node to node
    public static Edge GoByChain_lastedge;

    //case of calc distance during last call of DistanceToSegment()
    public static int DistanceToSegment_last_case = 0;

    //histogramm of speed classes
    public static int[] SpeedHistogram;
    
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

    public static DecimalFormat numFormat;

    //speed of chain after last call of EstimateChain()
    public static int EstimateChain_speed = 0;
    //label of chain after last call of EstimateChain()
    public static String EstimateChain_label = "";
    
    public static void main(String[] args) {
        //Locale.setDefault(Locale.ENGLISH);
        optimizeRouting(args[0]);
    }
    
    private static void optimizeRouting(String inputFile) {
        String outFile = "";
        String outFile2 = "";
        long time1 = 0;

        //System.out.println(String.format("Nod1=0,%d,0\n", 8742));
        //System.out.println(String.format("%f\n", 8742.547820d));
        //DecimalFormat nf = new DecimalFormat("0.#######");
        //System.out.println(nf.format(37.6170351227685d));
        //nothing to do
        if (inputFile.isEmpty()) { return; }

        //output file
        outFile = inputFile + "_opt.mp";
        //outFile2 = inputFile + "_p.mp";  //output2 - for intermediate results
        
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

        filterVoidEdges();

        //Optimize all roads by (Ramer)Douglas Peucker algorithm
        douglasPeucker_total_split(5, 100);
        //Epsilon = 5 metres
        //Max edge - 100 metres

//        collapseJunctions2(1000, 1200, 0.13);
        //Slide allowed up to 1000 metres
        //Max junction loop is 1200 metres
        //0.13 -> ~ 7.46 degress

        filterVoidEdges();

        //Optimize all roads by (Ramer)DouglasPeucker algorithm
//        douglasPeucker_total(5);
        //Epsilon = 5 metres

        //Join edges with very acute angle into one
//        joinAcute(100, 3);
        //100 metres for joining nodes
        //AcuteKoeff = 3 => 18.4 degrees

        //Optimize all roads by (Ramer)DouglasPeucker algorithm
//        douglasPeucker_total(5);
        //Epsilon = 5 metres

        //Save result
        save_MP_2(outFile);

        //'display timing
        System.out.printf("Done %1$tH:%1$tM:%1$tS s", System.currentTimeMillis() - time1);
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

        Chain = new ArrayList<Node>();
        /* ChainAlloc = 1000;
        Chain = new int[ChainAlloc];
        ChainNum = 0;*/

        AimEdgesAlloc = 50;
        AimEdges = new aimedge[AimEdgesAlloc];
        AimEdgesNum = 0;

        LabelStats = new ArrayList<LabelStat>();

        /*TWalloc = 100;
        TWforw = new int[TWalloc];
        TWback = new int[TWalloc];
        TWbackNum = 0;
        TWforwNum = 0;*/
        TWforw = new ArrayList<Edge>();
        TWback = new ArrayList<Edge>();

        SpeedHistogram = new int[10];
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
        //int iPhase = 0;
        //long iStartLine = 0;
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
            //iPhase = 0;
            label = "";
            //iStartLine = 0;
            iPrevLine = 0;
            MPheader = "";
            lastCommentHighway = Highway.HIGHWAY_UNSPECIFIED;
            
            // для однопроходной загрузки
            Node addedNode;// = new Node(-1);
            ArrayList<Node> addedNodes = new ArrayList<Node>();
            ArrayList<Edge> addedEdges = new ArrayList<Edge>();
            
            // delete after fix
            int autoINCNodesNum = 0;

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
                    //iPhase = 0;
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
                if (sLine.startsWith("[POLY") && (sectionType == 3 || sectionType == 2)) {
                    if ((iPrevLine / 1023) > lastPercent) {
                        //display progress
                        //Form1.Caption = "Load: " + CStr(iPrevLine) + " / " + CStr(fileLen): Form1.Refresh;
                        lastPercent = (int)(iPrevLine / 1023);
// TODO                        System.out.printf("Load: (%3$d%%) %1$d / %2$d\n", iPrevLine, fileLen, lastPercent);
                    }
                    dataLineNum = 0;
                    //if (iPhase == 0) {
                        //first pass of section? start scanning
                        wayClass = -1;
                        waySpeed = -1;
                        //iPhase = 1;
                        //remember current pos (where to go after ending pass)
                        //iStartLine = iPrevLine - originalLenLine;
                        //br.mark(40*1024);   // резерв буфера на 40 КБ
                        //System.out.printf("Mark: %1$d / %2$d\n", iPrevLine , originalLenLine);
                    //}
                    // Инициализирую объект свойства которого буду заполнять
                    //addedNode = new Node(-1);
                }

                if (sLine.startsWith("[END")) {
                    //section ended

                    if (sectionType == 1) {
                        //add ending of section into saved header
                        MPheader = MPheader + sLine + "\r\n";
                    }
                    if (waySpeed != -1 && addedNodes.size() > 0 && addedEdges.size() > 0) {
                            Nodes.addAll(addedNodes);
                            Edges.addAll(addedEdges);
                    } else {
                // delete after fix
autoINCNodesNum -= addedNodes.size();
                    }
                    //if (iPhase > 0 && iPhase < 3) {
                        //not last pass of section -> goto start of it
                        //relocate in file

                        //Seek(1, iStartLine);
                        //fis.getChannel().position(iStartLine);
                        //System.out.printf("Reset: %1$d / %2$d\n", iPrevLine , originalLenLine);
                        //iPrevLine = iStartLine;
                        //br.reset();
                        //iPhase = iPhase + 1;

                        addedNodes.clear();
                        addedEdges.clear();                        
                    //}
                    //no routing params found in 1st pass - skip way completely
                    //if (iPhase == 1  && (waySpeed == -1)) { 
                    //    iPhase = 0;
                    //}


                    //if no osm2mp info yet found
                    lastCommentHighway = Highway.HIGHWAY_UNSPECIFIED;
                    label = "";
                    //iPhase = 0;
                    sectionType = 0;
    //*TODO:** goto found: GoTo lNextLine;
                    continue;
                }

                //switch (iPhase) {
                //    case  0:
                        if (sLine.startsWith("; highway")) {
                            //comment, produced by osm2mp
                            lastCommentHighway = Highway.getHighwayType(sLine.substring(12).trim());
                        }
                        if (sectionType == 1) {
                            //line of header section
                            MPheader = MPheader + sLine + "\r\n";
                        }
                //        break;
                    //scan for routing param
                //    case  1:
                        if (sLine.startsWith("RouteParam")) {
                            //'skip ext
                            if (sLine.startsWith("RouteParamExt")) { continue; } // break; } //{ goto: GoTo lNoData; // skipp ext}
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
                //        break;
                //    case  3:
                        //scan for node routing info:
                        if (sLine.startsWith("Nod")) {
                            //Nod

                            k2 = sLine.indexOf("=") + 1;
                            if (k2 <= 0) { continue; }// break; } //*TODO:** goto found: GoTo lSkipRoadNode; }
                            routep = sLine.substring(k2).split(",");
                            k = Integer.parseInt(routep[0]);

                            if (k > addedNodes.size()) { //NodesAlloc) {
                                //error: too big node index: " + sLine
                                //*TODO:** goto found: GoTo lSkipRoadNode;
                                System.out.println("error: too big node index: " + sLine);
                                continue; //break;
                            }

                            //k3 = sLine.indexOf(",", k2);
                            //if (k3 < 0) {
                            if (routep.length < 2) {
                                //error: bad NodeID
                                //*TODO:** goto found: GoTo lSkipRoadNode;
                                System.out.println("error: bad NodeID");
                                continue; //break;
                            }
                            nodeID = Integer.parseInt(routep[1]);

                            //update max NodeID
                            if (nodeID > NodeIDMax) { NodeIDMax = nodeID; }

                            //store nodeid
                            //Nodes[NodesNum - thisLineNodes + k].nodeID = nodeID;
                            //Nodes.add(new Node(nodeID));
                            addedNodes.get(k).nodeID = nodeID;
                            
//*TODO:** label found: lSkipRoadNode:;
                        }
                //        break;
                //    case  2:
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
                                //Node addedNode = new Node(-1);    **
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
                                addedNode = new Node(-1);
                                
                                addedNode.lat = fLat;
                                addedNode.lon = fLon;
                                //addedNode.nodeID = -1;
                                                // delete after fix
                addedNode.VBNum = autoINCNodesNum++;
                                addedNodes.add(addedNode);

                                if (thisLineNodes > 0) {
                                    //not the first node of way -> create edge
                                    //Edge jEdge = joinByEdge(Nodes.size() - 2, Nodes.size() - 1);
                                    //Edge jEdge = joinByEdge(Nodes.get(Nodes.size() - 2), addedNode);
                                    Edge jEdge = Edge.joinByEdge(addedNodes.get(addedNodes.size() - 2), addedNode);
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
                                    addedEdges.add(jEdge);
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
                //        break;
                //}
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
    public static Edge joinByEdge(Node node1, Node node2) {
        Edge result = Edge.joinByEdge(node1, node2);
        Edges.add(result);
        return result;
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
        int k;
        int mapNum;
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
                        Node.mergeNodes(Nodes.get(iDmap[k]), Nodes.get(i));
                    }
                }
//*TODO:** label found: lSkip:;

                if ((i % 80) == 0) {  //8191
                    //display progress
                    //Form1.Caption = "Join soft " + CStr(i) + " / " + CStr(NodesNum): Form1.Refresh;
// TODO                    System.out.printf("Join soft %1$d / %2$d\n", i, NodesNum);
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
                            Node.mergeNodes(Nodes.get(nodeMap[j]), Nodes.get(i));
                            //*TODO:** goto found: GoTo lFound;
                            break;
                        }
                    }
                    if (j > i - 1) {        // добавил из-за брейка в цикле
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

    //Join two directions of road way
    //MaxCosine - cosine of max angle between start edges, -0.996 means (175,180) degrees - contradirectional edges or close
    //MaxCosine2 - cosine of max angle between other edges, during going-by-two-ways
    //MinChainLen - length of min two-way road to join
    public static void joinDirections3(double joinDistance, double maxCosine, double maxCosine2, double minChainLen, double combineDistance) {
        int i,  j;
        Node clusterNode;
        boolean mode1;
        Node eNode, dNode;
        int e, d;
        Edge q;
        double dist1, dist2;
        Bbox bbox_edge;
        double angl;
        double min_dist;
        Edge min_dist_edge;
        int roadtype;
        int speednew;

        //chain of forward edges
        //int[] edgesForw;
        //ArrayList<Edge> edgesForw;
        Edge[] edgesForw;
        //chain of backward edges
        //int[] edgesBack;
        //ArrayList<Edge> edgesBack;
        Edge[] edgesBack;
        //1 if road is circled
        int loopChain = 0;
        //len of half of road
        int halfChain = 0;

        //Algorithm will check all non-link oneway edges for presence of contradirectional edge in the vicinity
        //All found pairs of edges will be checked in both directions by GoByTwoWays function
        //for presence of continuous road of one type
        //During this check will be created new chain of nodes, which is projection of joining nodes into middle line
        //Then both found ways will be joined into one bidirectional way, consist from new nodes
        //All related roads will reconnected to new way and old edges were deleted

        //int NodesNum = Nodes.size();
        //mark all nodes as not checked - default = null
        /*for (i = 0; i <= NodesNum - 1; i++) {
            //not moved
            Nodes.get(i).mark = null;
        }*/
        int EdgesNum = Edges.size();
        for (i = 0; i < EdgesNum; i++) {
            Edge edgeI = Edges.get(i);
            edgeI.mark = 1;
            
            //skip deleted and 2-ways edges
            if (edgeI.node1 == null || edgeI.oneway == 0) {
                //*TODO:** goto found: GoTo lFinMarkEdge;
                continue;
            }
            //skip links
            if ((edgeI.roadtype & Highway.HIGHWAY_MASK_LINK) != 0) {
                //*TODO:** goto found: GoTo lFinMarkEdge;
                continue;
            }
            //skip edges between complex connections
            if (edgeI.node1.edgeL.size() != 2  && edgeI.node2.edgeL.size() != 2) { 
                //*TODO:** goto found: GoTo lFinMarkEdge;
                continue;
            }
            edgeI.mark = 0;
//*TODO:** label found: lFinMarkEdge:;
        }

        //rebuild cluster-index from 0
        buildNodeClusterIndex(false);

        for (i = 0; i < EdgesNum; i++) {
            Edge edgeI = Edges.get(i);
            //skip marked edge or deleted
            if (edgeI.mark > 0 || edgeI.node1 == null) {
        //*TODO:** goto found: GoTo lSkipEdge;
                continue;
            }
            //'get bbox
            //bbox_edge = getEdgeBbox(i);
            bbox_edge = Edge.getEdgeBbox(edgeI.node1, edgeI.node2);
            //'expand it
            Bbox.expandBbox(bbox_edge, joinDistance);
            min_dist = joinDistance;
            min_dist_edge = null;

            //first
            mode1 = false;

            while (true) {
//*TODO:** label found: lSkipNode2:;
                clusterNode = getNodeInBboxByCluster(bbox_edge, mode1);
                if (clusterNode != null) System.out.println("Node(" + clusterNode.VBNum + ").ID = " + clusterNode.nodeID);
                // System.out.println("k = " + clusterNode.VBNum + " ID=" + clusterNode.nodeID);
                //next (next time)
                mode1 = true;
                //no more nodes
                if (clusterNode == null) {
                    //*TODO:** goto found: GoTo lAllNodes;
                    break;
                }

                //skip nodes of same edge, deleted and complex nodes
                if (clusterNode == edgeI.node1  || clusterNode == edgeI.node2  || 
                        clusterNode.nodeID == Mark.MARK_NODEID_DELETED  || clusterNode.edgeL.size() != 2) {
                    //*TODO:** goto found: GoTo lSkipNode2;
                    continue;
                }

                //calc dist from found node to our edge
                dist1 = Node.distanceToSegment(edgeI.node1, edgeI.node2, clusterNode);
                //DistanceToSegment_last_case = Node.DistanceToSegment_last_case;
                //too far, skip
                if (dist1 > min_dist) {
                    //*TODO:** goto found: GoTo lSkipNode2;
                    continue;
                }

                //node is on join distance, check all (2) edges
                for (int d1 = 0; d1 < 2; d1++) {
                    q = clusterNode.edgeL.get(d1);
                    //deleted or 2-way edge or other road class
                    if (q.node1 == null || q.oneway == 0 || q.roadtype != edgeI.roadtype) {
                    //*TODO:** goto found: GoTo lSkipEdge2;
                        continue;
                    }
                    angl = Edge.cosAngleBetweenEdges(q, edgeI);
                    if (angl < maxCosine) {
                        //contradirectional edge or close

                        dist1 = Edge.distanceBetweenSegments(edgeI, q);
                        //found edge close enough
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

            if (min_dist_edge != null) {
                //found edge close enough
                //now - trace two ways in both directions
                //in the process we will fill Chain array with all nodes of joining ways
                //sequence of nodes in Chain will correspond to sequence of nodes on combined way
                //index of new node, where old node should join, will in .mark field of old nodes
                //also will be created two lists of deleting edges separated to two directions - arrays TWforw and TWback

                roadtype = edgeI.roadtype;
                loopChain = 0;
                //ChainNum = 0;
                //TWforwNum = 0;
                //TWbackNum = 0;
                Chain.clear();
                TWforw.clear();
                TWback.clear();

                //first pass, in direction of edge i
                goByTwoWays(edgeI, min_dist_edge, joinDistance, combineDistance, maxCosine2, false);

                //reverse of TWforw and TWback removed, as order of edges have no major effect

                //reverse Chain
                reverseArray(Chain);


                //second pass, in direction of min_dist_edge
                goByTwoWays(min_dist_edge, edgeI, joinDistance, combineDistance, maxCosine2, true);

                //first and last nodes coincide - this is loop road
                if (Chain.get(0).equals(Chain.get(Chain.size() - 1))) { loopChain = 1; }

                //half len of road in nodes
                halfChain = Chain.size() / 2;
                //will "kill" halfchain limit for very short loops
                // TODO: возможно +1 не нужен
                if (halfChain < 10) { halfChain = Chain.size(); }

                //call metric length of found road
                dist1 = 0;
                for (j = 1; j < Chain.size(); j++) {
                    // TODO: марк вроде бы как маркер, а тут как индекс ?
                    dist1 += Node.distance(Chain.get(j - 1).mark, Chain.get(j).mark);
                }

                if (dist1 < minChainLen) {
                    //road is too short -> unmark all edges and not delete anything
                    /*for (j = 0; j < Chain.size(); j++) {
                        for (clusterNode = 0; clusterNode < Chain.get(j).edgeL.size(); clusterNode++) {
                            if (Edges.get(Chain.get(j).edgeL[clusterNode]).mark == 2) {
                                Edges.get(Chain.get(j).edgeL[clusterNode]).mark = 1;
                            }
                        }
                    }*/
                    for(Iterator<Node> jChain = Chain.iterator(); jChain.hasNext();) {
                        for(Iterator<Edge> kEdge = jChain.next().edgeL.iterator(); kEdge.hasNext();) {
                            Edge kEdgeN = kEdge.next();
                            if (kEdgeN.mark == 2) { kEdgeN.mark = 1; }
                        }
                    }
                    //*TODO:** goto found: GoTo lSkipDel;
                } else {

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
                    //edgesForw = new int[ChainNum];
                    //G.redim(edgesBack, ChainNum);
                    //edgesBack = new int[ChainNum];
                    // TODO: не уверен что не будет расширение элементов, но буду надеяться
                    //edgesForw = new ArrayList<Edge>(Chain.size());
                    //edgesBack = new ArrayList<Edge>(Chain.size());
                    edgesForw = new Edge[Chain.size()];
                    edgesBack = new Edge[Chain.size()];
                    // TODO: думаю не актуально
                    /*
                    for (j = 0; j < Chain.size(); j++) {
                        edgesForw.set(j, null);    // -1;
                        edgesBack.set(j, null);    // -1;
                    }
                    */
                    //process forward direction
                    for (j = 0; j < TWforw.size(); j++) {
                        eNode = TWforw.get(j).node1;
                        dNode = TWforw.get(j).node2;
                        //get indexes of nodes inside Chain
                        e = Chain.indexOf(eNode);
                        d = Chain.indexOf(dNode);
                        if (e == -1 || d == -1) {
                            //(should not happen)
                            //edge with nodes not in chain - skip
                    //*TODO:** goto found: GoTo lSkip1;
                            continue;
                        }
//                        System.out.println("indexOf: " + e + ", " + d);

                        if (e < d) {
                            //normal forward edge (or pleat crossing 0 of chain)
                            // ... e ---> d .....
                            //skip too long edges on loop chains as it could be wrong (i.e. pleat edge which cross 0 of chain)
                            if (loopChain == 1  && (d - e) > halfChain) {
                    //*TODO:** goto found: GoTo lSkip1:;
                                continue;
                            }
                            for (int q1 = e; q1 < d; q1++) {
                                //in forward direction between q and q+1 node is edge TWforw(j)
                                //edgesForw.set(q1, TWforw.get(j));
                                edgesForw[q1] = TWforw.get(j);
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
                                for (int q1 = 0; q1 < d; q1++) {
                                    //edgesForw.set(q1, TWforw.get(j));
                                    edgesForw[q1] = TWforw.get(j);
                                }
                                for (int q1 = e; q1 < Chain.size(); q1++) {
                                    //edgesForw.set(q1, TWforw.get(j));
                                    edgesForw[q1] = TWforw.get(j);
                                }
                            }
                        }
    //*TODO:** label found: lSkip1:;
                    }

                    //process backward direction
                    for (j = 0; j < TWback.size(); j++) {
                        eNode = TWback.get(j).node1;
                        dNode = TWback.get(j).node2;
                        //get indexes of nodes inside Chain
                        e = Chain.indexOf(eNode);
                        d = Chain.indexOf(dNode);
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
                            for (int q1 = d; q1 < e; q1++) {
                                //edgesBack.set(q1, TWback.get(j));
                                edgesBack[q1] = TWback.get(j);
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
                                for (int q1 = 0; q1 < e; q1++) {
                                    edgesBack[q1] = TWback.get(j);
                                }
                                for (int q1 = d; q1 < Chain.size(); q1++) {
                                    edgesBack[q1] = TWback.get(j);
                                }
                            }
                        }
    //*TODO:** label found: lSkip2:;
                    }

                    for (j = 1; j < Chain.size(); j++) {
                        Node chainJ_1 = Chain.get(j - 1);
                        Node chainJ = Chain.get(j);
                        Edge edgesForwJ_1 = edgesForw[j - 1];
                        Edge edgesBackJ_1 = edgesBack[j - 1];
                        dNode = chainJ_1.mark;
                        eNode = chainJ.mark;
                        if (dNode != eNode) {
                            /*k = joinByEdge(Nodes.get(Chain[j - 1]).mark, Nodes.get(Chain[j]).mark);
                            Edges.get(k).roadtype = roadtype;
                            Edges.get(k).oneway = 0;
                            Edges.get(k).mark = 1;
                            */
                            Edge edg = joinByEdge(chainJ_1.mark, chainJ.mark);
                            // TODO: не будет ли потери при байте
                            edg.roadtype = (byte)roadtype;
                            edg.oneway = 0;
                            edg.mark = 1;

                            if ((edgesForwJ_1 == null) && (edgesBackJ_1 == null)) {
                                //no edges for this interval between nodes
                                //(should never happens)
                                //default value
                                /*Edges.get(k).speed = 3;
                                Edges.get(k).label = "";*/
                                edg.speed = 3;
                                edg.label = "";
                            }
                            else {
                                //get minimal speed class of both edges
                                speednew = 10;
                                resetLabelStats();
                                if (edgesForwJ_1 != null) {
                                    //forward edge present
                                    speednew = edgesForwJ_1.speed;
                                    addLabelStat0(edgesForwJ_1.label);
                                }
                                if (edgesBackJ_1 != null) {
                                    //backward edge present
                                    if (speednew > edgesBackJ_1.speed) { speednew = edgesBackJ_1.speed; }
                                    addLabelStat0(edgesBackJ_1.label);
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

                            if (edgesBackJ_1 == null) {
                                //no backward edge - result in one-way
                                //Edges.get(k).oneway = 1;
                                edg.oneway = 1;
                            }
                            else if (edgesForwJ_1 == null) {
                                //no forward edge - result in one-way, backward to other road
                                /*Edges.get(k).oneway = 1;
                                Edges.get(k).node1 = Nodes[Chain[j]]..mark;
                                Edges.get(k).node2 = Nodes[Chain[j - 1]]..mark;*/
                                edg.oneway = 1;
                                edg.node1 = chainJ.mark;
                                edg.node2 = chainJ_1.mark;
                            }
                        }
                    }

                    //delete all old edges
                    /*for (j = 0; j < TWforw.size(); j++) {
                        TWforw.get(j).delEdge();
                    }
                    for (j = 0; j < TWback.size(); j++) {
                        delEdge(TWback.get(j));
                    }
                    */
                    for(Iterator<Edge> iEdge = TWforw.iterator(); iEdge.hasNext();) {
                        iEdge.next().delEdge();
                    }
                    for(Iterator<Edge> iEdge = TWback.iterator(); iEdge.hasNext();) {
                        iEdge.next().delEdge();
                    }

                    //merge all old nodes into new ones
                    /*for (j = 0; j <= ChainNum - 1; j++) {
                        mergeNodes(Nodes.get(Chain[j]).mark, Chain[j], 1);
                    }*/
                    for(Iterator<Node> iNode = Chain.iterator(); iNode.hasNext();) {
                        Node iNodeN = iNode.next();
                        Node.mergeNodes(iNodeN.mark, iNodeN, true);
                    }

                    //update cluster index to include only newly created nodes (i.e. nodes of joined road)
                    buildNodeClusterIndex(true);

    //*TODO:** label found: lSkipDel:;
                }
            }
System.out.println("i = " + i);
            //mark edge as checked
            edgeI.mark = 1;

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
    public static void buildNodeClusterIndex(boolean flags) {
        int i, j, k;
        int x;
        int y;

        if (flags) {
            //Only update
            //TODO(?): remove chain from deleted nodes
            //G.redimPreserve(ClustersChain, NodesNum);
            //TODO: хз, что тут делать, пока
            int[] newClustersChain = new int[Nodes.size()];
            System.arraycopy(ClustersChain, 0, newClustersChain, 0, ClustersChain.length);
            ClustersChain = newClustersChain;//new int[Nodes.size()];

            //*TODO:** goto found: GoTo lClustering;
        } else {

            //calc overall bbox
            Bbox wholeBbox = new Bbox();
            wholeBbox.lat_max = -360;
            wholeBbox.lat_min = 360;
            wholeBbox.lon_max = -360;
            wholeBbox.lon_min = 360;
            //for (i = 0; i <= NodesNum - 1; i++) {
            for (Iterator<Node> iNode = Nodes.iterator(); iNode.hasNext();) {
                Node iNodeN = iNode.next();
                //skip deleted nodes
                if (iNodeN.nodeID != Mark.MARK_NODEID_DELETED) {
                    if (iNodeN.lat < wholeBbox.lat_min) { wholeBbox.lat_min = iNodeN.lat; }
                    if (iNodeN.lat > wholeBbox.lat_max) { wholeBbox.lat_max = iNodeN.lat; }
                    if (iNodeN.lon < wholeBbox.lon_min) { wholeBbox.lon_min = iNodeN.lon; }
                    if (iNodeN.lon > wholeBbox.lon_max) { wholeBbox.lon_max = iNodeN.lon; }
                }
            }

            ClustersIndexedNodes = 0;
            //no nodes at all or something wrong
            if (wholeBbox.lat_max < wholeBbox.lat_min || wholeBbox.lon_max < wholeBbox.lon_min) { return; }

            //calc number of clusters
            ClustersLatNum = (int)Math.round(1 + (wholeBbox.lat_max - wholeBbox.lat_min) / CLUSTER_SIZE);
            ClustersLonNum = (int)Math.round(1 + (wholeBbox.lon_max - wholeBbox.lon_min) / CLUSTER_SIZE);

            /*
            //starts of chains
            G.redim(ClustersFirst, ClustersLatNum * ClustersLonNum);
            //ends of chains (for updating)
            G.redim(ClustersLast, ClustersLatNum * ClustersLonNum);
            //whole chain
            G.redim(ClustersChain, NodesNum);
            */
            ClustersChain = new int[Nodes.size()];
            ClustersFirst = new int[ClustersLatNum * ClustersLonNum];
            ClustersLast = new int[ClustersLatNum * ClustersLonNum];

            //edge of overall bbox
            ClustersLat0 = wholeBbox.lat_min;
            ClustersLon0 = wholeBbox.lon_min;

            for (i = 0; i < ClustersLatNum * ClustersLonNum; i++) {
                //'no nodes in cluster yet
                ClustersFirst[i] = -1;
                ClustersLast[i] = -1;
            }

            ClustersIndexedNodes = 0;
        }
//*TODO:** label found: lClustering:;
        for (i = ClustersIndexedNodes; i < Nodes.size(); i++) {
            Node iNode = Nodes.get(i);
            if (iNode.nodeID != Mark.MARK_NODEID_DELETED) {
                //get cluster from lat/lon
                x = (int)Math.round((iNode.lat - ClustersLat0) / CLUSTER_SIZE);
                y = (int)Math.round((iNode.lon - ClustersLon0) / CLUSTER_SIZE);
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
                //this is last node in chain
                ClustersChain[i] = -1;
                ClustersLast[j] = i;
            }
        }

        //last node in cluster index
        ClustersIndexedNodes = Nodes.size();
    }

    //Find node in bbox by using cluster index
    //Flags: 1 - next (0 - first)
    // TODO: перелопачена здорого, могут быть БАГИ
    public static Node getNodeInBboxByCluster(Bbox box1, boolean flags) {
        int j, k = 0;
        int x, y;
        int x1, x2, y1, y2;

        boolean endChain = true;
        
        if (!flags) {
            //first node needed

            //get coordinates of all needed clusters
            x1 = (int)Math.round((box1.lat_min - ClustersLat0) / CLUSTER_SIZE);
            x2 = (int)Math.round((box1.lat_max - ClustersLat0) / CLUSTER_SIZE);
            y1 = (int)Math.round((box1.lon_min - ClustersLon0) / CLUSTER_SIZE);
            y2 = (int)Math.round((box1.lon_max - ClustersLon0) / CLUSTER_SIZE);

            //store bbox for next searches
            ClustersFindLastBbox = box1;
            x = x1;
            y = y1;
        } else {
            if (ClustersFindLastNode == -1) {
                //Last time nothing found - nothing to do further
                return null;
            }

            //get coordinates of all needed clusters
            x1 = (int)Math.round((ClustersFindLastBbox.lat_min - ClustersLat0) / CLUSTER_SIZE);
            x2 = (int)Math.round((ClustersFindLastBbox.lat_max - ClustersLat0) / CLUSTER_SIZE);
            y1 = (int)Math.round((ClustersFindLastBbox.lon_min - ClustersLon0) / CLUSTER_SIZE);
            y2 = (int)Math.round((ClustersFindLastBbox.lon_max - ClustersLon0) / CLUSTER_SIZE);

            //get coordinates of last used cluster
            x = ClustersFindLastCluster % ClustersLatNum;   // Mod
            y = (int)((ClustersFindLastCluster - x) / ClustersLatNum);  // было целое деление или как-то так "\"

            k = ClustersChain[ClustersFindLastNode];
            if (k == -1) {
                x++;
                endChain = true;
            } else { endChain = false; }
        }
        
        
        //proceed to next cluster
        while(true) {
            if (endChain) {
                //next line of cluster
                if (x > x2) {
                    y ++;
                    x = x1;
                }
                if (y > y2) {
                    //last cluster - no nodes
                    //nothing found
                    //nothing will be found next time
                    ClustersFindLastNode = -1;
                    ClustersFindLastCluster = -1;
                    break;
                }

                //get first node of cluster

                j = x + y * ClustersLatNum;
                k = ClustersFirst[j];
                //no first node - skip cluster
                if (k == -1) {
                    //*TODO:** goto found: GoTo lNextCluster;
                    continue;
                }
                ClustersFindLastCluster = j;
            }
            //get node from chain
            //k = ClustersChain[ClustersFindLastNode];
            // TODO: не уверен что так можно
            do { // ((k = ClustersChain[ClustersFindLastNode]) != -1) {
            //if (k != -1) {

                //not end of chain
//*TODO:** label found: lCheckNode:;
                //keep as last result
                ClustersFindLastNode = k;
                Node testNode = Nodes.get(k);
                //deleted node - find next
                if (testNode.nodeID == Node.MARK_NODEID_DELETED) {
                //*TODO:** goto found: GoTo lNextNode;
                    continue;
                }
                //node outside desired bbox - find next
                if (testNode.lat < ClustersFindLastBbox.lat_min || testNode.lat > ClustersFindLastBbox.lat_max) {
                    //*TODO:** goto found: GoTo lNextNode;
                    continue;
                }
                if (testNode.lon < ClustersFindLastBbox.lon_min || testNode.lon > ClustersFindLastBbox.lon_max) {
                    //*TODO:** goto found: GoTo lNextNode;
                    continue;
                }
                //OK, found
                //*TODO:** goto found: GoTo lExit;
                return testNode;
            } while ((k = ClustersChain[ClustersFindLastNode]) != -1);
            //end of chain -> last node in cluster
            x ++;
            endChain = true;
        }
        
        
        
        return null;
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
    //public static void goByTwoWays(int edge1, int edge2, double joinDistance, double combineDistance, double maxCosine2, int params) {
    public static void goByTwoWays(Edge edge1, Edge edge2, double joinDistance, double combineDistance, double maxCosine2, boolean params) {
        //int i;

        //arrow-head edges
        //int edge_side1 = 0;
        //int edge_side2 = 0;
        Edge edge_side1;
        Edge edge_side2;

        //arrow-head nodes
        Node side1i, side1j;
        Node side2i, side2j;
        //flags of circle on each side
        int side1circled;
        int side2circled;

        Node[] side = new Node[4];
        double[] dist = new double[4];
        double dist_t;
        double dx;
        double dy;
        double px, py;
        double dd;
        int roadtype;
        double angl;
        int calc_side;
        double angl_min;
        Edge angl_min_edge;
        int checkchain;
        int passNumber;

        //keep road type for comparing
        roadtype = edge1.roadtype;

        //mark edges as participating in joining
        edge1.mark = 2;
        edge2.mark = 2;

        //arrow-head of finding chains
        edge_side1 = edge1;
        edge_side2 = edge2;

        //i node is back, j is front of arrow-head - on both sides
        side1i = edge1.node1;
        side1j = edge1.node2;
        side2i = edge2.node2;
        side2j = edge2.node1;

        //circles not yet found
        side1circled = 0;
        side2circled = 0;

        passNumber = 0;
        if (params) { passNumber = 1; }

        if (passNumber == 1) {
            //second pass
            //skip initial part, as it is already done in first pass
   //*TODO:** goto found: GoTo lKeepGoing;
        } else {

            //middle line projection vector
            //TODO: fix (not safe to 180/-180 edge)
            //sum of two edges
            dx = (side1j.lat - side1i.lat) + (side2j.lat - side2i.lat);
            dy = (side1j.lon - side1i.lon) + (side2j.lon - side2i.lon);
            //start point - average of two starts
            px = (side1i.lat + side2i.lat) * 0.5;
            py = (side1i.lon + side2i.lon) * 0.5;

            side[0] = side1i;
            side[1] = side1j;
            side[2] = side2i;
            side[3] = side2j;

            //calc relative positions of projections of all 4 noes to edge1
            dd = 1 / (dx * dx + dy * dy);
            for (int i = 0; i <= 3; i++) {
                dist[i] = (side[i].lat - px) * dx + (side[i].lon - py) * dy;
            }

            //Sort dist() and side() by dist() by bubble sort
            for (int i = 0; i <= 3; i++) {
                for (int j = i + 1; j <= 3; j++) {
                    if (dist[j] < dist[i]) {
                        dist_t = dist[j]; dist[j] = dist[i]; dist[i] = dist_t;
                        Node k = side[j]; side[j] = side[i]; side[i] = k;
                    }
                }
            }

            //Add nodes to chain in sorted order
            for (int i = 0; i <= 3; i++) {
                //addChain(side[i]);
                Chain.add(side[i]);
                /*
                Nodes.get(Nodes.size().Edges = 0;
                Nodes.get(Nodes.size().nodeID = -1;
                Nodes.get(Nodes.size().mark = -1;
                //'info that old node will collapse to this new one
                Nodes.get([side[i]).mark = NodesNum;
                //'projected coordinates
                Nodes.get(Nodes.size().lat = px + dist[i] * dx * dd;
                Nodes.get(Nodes.size().lon = py + dist[i] * dy * dd;
                addNode(); */
                Node addedNode = new Node(-1);
                //addedNode.mark = null;
                addedNode.lat = px + dist[i] * dx * dd;
                addedNode.lon = py + dist[i] * dy * dd;
                
                // delete after fix
                addedNode.VBNum = Nodes.size();
                Nodes.add(addedNode);
                side[i].mark = addedNode;
            }
        }

//*TODO:** label found: lKeepGoing:;
        while (true) {

            angl_min = maxCosine2; angl_min_edge = null;

            if (Chain.get(Chain.size()-1) == side1j) {
                //side1 is leading, side2 should be prolonged
                calc_side = 2;
            }
            else {
                //side2 is leading, side1 should be prolonged
                calc_side = 1;
            }

            if (calc_side == 2) {
                //search edge from side2j which is most opposite to edge_side1
                for (Iterator<Edge> iEdge = side2j.edgeL.iterator(); iEdge.hasNext();) {
                //for (i = 0; i < side2j.Edges - 1; i++) {
                    Edge jEdge = iEdge.next();//Nodes.get(side2j).edgeL[i];
                    // TODO: возможно что-то не так, было j == edge_side2 (индексы)
                    if (jEdge.equals(edge_side2)  || jEdge.node1 == null || jEdge.oneway == 0
                            || jEdge.roadtype != roadtype  || jEdge.node2 != side2j) {
                //*TODO:** goto found: GoTo lSkipEdgeSide2;
                        continue;
                    }
                    //skip same edge_side2, deleted, 2-ways, other road types and directed from this node outside
                    //dist_t = distanceBetweenSegments(j, edge_side1);
                    dist_t = Edge.distanceBetweenSegments(jEdge, edge_side1);
                    //skip too far edges
                    if (dist_t > joinDistance) {
                //*TODO:** goto found: GoTo lSkipEdgeSide2;
                        continue;
                    }
                    //angl = cosAngleBetweenEdges(j, edge_side1);
                    angl = Edge.cosAngleBetweenEdges(jEdge, edge_side1);
                    //remember edge with min angle
                    if (angl < angl_min) { angl_min = angl; angl_min_edge = jEdge; }
    //*TODO:** label found: lSkipEdgeSide2:;
                }

                //mark edge as participating in joining
                edge_side2.mark = 2;
                //add edge to chain (depending on pass number)
                addTW(edge_side2, passNumber);

                if (angl_min_edge == null) {
                    //no edge found - end of chain
                    //mark last edge of side1
                    edge_side1.mark = 2;
                    //and add it to chain
                    addTW(edge_side1, 1 - passNumber);
            //*TODO:** goto found: GoTo lChainEnds;
                    break;
                }

                edge_side2 = angl_min_edge;
                //update i and j nodes of side
                side2i = side2j;
                side2j = edge_side2.node1;

                if (edge_side2.mark == 2) {
                    //found marked edge, this means that we found cycle
                    side2circled = 1;
                }

                if (side2j == side1j) {
                    //found joining of two directions, should end chain
                    //mark both last edges as participating in joining
                    edge_side2.mark = 2;
                    edge_side1.mark = 2;
                    //add them to chains
                    addTW(edge_side2, passNumber);
                    addTW(edge_side1, 1 - passNumber);
            //*TODO:** goto found: GoTo lChainEnds;
                    break;
                }

            }
            else {
                //search edge from side1j which is most opposite to edge_side2
                for (Iterator<Edge> iEdge = side1j.edgeL.iterator(); iEdge.hasNext();) {
                //for (i = 0; i <= Nodes.get(side1j).Edges - 1; i++) {
                    //Edge jEdge = iEdge.next();  //Nodes.get(side1j).edgeL[i];
                    Edge edgeJ = iEdge.next();  //Edges.get(j);
                    if (edgeJ.equals(edge_side1) || edgeJ.oneway == 0 || edgeJ.roadtype != roadtype  || edgeJ.node1 != side1j) {
                //*TODO:** goto found: GoTo lSkipEdgeSide1;
                        continue;
                    }
                    //skip same edge_side1, 2-ways, other road types and directed from this node outside
                    dist_t = Edge.distanceBetweenSegments(edgeJ, edge_side2);
                    //skip too far edges
                    if (dist_t > joinDistance) {
                //*TODO:** goto found: GoTo lSkipEdgeSide1;
                        continue;
                    }
                    angl = Edge.cosAngleBetweenEdges(edgeJ, edge_side2);
                    //remember edge with min angle
                    if (angl < angl_min) { angl_min = angl; angl_min_edge = edgeJ; }
                //*TODO:** label found: lSkipEdgeSide1:;
                }

                //mark edge as participating in joining
                edge_side1.mark = 2;
                //add edge to chain (depending on pass number)
                addTW(edge_side1, 1 - passNumber);

                if (angl_min_edge == null) {
                    //no edge found - end of chain
                    //mark last edge of side2
                    edge_side2.mark = 2;
                    //and add it to chain
                    addTW(edge_side2, passNumber);
                //*TODO:** goto found: GoTo lChainEnds;
                    break;
                }

                edge_side1 = angl_min_edge;
                //update i and j nodes of side
                side1i = side1j;
                side1j = edge_side1.node2;

                if (edge_side1.mark == 2) {
                    //found marked edge, means, that we found cycle
                    side1circled = 1;
                }

                if (side2j == side1j) {
                    //found marked edge, this means that we found cycle
                    //mark both last edges as participating in joining
                    edge_side2.mark = 2;
                    edge_side1.mark = 2;
                    //add them to chains
                    addTW(edge_side2, passNumber);
                    addTW(edge_side1, 1 - passNumber);
            //*TODO:** goto found: GoTo lChainEnds;
                    break;
                }
            }

            //middle line projection vector
            //TODO: fix (not safe to 180/-180 edge)
            dx = side1j.lat - side1i.lat + side2j.lat - side2i.lat;
            dy = side1j.lon - side1i.lon + side2j.lon - side2i.lon;
            px = (side1i.lat + side2i.lat) * 0.5;
            py = (side1i.lon + side2i.lon) * 0.5;
            dd = 1 / (dx * dx + dy * dy);

            //remember current chain len
            checkchain = Chain.size();

            //create new node
            // создаю чуть раньше, т.к. ссылка не него используется в след. блоке
            Node createNode = new Node(-1);

            if (calc_side == 2) {
                //project j node from side2 to middle line
                dist_t = (side2j.lat - px) * dx + (side2j.lon - py) * dy;
                //addChain(side2j);
                Chain.add(side2j);
                //old node will collapse to this new one
                side2j.mark = createNode;   //Nodes.get(Nodes.size()-1);
            }
            else {
                //project j node from side1 to middle line
                dist_t = (side1j.lat - px) * dx + (side1j.lon - py) * dy;
                Chain.add(side1j);
                //old node will collapse to this new one
                side1j.mark = createNode;   //Nodes.get(Nodes.size()-1);
            }

            /*Nodes[NodesNum].Edges = 0;
            Nodes[NodesNum]..nodeID = -1;
            Nodes[NodesNum]..mark = -1;
            Nodes[NodesNum]..lat = px + dist_t * dx * dd;
            Nodes[NodesNum]..lon = py + dist_t * dy * dd;
            */
            createNode.lat = px + dist_t * dx * dd;
            createNode.lon = py + dist_t * dy * dd;

            //reproject prev node into current middle line ("ChainNum - 2" because ChainNum were updated above by AddChain)
            //j = Nodes[Chain[ChainNum - 2]]..mark;
            Node jNode = Chain.get(Chain.size() - 2).mark;
            dist_t = (jNode.lat - px) * dx + (jNode.lon - py) * dy;
            jNode.lat = px + dist_t * dx * dd;
            jNode.lon = py + dist_t * dy * dd;

            if (Node.distance(jNode, createNode /*Nodes.get(Nodes.size()-1)*/) < combineDistance) {
                //Distance from new node to prev-one is too small, collapse node with prev-one
                //TODO(?): averaging coordinates?
                if (calc_side == 2) {
                    side2j.mark = jNode;
                }
                else {
                    side1j.mark = jNode;
                }
                //do not call AddNode -> new node will die
            }
            else {
                //addNode();
                                // delete after fix
                createNode.VBNum = Nodes.size();
                
                Nodes.add(createNode);
                //fix order of nodes in chain
                fixChainOrder(checkchain);
            }

            //both sides circled - whole road is a loop
            if (side1circled > 0 && side2circled > 0) {
        //*TODO:** goto found: GoTo lFoundCycle;

//*TODO:** label found: lFoundCycle:;
                //handle cycle road

                //find all nodes from end of chain which is present in chain two times
                //remove all of them, except last one
                //in good cases last node should be same as first node
                //TODO: what if not?
                //for (int i = Chain.size(); i >= 0; i--) {
                int i = Chain.size()-1;
                while(true) {
                    Node iChain = Chain.get(i);
                    for (int j = 0; j < i; j++) {
                        if (iChain.equals(Chain.get(j))) {
            //*TODO:** goto found: GoTo lFound;
                            i--;
                            Chain.remove(iChain);
                            break;
                        }
                    }
                    //not found
                    //keep this node (which is one time in chain) and next one (which is two times)
                    // TODO: не понятка как это толком энтерпретировать. Переделал два фора, в один фор и ваил, где регулирование индекса идет в форе
                    //ChainNum = i + 2;
                    break;
                    //*TODO:** goto found: GoTo lExit;
        //*TODO:** label found: lFound:;
                }
            }

            //proceed to searching next edge
        //*TODO:** goto found: GoTo lKeepGoing;
        }

//*TODO:** label found: lChainEnds:;

        //Node there is chance, that circular way will be not closed from one of sides
        //Algorithm does not handle this case, it should collapse during juctions collapsing

//*TODO:** label found: lExit:;
    }
/*
    //Reverse array into backward direction
    public static void reverseArray(int[] arr, int num) { // TODO: Use of ByRef founded
        int i = 0;
        int j = 0;
        int t = 0;
        //half of len
        j = num / 2;
        for (i = 0; i <= j - 1; i++) {
            //swap elements from first and second halfs
            t = arr[i];
            arr[i] = arr[num - 1 - i];
            arr[num - 1 - i] = t;
        }
    }
*/
    //Add edge into one of TW arrays
    //side: 0 - into TWforw, 1 - into TWback
    public static void addTW(Edge edge1, int side) {
        if (side == 1) {
            TWback.add(edge1);
            /*TWback[TWbackNum] = edge1;
            TWbackNum = TWbackNum + 1;
            if (TWbackNum >= TWalloc) {
                //*TODO:** goto found: GoTo lRealloc;
            }*/
        } else {
            TWforw.add(edge1);
            /*TWforw[TWforwNum] = edge1;
            TWforwNum = TWforwNum + 1;
            if (TWforwNum >= TWalloc) {
//*TODO:** label found: lRealloc:;
                //realloc if needed
                TWalloc = TWalloc * 2;
                G.redimPreserve(TWforw, TWalloc);
                G.redimPreserve(TWback, TWalloc);
            }*/
        }
    }
    /*
    //Find index of node1 in Chain() array, return -1 if not present
    public static int findInChain(Node node1) {
        /*int i = 0;
        
        for (i = 0; i < Chain.size(); i++) {
            if (Chain[i] == node1) { return 0; }
        }*//*
        if (Chain.contains(node1)) { return Chain.indexOf(node1); }
        return -1;
    }
    */

    //Fix order of nodes in Chain
    //Fixing is needed when last node is not new arrow-head of GoByTwoWays algorithm (ex. several short edges of one side, but long edge of other side)
    public static void fixChainOrder(int checkindex) {

        Node i2, i1, i0;
        int k;
        double p;
        //2 or less nodes in chain, nothing to fix
        if (checkindex < 2) { return; }

        //last new node
        i2 = Chain.get(checkindex).mark;
        //exit in case of probles
        if (i2 == null) { return; }
        //prev new node
        i1 = Chain.get(checkindex - 1).mark;
        if (i1 == null) { return; }
        //prev-prev new node
        i0 = Chain.get(checkindex - 2).mark;
        if (i0 == null) { return; }

        k = 3;
        //if prev-prev new nodes is combined with prev new node - find diffent node backward
        while (i0.equals(i1)) {
            //reach Chain(0)
            if (checkindex < k) { return; }
            i0 = Chain.get(checkindex - k).mark;
            k = k + 1;
        }

        //Scalar multiplication of vectors i0->i1 and i1->i2
        p = (i2.lat - i1.lat) * (i1.lat - i0.lat) + (i2.lon - i1.lon) * (i1.lon - i0.lon);

        if (p < 0) {
            //vectors are contradirectional -> swap
            i0 = Chain.get(checkindex);
            Chain.set(checkindex, Chain.get(checkindex - 1));
            Chain.set(checkindex - 1, i0);
            //check last new node on new place
            fixChainOrder(checkindex - 1);
        }
    }

    public static String getLabelByStats(int flags) {
        //GetLabelByStats = GetLabelByStats1(Text) majoritary version
        //combinatory version
        // TODO: флаг пока не использовался
        return LabelStat.getLabelByStats2(LabelStats);
    }

    //Add label to label stats
    public static void addLabelStat0(String text) {
        //Call AddLabelStat1(Text) majoritary version
        //combinatory version
        LabelStat.addLabelStat2(text, LabelStats);
    }

    //Remove all labels stats from memory
    public static void resetLabelStats() {
        LabelStats.clear();
    }

    //Reverse array into backward direction
    public static void reverseArray(ArrayList arr) { // TODO: Use of ByRef founded
        int i;
        int j;
        Object t;
        int num = arr.size();
        //half of len
        j = num / 2;
        for (i = 0; i < j; i++) {
            //swap elements from first and second halfs
            t = arr.get(i);
            arr.set(i, arr.get(num - 1 - i));
            arr.set(num - 1 - i, t);
        }
    }

    //Delete edges which connect node with itself
    public static void filterVoidEdges() {
        int i = 0;
        for (i = 0; i < Edges.size(); i++) {
            Edge edgeI = Edges.get(i);
            if ((edgeI.node1 != null) && (edgeI.node1 == edgeI.node2)) {
                edgeI.delEdge();
                // TODO возможно стоит добавить удаление сомого edgeI
            }
        }
    }

    //Save geometry to .mp file with joining chains into polylines
    public static void save_MP_2(String filename) {
        int i = 0;
        int k1, k2;
        int typ = 0;

        //Open(filename For Output As #2);
        BufferedWriter bw;
        FileOutputStream fos;
        try {
            fos = new FileOutputStream(filename);
            bw = new BufferedWriter(new OutputStreamWriter(fos, "CP1251"));

            bw.write("; Generated by mp_extsimp (java)\r\n");
            bw.newLine();
            bw.write(MPheader);
            bw.newLine();

            for(Iterator<Edge> kEdge = Edges.iterator(); kEdge.hasNext();) {
                Edge kEdgeN = kEdge.next();
                if (kEdgeN.node1 == null) {
                    //deleted edge
                    //mark to ignore
                    kEdgeN.mark = 1;
                } else {
                    //mark to save
                    kEdgeN.mark = 0;
                }
            }
            numFormat = new DecimalFormat("0.#######");
            DecimalFormatSymbols dfs = new DecimalFormatSymbols();
            dfs.setDecimalSeparator('.');
            numFormat.setDecimalFormatSymbols(dfs);
                    
            for(Iterator<Edge> kEdge = Edges.iterator(); kEdge.hasNext();) {
                Edge kEdgeN = kEdge.next();
                if (kEdgeN.mark == 0) {
                    //all marked to save - find chain and save
                    bw.write(saveChain(kEdgeN));
                }
            }

            //file finalization flag
            bw.write("; Completed\r\n");

            bw.close();
        } catch (FileNotFoundException ex) {
            System.err.println(ex.getLocalizedMessage() + " : " + filename);
        } catch (IOException ex) {
            System.err.println(ex.getLocalizedMessage());
        }
    }

    //Save chain of edges into mp file (already opened as #2)
    public static String saveChain(Edge edge1) {

        String result = "";
        int m;
        boolean chainEnd = false;
        Edge nextChainEdge = null;

        int k1, k2;

        Edge refEdge;

        //Algorithm go from specified edge into one direction by chain of nodes
        //(nodes connected one by one, without junctions) until end (or junction) is reached
        //After that algorithm will go from final edge into opposite direction and will compare edges
        //and add nodes into Chain array
        //On findind different edge (or reaching other end of chain) algorithm will save found (sub)chain into mp file
        //Then rest of chain (if it exits) will be processed in similar way

        //1) go by chain to the one end - to node with !=2 edges

        //start node
        Node nodeI = edge1.node1;
        Node nodeJ = edge1.node2;
        Node nodeK;

        if (nodeI.edgeL.size() != 2) {
            //i is end of chain
            //ChainNum = 0;
            Chain = new ArrayList<Node>();
            //addChain(i);
            //addChain(j);
            Chain.add(nodeI);
            Chain.add(nodeJ);
            refEdge = edge1;
            
            //saved
            edge1.mark = 1;

            if (nodeJ.edgeL.size() != 2) {
                //that's all
                chainEnd = true;
                result += saveChainInString(refEdge);
        //*TODO:** goto found: GoTo lBreak;
            }
            else {
                nodeJ = Chain.get(0);
                nodeI = Chain.get(1);
        //*TODO:** goto found: GoTo lGoNext2;
            }
        } else {

    //*TODO:** label found: lGoNext:;
            while (true) {
                //go by chain
                nodeK = goByChain(nodeI, nodeJ);
                //if still 2 edges - proceed
                if (nodeK.edgeL.size() == 2) {
                    nodeJ = nodeI;
                    nodeI = nodeK;
            //GoTo lGoNext;
                } else { break; }
            }

            //   *-----*-----*-----*---...
            //   k     i     j

            //OK, we found end of chain
            nodeJ = nodeK;

            //   *---------*-----*-----*---...
            //  k=j        i

            //2) go revert - from found end to another one and saving all nodes into Chain() array

            //ChainNum = 0;
            Chain = new ArrayList<Node>();
            //addChain(k);
            //addChain(i);
            Chain.add(nodeK);
            Chain.add(nodeI);

            //keep info about first edge in chain
            refEdge = new Edge(GoByChain_lastedge);
            GoByChain_lastedge.mark = 1;
            //reversed oneway
            if (refEdge.node1 != Chain.get(0) && refEdge.oneway == 1) { refEdge.oneway = 2; }
        }
//*TODO:** label found: lGoNext2:;

        while (!chainEnd) {
            nodeK = goByChain(nodeI, nodeJ);

            //   *-------------*-----*-----*---...
            //  j              i     k

            //check oneway
            m = GoByChain_lastedge.oneway;
            if (m > 0 && GoByChain_lastedge.node1 != nodeI) { m = 2; }

            //if oneway flag is differnt or road type, speed or label is changed - break chain
            if (m != refEdge.oneway) {
                nextChainEdge = GoByChain_lastedge;
        //GoTo lBreak;
            }
            if (GoByChain_lastedge.roadtype != refEdge.roadtype) {
                nextChainEdge = GoByChain_lastedge;
        //GoTo lBreak;
            }
            if (GoByChain_lastedge.speed != refEdge.speed) {
                nextChainEdge = GoByChain_lastedge;
        //GoTo lBreak;
            }
            if (!(GoByChain_lastedge.label.equals(refEdge.label))) {
                nextChainEdge = GoByChain_lastedge;
        //GoTo lBreak;
            }

            if (nextChainEdge != GoByChain_lastedge) {
                //saved
                GoByChain_lastedge.mark = 1;

                //addChain(k);
                Chain.add(nodeK);

                if (nodeK.edgeL.size() == 2) {
                    //still 2 edges - still chain
                    nodeJ = nodeI;
                    nodeI = nodeK;
                    continue;
            //*TODO:** goto found: GoTo lGoNext2;
                } else {
                    chainEnd = true;
                }
            }

    //*TODO:** label found: lBreak:;
            //3) save chain to file

            result += saveChainInString(refEdge);

            if (!chainEnd) {
                //continue with this chain, as it is not ended

                //   *================*--------------------*-----------*-----*---...
                //                        NextChainEdge

                //new reference info
                refEdge = nextChainEdge;
                if (refEdge.node1 == Chain.get(Chain.size()-1)) {
                    nodeI = refEdge.node2;
                    nodeJ = refEdge.node1;
                }
                else {
                    if (refEdge.oneway == 1) { refEdge.oneway = 2; }
                    nodeI = refEdge.node1;
                    nodeJ = refEdge.node2;
                }

                //   *================*--------------------*-----------*-----*---...
                //                    j                    i

                nextChainEdge.mark = 1;

                //add both nodes of last edge
                //ChainNum = 0;
                //addChain(j);
                //addChain(i);
                Chain = new ArrayList<Node>();
                Chain.add(nodeJ);
                Chain.add(nodeI);
                if (nodeI.edgeL.size() != 2) {
                    //chain from one edge
                    chainEnd = true;
                    result += saveChainInString(refEdge);
                    continue; //break;  // без разницы флаг конца установлен

        //*TODO:** goto found: GoTo lBreak;
                } else {

                    nextChainEdge = null;
                    continue;
                }
                //continue with chain
        //*TODO:** goto found: GoTo lGoNext2;
            }
        }

        return result;
    }

    //Go by chain from node1 in some direction, but not to Node0
    //(assumed, that node1 have two edges, not 1, not 3 or more, otherwise - UB)
    //Usage: GoByChain(x,x) goes by first edge, z=GoByChain(x,y)->u=GoByChain(z,x)->... allows to travel by chain node by node
    public static Node goByChain(Node node1, Node node0) {
        //check first edge
        Edge i = node1.edgeL.get(0);
        Node k = i.node1;
        if (k == node1) { k = i.node2; }
        GoByChain_lastedge = i;
        if (k == node0) {
            //node0 -> check second edge
            i = node1.edgeL.get(1);
            k = i.node1;
            if (k == node1) { k = i.node2; }
            GoByChain_lastedge = i;
        }
        return k;
    }

    private static String saveChainInString(Edge refEdge) {
        String result = "";
        //Print #2, "; roadtype=" + CStr(refedge.roadtype) 'debug info about road type
        //Print #2, "[POLYLINE]";
        result += "[POLYLINE]\r\n";
        //object type - from road type
        int typ = Highway.getType_by_Highway(refEdge.roadtype);
        //Print #2, "Type=0x"; Hex(typ);
        result += String.format("Type=0x%1$X\r\n", typ);
        if (refEdge.label.length() > 0) {
            //labels - into special codes fro labelization
            //Print #2, "Label=~[0x05]" + refEdge.label;
            //Print #2, "StreetDesc=~[0x05]" + refEdge.label;
            result += "Label=~[0x05]" + refEdge.label + "\r\n";
            result += "StreetDesc=~[0x05]" + refEdge.label + "\r\n";
        }
        //oneway indicator
        if (refEdge.oneway > 0) {
            //Print #2, "DirIndicator=1";
            result += "DirIndicator=1\r\n";
        }
        //top level of visibility - from road type
        //Print #2, "EndLevel=" + CStr(GetTopLevel_by_Highway(refEdge.roadtype));
        //Print #2, "RouteParam=";
        result += "EndLevel=" + Highway.getTopLevel_by_Highway(refEdge.roadtype) + "\r\n";
        result += "RouteParam=";
        //speed class
        //Print #2, CStr(refEdge.speed); ",";
        result += refEdge.speed + ",";
        //road class - from road type
        //Print #2, CStr(GetClass_by_Highway(refEdge.roadtype)); ",";
        result += Highway.getClass_by_Highway(refEdge.roadtype) + ",";
        if (refEdge.oneway > 0) {
            //one_way
            //Print #2, "1,";
            result += "1,";
        } else {
            //Print #2, "0,";
            result += "0,";
        }
        //other params are not handled
        //Print #2, "0,0,0,0,0,0,0,0,0";
        result += "0,0,0,0,0,0,0,0,0\r\n";
        //Print #2, "Data0=";
        result += "Data0=";
        if (refEdge.oneway == 2) {
            //reverted oneway, save in backward sequence
            /*for (i = ChainNum - 1; i <= 0; i--) {
                if (i != ChainNum - 1) { Print #2, ","; }
                Print #2, "("; CStr(Nodes(Chain(i)).lat); ","; CStr(Nodes(Chain(i)).lon); ")";
            }*/
            for(int i = Chain.size() - 1; i >= 0; i--) {
                Node iChain = Chain.get(i);
                if (i != Chain.size() - 1) { result += ","; }
                result += "(" + numFormat.format(iChain.lat) + "," + numFormat.format(iChain.lon) +")";
            }
            //Print #2,;
            result += "\r\n";
            //Print #2, "Nod1=0,"; CStr(Chain(ChainNum - 1)); ",0";
            //Print #2, "Nod2=" + CStr(ChainNum - 1) + ","; CStr(Chain(0)); ",0";
            result += String.format("Nod1=0,%d,0\r\n", Nodes.indexOf(Chain.get(Chain.size()-1)));
            result += String.format("Nod2=%d,%d,0\r\n", Chain.size()-1, Nodes.indexOf(Chain.get(0)));
        }
        else {
            //forward oneway or twoway, save in direct sequence
            /*for (i = 0; i <= ChainNum - 1; i++) {
                if (i != 0) { Print #2, ","; }
                Print #2, "("; CStr(Nodes(Chain(i)).lat); ","; CStr(Nodes(Chain(i)).lon); ")";
            }
            Print #2,;
            Print #2, "Nod1=0,"; CStr(Chain(0)); ",0";
            Print #2, "Nod2=" + CStr(ChainNum - 1) + ","; CStr(Chain(ChainNum - 1)); ",0";
            */
            for(int i = 0; i < Chain.size(); i++) {
                Node iChain = Chain.get(i);
                if (i != 0) { result += ","; }
                //result += String.format("(%.7f,%.7f)", iChain.lat, iChain.lon);
                result += "(" + numFormat.format(iChain.lat) + "," + numFormat.format(iChain.lon) +")";
            }
            result += "\r\n";
            result += String.format("Nod1=0,%d,0\r\n", Nodes.indexOf(Chain.get(0)));
            result += String.format("Nod2=%d,%d,0\r\n", Chain.size()-1, Nodes.indexOf(Chain.get(Chain.size()-1)));
        }
        //Print #2, "[END]";
        //Print #2, "";
        result += "[END]\r\n";
        result += "\r\n";

        return result;
    }

    //find and optimize all chains by Douglas-Peucker with Epsilon (in metres) and limiting max edge (in metres)
    public static void douglasPeucker_total_split(double epsilon, double maxEdge) {
        for (Iterator<Node> iNode = Nodes.iterator(); iNode.hasNext();) {
            //mark all nodes as not passed
            iNode.next().mark = null;
        }
        for (Iterator<Node> iNode = Nodes.iterator(); iNode.hasNext();) {
            Node nodeI = iNode.next();
            if (nodeI.nodeID == Mark.MARK_NODEID_DELETED  || nodeI.edgeL.size() != 2  || nodeI.mark != null) { // nodeI.mark == 1
                //*TODO:** goto found: GoTo lSkip;
            } else {
            //node: not deleted, not yet passed and with 2 edges -> should be checked for chain
                douglasPeucker_chain_split(nodeI, epsilon, maxEdge);
            }
//*TODO:** label found: lSkip:;
/*            if ((i & 8191) == 0) {
                //show progress
                Form1.Caption = "Doug-Pek sp " + CStr(i) + " / " + CStr(NodesNum): Form1.Refresh;
            }*/
        }
    }

    //find one chain (starting from node1) and optimize it by Douglas-Peucker with Epsilon (in metres) and limiting edge len by MaxEdge
    // Ну ооочень похоже на SaveChain
    public static void douglasPeucker_chain_split(Node node1, double epsilon, double maxEdge) {
        Node nodeI, nodeJ, nodeK;
        Edge refEdge;
        boolean chainEnd = false;
        Edge nextChainEdge = null;
        int m;

        //Algorithm works as DouglasPeucker_chain above
        //difference is only inside OptimizeByDouglasPeucker_One_split

        //1) go by chain to the one end - to node with !=2 edges

        //start node
        nodeI = node1;
        nodeJ = node1;
//*TODO:** label found: lGoNext:;
        while (true) {
            //go by chain
            nodeK = goByChain(nodeI, nodeJ);
            //'if still 2 edges - proceed
            if (nodeK != node1 && nodeK.edgeL.size() == 2) {
                nodeJ = nodeI; nodeI = nodeK;
                //GoTo lGoNext;
            } else { break; }
        }
        //   *-----*-----*-----*---...
        //   k     i     j

        //OK, we found end of chain
        nodeJ = nodeK;

        //   *---------*-----*-----*---...
        //  k=j        i

        //2) go revert - from found end to another one and saving all nodes into Chain() array

        //ChainNum = 0;
        Chain = new ArrayList<Node>();
        //addChain(k);
        //addChain(i);
        Chain.add(nodeK);
        Chain.add(nodeI);

        //keep info about first edge in chain
        refEdge = new Edge(GoByChain_lastedge);
        //reversed oneway
        if (refEdge.node1 != Chain.get(0) && refEdge.oneway == 1) { refEdge.oneway = 2; }

//*TODO:** label found: lGoNext2:;
        while (!chainEnd) {
            nodeK = goByChain(nodeI, nodeJ);

            //   *-------------*-----*-----*---...
            //  j              i     k

            //check oneway
            m = GoByChain_lastedge.oneway;
            if (m > 0 && GoByChain_lastedge.node1 != nodeI) { m = 2; }

            //if oneway flag is differnt or road type is changed - break chain
            if (m != refEdge.oneway) {
                nextChainEdge = GoByChain_lastedge;
        //GoTo lBreak;
            }
            if (GoByChain_lastedge.roadtype != refEdge.roadtype) {
                nextChainEdge = GoByChain_lastedge;
        //GoTo lBreak;
            }

            if (nextChainEdge != GoByChain_lastedge) {
                Chain.add(nodeK);

                if (nodeK != Chain.get(0) && nodeK.edgeL.size() == 2) {
                    //still 2 edges - still chain
                    nodeK.mark = new Node(-1);
                        nodeJ = nodeI;
                        nodeI = nodeK;
                        continue;
            //*TODO:** goto found: GoTo lGoNext2;
                } else {
                    chainEnd = true;
                }
            }

//*TODO:** label found: lBreak:;

            //3) optimize found chain by D-P
            //optimizeByDouglasPeucker_One_split(0, ChainNum - 1, epsilon, refEdge, maxEdge);
            optimizeByDouglasPeucker_One_split(Chain.get(0), Chain.get(Chain.size()), epsilon, refEdge, maxEdge);

            if (!chainEnd) {
                //continue with this chain, as it is not ended

                //   *================*--------------------*-----------*-----*---...
                //                        NextChainEdge

                //new reference info
                refEdge = nextChainEdge;
                if (refEdge.node1 == Chain.get(Chain.size()-1)) {
                    nodeI = refEdge.node2;
                    nodeJ = refEdge.node1;
                }
                else {
                    if (refEdge.oneway == 1) { refEdge.oneway = 2; }
                    nodeI = refEdge.node1;
                    nodeJ = refEdge.node2;
                }

                //   *================*--------------------*-----------*-----*---...
                //                    j                    i

                //chain from one edge - nothing to optimize by D-P
                if (nodeI.edgeL.size() != 2) { return; }

                //add both nodes of last edge
                Chain = new ArrayList<Node>();
                Chain.add(nodeJ);
                Chain.add(nodeI);

                nextChainEdge = null;

                //continue with chain
        //*TODO:** goto found: GoTo lGoNext2;
            }
        }
    }

    //Recursive check to optimize chain/subchain by Douglas-Peucker with Epsilon (in metres) and limiting edge len by MaxEdge
    //subchain is defined by IndexStart,IndexLast
    //refEdge - road parameters of chain (for create new edge in case of optimization)
    //(180/-180 safe)
    private static void optimizeByDouglasPeucker_One_split(Node nodeStart, Node nodeLast, double epsilon, Edge refEdge, double maxEdge) {
        int i = 0;
        Node farestNode = null;
        double farestDist = 0;
        double dist = 0;
        double k = 0;
        double scalarMult = 0;
        int newspeed = 0;
        String newlabel = "";
        int indexStart = Chain.indexOf(nodeStart);
        int indexLast = Chain.indexOf(nodeLast);

        //one edge (or less) -> nothing to do
        if (((indexStart + 1) >= indexLast)) { return; }

        //distance between subchain edge
        k = Node.distance(nodeStart, nodeLast);

        //find node, farest from line first-last node (farer than Epsilon)
        //start max len - Epsilon
        farestDist = epsilon;
        //nothing yet found
        for (i = indexStart + 1; i < indexLast; i++) {
            Node nodeI = Chain.get(i);
            if (k == 0) {
                //circled subchain
                dist = Node.distance(nodeI, nodeStart);
            }
            else {
                dist = Node.distanceToSegment(nodeStart, nodeLast, nodeI);
            }
            if (dist > farestDist) {
                farestDist = dist; farestNode = nodeI;
            }

            if (Node.distance(nodeI, nodeStart) > maxEdge) {
                //distance from start to this node is more than limit -> we should keep this node
                farestNode = nodeI;
    //*TODO:** goto found: GoTo lKeepFar;
                // т.к. farestNode != null след. условие пропускается и переходим как раз на нужную метку
                break;
            }
        }

        if (farestNode == null) {
            //farest node not found -> all distances less than Epsilon -> remove all internal nodes

            //calc speed and label from all subchain edges
            estimateChain(indexStart, indexLast);
            newspeed = EstimateChain_speed;
            newlabel = EstimateChain_label;

            for (i = indexStart + 1; i < indexLast; i++) {
                //kill with edges
                Chain.get(i).delNode();
            }
            Edge edgeI;
            //join first and last nodes by new edge
            if (refEdge.oneway == 2) {
                //reversed oneway
                edgeI = Edge.joinByEdge(nodeLast, nodeStart);
                edgeI.oneway = 1;
            }
            else {
                edgeI = Edge.joinByEdge(nodeStart, nodeLast);
                edgeI.oneway = refEdge.oneway;
            }
            edgeI.roadtype = refEdge.roadtype;
            edgeI.speed = (byte)newspeed;
            edgeI.label = newlabel;

            return;
        }

//*TODO:** label found: lKeepFar:;
        //farest point found - keep it
        //call Douglas-Peucker for two new subchains
        //Douglas-Peucker for two new subchains
        optimizeByDouglasPeucker_One_split(nodeStart, farestNode, epsilon, refEdge, maxEdge);
        optimizeByDouglasPeucker_One_split(farestNode, nodeLast, epsilon, refEdge, maxEdge);

    }

    //Calc speedclass and label of combined subchain of edges
    public static void estimateChain(int indexStart, int indexLast) {

        for (int i = 0; i <= 10; i++) {
            SpeedHistogram[i] = 0;
        }

        EstimateChain_label = "";
        EstimateChain_speed = 0;
        resetLabelStats();

        for (int i = indexStart; i < indexLast; i++) {
            Edge edgeJ = Edge.getEdgeBetween(Chain.get(i), Chain.get(i + 1));
            if (edgeJ != null) {
                //add label of edge into stats
                addLabelStat0(edgeJ.label);
                //add speed into histogram
                SpeedHistogram[edgeJ.speed]++;
            }
        }

        //estimate speed
        EstimateChain_speed = Highway.estimateSpeedByHistogram(SpeedHistogram);
        //calc resulting label
        EstimateChain_label = getLabelByStats(0);
    }
}
