using System;

using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    int counter = 0;
    private DroneController m_Drone; // the car controller we want to use
    SphereCollider droneCollider;
    public float radiusMargin;

    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    private Graph DroneGraph;
    //All edges have same length:
    public float edgeLength = 5.0f;
    public float edgeMinDist = 0.1f;
    public float addEdgeMaxLength =  2.5f;

    int randomTimer = 0;
    Vector3 goal_pos;
    private void Start()
    {
        int n = 500;
        Debug.Log("Starting");

        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        droneCollider = GetComponent<SphereCollider>();
        float radiusMargin = droneCollider.radius + 0.5f;

        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();



        Vector3 start_pos = terrain_manager.myInfo.start_pos;
        goal_pos = terrain_manager.myInfo.goal_pos;

        Node StartNode = new Node(new Vector3(start_pos.x, 1, start_pos.z));
        Debug.Log("Creating the graph");

        DroneGraph = new Graph();
        DroneGraph.addNode(StartNode);

        //List<Vector3> my_path = new List<Vector3>();


        // Plan your path here
        // ...
        
        RRG(n, DroneGraph);
        
        for (int i=0; i< DroneGraph.getSize(); i++){
                GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                Collider c = cube.GetComponent<Collider>();
                c.enabled = false;
                Vector3 position = DroneGraph.getNode(i).getPosition();
            //Debug.Log("Node n." + i.ToString());
            //foreach (object o in DroneGraph.getAdjList(i))
            //{
             //   Debug.Log(o);
            //}
            cube.transform.position = new Vector3(position.x, 1, position.z);
                cube.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
                for (int j = 0; j < DroneGraph.getAdjList(i).Count(); j++){
                    Debug.DrawLine(DroneGraph.getNode(i).getPosition(), DroneGraph.getNode(DroneGraph.getAdjList(i)[j]).getPosition(), Color.blue, 100f);
                    //Debug.Log(DroneGraph.getNode(i).getPosition() +" - "+ DroneGraph.getNode(j).getPosition());
            }
            Debug.Log("Adj list of node " + i.ToString());
            var strings = string.Join(", ", DroneGraph.getAdjList(i));
            Debug.Log(strings);
        }
        Node goalN=DroneGraph.FindClosestNode(goal_pos, DroneGraph);
        int goal_idx = goalN.getId();
        List<List<int>> paths;
        paths=dfs(DroneGraph, goal_idx);


         
  

        

        foreach (var p in paths)
        {
            Color color = Color.white;
            color.r = UnityEngine.Random.Range(0f, 1f);
            color.g = UnityEngine.Random.Range(0f, 1f);
            color.b = UnityEngine.Random.Range(0f, 1f);
            Vector3 old_wp = start_pos;
            foreach (var wp in p)
            {


                Debug.DrawLine(old_wp, DroneGraph.getNode(wp).getPosition(), color, 100f);
                old_wp = DroneGraph.getNode(wp).getPosition();
            }
        }
        //my_path.Add(start_pos);

        //for (int i = 0; i < 200; i++)
        // {
        //   Vector3 waypoint = Pseudo_random(radiusMargin);
        //my_path.Add(waypoint);


        //}

        //my_path.Add(goal_pos);



        // Plot your path to see if it makes sense
        //Vector3 old_wp = start_pos;
        //foreach (var wp in my_path)
        //{
        //    Debug.DrawLine(old_wp, wp, Color.blue, 100f);
        //    old_wp = wp;
        //}
        //

    }

    //void onDrawGizmos(Vector3 position) {
    //    Gizmos.color = Color.red;
    //    Gizmos(position, 1f);
    //}

    private void FixedUpdate()
    {
        // Execute your path here
        // ...

        // this is how you access information about the terrain
        int i = terrain_manager.myInfo.get_i_index(transform.position.x);
        int j = terrain_manager.myInfo.get_j_index(transform.position.z);
        float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
        float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

        Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z), Color.white, 1f);

        // this is how you control the car
        m_Drone.Move(0.0f, 0.0f);

    }



    // Update is called once per frame
    void Update()
    {

    }

    bool position_collision(float radiusMargin, Vector3 position)
    {
        float[,] radiusHelpMatrix = new float[,] { { radiusMargin, radiusMargin }, { radiusMargin, -radiusMargin }, { -radiusMargin, radiusMargin }, { -radiusMargin, -radiusMargin }, { radiusMargin, 0.0f }, { -radiusMargin, 0.0f }, { 0.0f, radiusMargin }, { 0.0f, -radiusMargin } };
        for (int a = 0; a < 8; a = a + 1)
        {
            int i = terrain_manager.myInfo.get_i_index(position.x + radiusHelpMatrix[a, 0]);
            int j = terrain_manager.myInfo.get_j_index(position.z + radiusHelpMatrix[a, 1]);

            if (terrain_manager.myInfo.traversability[i, j] == 1.0f)
            {
                return true;
            }
        }
        return false;
    }

    bool IsCollidingOnEdge(Vector3 from, Vector3 to)
    {
        from.y = 3;
        to.y = 3;
        
        bool hit = Physics.Raycast(from, to - from, Vector3.Distance(from,to));
        if (hit) { 
            return true; }
        return false;
    }


    Vector3 Pseudo_random(float radiusMargin)
    {

        if (randomTimer == 10)
        {
            randomTimer = 0;
            return goal_pos;
        }
        randomTimer++;
        float cordx = 50.0f; float cordz = 50.0f;
        bool foundNonColidingPos = false;
        float[,] radiusHelpMatrix = new float[,] { { radiusMargin, radiusMargin }, { radiusMargin, -radiusMargin }, { -radiusMargin, radiusMargin}, { -radiusMargin, -radiusMargin }, { radiusMargin, 0.0f }, { -radiusMargin, 0.0f }, { 0.0f, radiusMargin }, { 0.0f, -radiusMargin } };
        while (foundNonColidingPos == false)
        {
            cordx = UnityEngine.Random.Range(terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high);
            cordz = UnityEngine.Random.Range(terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high);
            bool traversbel = true;
            for (int a = 0; a < 8; a = a + 1)
            {
                int i = terrain_manager.myInfo.get_i_index(cordx + radiusHelpMatrix[a, 0]);
                int j = terrain_manager.myInfo.get_j_index(cordz + radiusHelpMatrix[a, 1]);

                if (terrain_manager.myInfo.traversability[i, j] == 1.0f)
                {
                    traversbel = false;

                    break;
                }
            }
            foundNonColidingPos = traversbel;
        }

        //Debug.Log("x: "+cordx.ToString()+"| z: "+cordz.ToString());

        return new Vector3(cordx, 0, cordz);

    }

    public class Node
    {

        private int id;
        private double speedX, speedZ, AccellerationX, AccellerationZ, theta;
        private Vector3 position;
        private float x, z;
        private int color;
        public void setColor(int _c)
        {
            color = _c;
        }
        public int getColor() { return color; }
        public Vector3 getPosition()
        {
            return position;
        }

        public float getPositionX()
        {
            return position.x;
        }
        public float getPositionZ()
        {
            return position.z;
        }
        public int getId()
        {
            return id;
        }
        public Node(float _x, float _z)
        {
            x = _x;
            z = _z;
            position = new Vector3(_x, 0, _z);
            id = -1;
        }
        public Node(Vector3 _position)
        {
            position = _position;
            id = -1;
        }
        public Node()
        {
            x = 0;
            z = 0;
            id = -1;
        }
        public void setId(int _id)
        {
            id = _id;
        }
        public void setPositionX(float _x)
        {
            x = _x;
            position.x = _x;
        }
        public void setPositionZ(float _z)
        {
            z = _z;
            position.z = _z;
        }
        public void setTheta(double _theta)
        {
            theta = _theta;
        }
        public double getTheta()
        {
            return theta;
        }
    }

    public class Graph
    {
        Dictionary<int, Node> nodes;
        Dictionary<int, List<int>> adjList;
        List<int> endNodes;

        int size;

        public Graph() {
            Debug.Log("I am in the constructor");
        nodes =  new Dictionary<int, Node>();
        adjList =  new Dictionary<int, List<int>>();
        endNodes = new List<int>();
        size = 0;
        }

        // The following constructor has parameters for two of the three 
        // properties. 

        public void setColorOfNode(int _idx, int color)
        {
             nodes[_idx].setColor(color);
        }
        public int getColorOfNode(int _idx)
        {
            return nodes[_idx].getColor();
        }
        public Dictionary<int, Node> getNodes()
        {
            return nodes;
        }
        public int getSize()
        {
            return size;
        }
        public int addNode(Node _newNode)
        {
            int id = size++;
            _newNode.setId(id);
            _newNode.setColor(0);
            nodes.Add(id, _newNode);
            adjList.Add(id, new List<int>());
            return id;
        }
        public int addNode(Node _newNode, List<int> _adjList)
        {
            int id = size++;
            nodes.Add(id, _newNode);
            adjList.Add(id, _adjList);
            return id;
        }
        public Node getNode(int _id)
        {
            return nodes[_id];
        }
        public List<int> getAdjList(int _id)
        {
            return adjList[_id];
        }
        public void setAdjList(int _id, List<int> _adjList)
        {
            adjList[_id]= _adjList;
        }
        public void addEdge(int _idA, int _idB)
        {
            List<int> actualList;

            actualList = adjList[_idA];
            if (!actualList.Contains(_idB)) { 
                actualList.Add(_idB);
                setAdjList(_idA, actualList);
            }
            actualList = adjList[_idB];
            if (!actualList.Contains(_idA))
            {
                actualList.Add(_idA);
                setAdjList(_idB, actualList);
            }
        }

        public double computeAngle(Node _A, Node _B, Node _C)
        {
            double a_x = _A.getPositionX() - _B.getPositionX();
            double a_z = _A.getPositionZ() - _B.getPositionZ();
            double b_x = _B.getPositionX() - _C.getPositionX();
            double b_z = _B.getPositionX() - _C.getPositionZ();
            double num = a_x * b_x + a_z * b_z;
            double den = Math.Sqrt(a_x * a_x + a_z * a_z) * Math.Sqrt(b_x * b_x + b_z * b_z);
            double angle = Math.Acos(num / (den + 0.000001f));
            return angle;
        }
        public void setPathTheta(List<int> _path)
        {
            int A, B, C = 0;
            A = _path[0];
            nodes[A].setTheta(0);

            for (int i = 1; i < _path.Count() - 1; i++)
            {
                A = _path[i - 1];
                B = _path[i];
                C = _path[i + 1];
                nodes[B].setTheta(computeAngle(nodes[A], nodes[B], nodes[C]));
            }
            nodes[C].setTheta(0);

        }
        public double computePathCost(List<int> _path)
        {
            setPathTheta(_path); //Compute the angles of the nodes of that path

            double cost = 0;
            //Compute the max_speed that we can use to reach each node
            //compte the time space / speed for the path
            return cost;
        }


        public Node FindClosestNode(Vector3 target,Graph G)
        {

            Node temp;
            Node closest = nodes[0];//root
            float closestDistance = Vector3.Distance(closest.getPosition(), target);
            float checkDistance = 0f;
            
            for (int i=0; i < G.getSize(); i++)
            {
                temp = G.getNode(i);
                checkDistance = Vector3.Distance(temp.getPosition(), target);
                if (checkDistance < closestDistance)
                {
                    closestDistance = checkDistance;
                    closest = temp;
                }
            }
            return closest;
        }


    }
    public void RRG(int max_nodes,Graph G)
    {
        float edgeLength = 5.0f;
        float nodeMinDistance = 2.5f;
        float addEdgeMaxLength = 8.0f;
        float radiusMargin = droneCollider.radius + 0.5f;

        int max_iter=100;
        Node close_node=null;
        Vector3 new_coord= new Vector3(0,0,0);
        for (int i = 0; i<max_nodes && max_iter>0; i++)
        {
            //For all the new nodes;

            bool found = false;
            for (max_iter = 1000; !found && max_iter>0; max_iter--)
            {
                Vector3 goal = Pseudo_random(radiusMargin); //Find random point
                close_node = G.FindClosestNode(goal,G); //Find closest node
                float distance = Vector3.Distance(close_node.getPosition(), goal); //And compute the distance
                if (distance > edgeLength)
                {  //skip if B too close 
                    new_coord = Vector3.Lerp(close_node.getPosition(), goal, edgeLength / distance);
                    distance = Vector3.Distance(new_coord, G.FindClosestNode(new_coord, G).getPosition());
                    if (distance > nodeMinDistance)
                    {  //skip if C too close to another point
                        if (!position_collision(radiusMargin, new_coord) && !IsCollidingOnEdge(close_node.getPosition(), new_coord))
                        {
                            found = true;
                        }
                    }
                }
            }
            //Debug.Log(max_iter);


            int idx = G.addNode(new Node(new_coord));
            G.addEdge(idx, close_node.getId());
            for ( int j =0; j<G.getSize()-1; j++)
            {
                //if(i == j){continue;}

                Node temp = G.getNode(j);
                float checkDistance = Vector3.Distance(temp.getPosition(), G.getNode(idx).getPosition());
                if (checkDistance < addEdgeMaxLength && j != idx && !IsCollidingOnEdge(temp.getPosition(), G.getNode(idx).getPosition()))
                {
                    G.addEdge(j, idx);
                }
            }
        }
    }

    public void dfs_step(Graph G, int position, List<List<int>> paths,List<int> thisPath, int idx_goal)
    {
        
        G.getNode(position).setColor(1);
        thisPath.Add(position);
        if (counter>=100)
        {
            return;
        }
        if (position == idx_goal)
        {
            counter++;
            Debug.Log(counter);
            var strings = string.Join(", ", thisPath);
            Debug.Log(strings);

            List<int> good_path = new List<int>(thisPath);
            paths.Add(good_path);
            thisPath.RemoveAt(thisPath.Count - 1);
            G.getNode(position).setColor(0);
            return;
        }
        List<int> child = G.getAdjList(position);
        for(int i = 0; i < child.Count; i++)
        {
            if (G.getNode(child[i]).getColor() == 0) //not visited
            {
                dfs_step(G, child[i], paths, thisPath, idx_goal);
            }
        }
        thisPath.RemoveAt(thisPath.Count - 1);
        G.getNode(position).setColor(0);

    }
    public List<List<int>>  dfs(Graph G,int idx_goal)
    {

        List<List<int>> paths = new List<List<int>>();
        List<int> thisPath = new List<int>();
        dfs_step(G, 0, paths,thisPath, idx_goal);


        return paths;
    }

    /*List<int> DikDijkstra(Graph G):


    function Dijkstra(Graph, source):
 2
 3      create vertex set Q
 4
 5      for each vertex v in Graph:             
 6          dist[v] ← INFINITY                  
 7          prev[v] ← UNDEFINED                 
 8          add v to Q                      
10      dist[source] ← 0                        
11      
12      while Q is not empty:
13          u ← vertex in Q with min dist[u]    
14                                              
15          remove u from Q 
16          
17          for each neighbor v of u:           // only v that are still in Q
18              alt ← dist[u] + length(u, v)
19              if alt<dist[v]:               
20                  dist[v] ← alt 
21                  prev[v] ← u 
22
23      return dist[], prev[]*/


}
