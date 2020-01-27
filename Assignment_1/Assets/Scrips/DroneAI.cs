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

    private DroneController m_Drone; // the car controller we want to use
    SphereCollider droneCollider;
    public float radiusMargin;

    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    Graph DroneGraph;
    //All edges have same length:
    float edgeLength = 3.0f;
    float edgeMaxLength = 3.0f + 1.0f;
    private void Start()
    {

        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        droneCollider = GetComponent<SphereCollider>();
        float radiusMargin = droneCollider.radius + 0.5f;

        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();



        Vector3 start_pos = terrain_manager.myInfo.start_pos;
        Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

        Node StartNode = new Node(start_pos.x, start_pos.z);

        DroneGraph = new Graph(StartNode);

        //List<Vector3> my_path = new List<Vector3>();


        // Plan your path here
        // ...
        RRG(1000, DroneGraph);
        
        for (int i=0; i< 50; i++){
                GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                Vector3 position = DroneGraph.getNode(i).getPosition();
                
                cube.transform.position = new Vector3(position.x, 1, position.z);
                cube.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);

            Debug.Log(position);
                
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
        float[,] radiusHelpMatrix = new float[,] { { radiusMargin, 0.0f }, { -radiusMargin, 0.0f }, { 0.0f, radiusMargin }, { 0.0f, -radiusMargin } };
        for (int a = 0; a < 4; a = a + 1)
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
        bool hit = Physics.Raycast(from, to - from, edgeLength);
        if (hit) { return true; }
        return false;
    }


    Vector3 Pseudo_random(float radiusMargin)
    {

        float cordx = 50.0f; float cordz = 50.0f;
        bool foundNonColidingPos = false;
        float[,] radiusHelpMatrix = new float[,] { { radiusMargin, 0.0f }, { -radiusMargin, 0.0f }, { 0.0f, radiusMargin }, { 0.0f, -radiusMargin } };
        while (foundNonColidingPos == false)
        {
            cordx = UnityEngine.Random.Range(terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high);
            cordz = UnityEngine.Random.Range(terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high);
            bool traversbel = true;
            for (int a = 0; a < 4; a = a + 1)
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
        Dictionary<int, Node> nodes = new Dictionary<int, Node>();
        Dictionary<int, List<int>> adjList = new Dictionary<int, List<int>>();
        List<int> endNodes = new List<int>();

        int size;

        public Graph() { }

        // The following constructor has parameters for two of the three 
        // properties. 
        public Graph(Node _StartNode)
        {
            size = 0;
            int id = size++;
            nodes.Add(id, _StartNode);
            adjList.Add(id, new List<int>());
        }

        public Dictionary<int, Node> getNodes()
        {
            return nodes;
        }
        public int addNode(Node _newNode)
        {
            int id = size++;
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
            adjList.Add(_id, _adjList);
        }
        public void addEdge(int _idA, int _idB)
        {
            List<int> actualList;

            actualList = adjList[_idA];
            actualList.Add(_idB);
            setAdjList(_idA, actualList);

            actualList = adjList[_idB];
            actualList.Add(_idA);
            setAdjList(_idB, actualList);
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


        public Node FindClosestNode(Vector3 target)
        {

            Node temp;
            Node closest = nodes[0];//root
            float closestDistance = Vector3.Distance(closest.getPosition(), target);
            float checkDistance = 0f;
            foreach (KeyValuePair<int, Node> node in nodes)
            {
                temp = node.Value;
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
        Node close_node=null;
        Vector3 new_coord= new Vector3(0,0,0);
        for (int i = 0; i<max_nodes; i++)
        {
            bool found = false;
            while (!found)
            {
                Vector3 goal = Pseudo_random(radiusMargin);
                close_node = G.FindClosestNode(goal);
                float distance = Vector3.Distance(close_node.getPosition(), goal);

                new_coord = Vector3.Lerp(close_node.getPosition(), goal, edgeLength / distance);
                if (position_collision(radiusMargin, new_coord) && IsCollidingOnEdge(new_coord, close_node.getPosition()))
                {
                    found = true;
                }
            }



            int idx = G.addNode(new Node(new_coord));
            G.addEdge(idx, close_node.getId());
            foreach (KeyValuePair<int, Node> node in G.getNodes())
            {
                Node temp = node.Value;
                float checkDistance = Vector3.Distance(temp.getPosition(), G.getNode(idx).getPosition());
                if (checkDistance < edgeMaxLength && node.Key != idx && IsCollidingOnEdge(temp.getPosition(), G.getNode(idx).getPosition()))
                {
                    G.addEdge(node.Key, idx);
                }
            }
        }
    }
    
}
