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


    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    private Graph DroneGraph;
    //All edges have same length:
    public float edgeLength = 5.0f;
    public float edgeMinDist = 0.1f;
    public float addEdgeMaxLength =  2.5f;

    int randomTimer = 0;
    Vector3 goal_pos;
    int lastPointInPath=0;

    List<Node> my_path = new List<Node>();
    private void Start()
    {
        int n = 10000   ;
        Debug.Log("Starting");

        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        droneCollider = GetComponent<SphereCollider>();
        //radiusMargin = droneCollider.radius + 0.5f;

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
        killFuckers(DroneGraph);
        Debug.Log("Computing Distance to wall");
        computeDiStanceToWall(DroneGraph);
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
            //Debug.Log("Adj list of node " + i.ToString());
            //var strings = string.Join(", ", DroneGraph.getAdjList(i));
            //Debug.Log(strings);

        }
        Node goalN=DroneGraph.FindClosestNode(goal_pos, DroneGraph);
        int goal_idx = goalN.getId();
        List<int> path = new List<int>();
        //paths=dfs(DroneGraph, goal_idx);
        ASuperStar(DroneGraph, goal_idx);
        path = getBestPath(DroneGraph, goal_idx);


        Vector3 old_wp = start_pos;
        Debug.Log("Printing space now");
        foreach (var wp in path)
        {

            Debug.Log(Vector3.Distance(old_wp, DroneGraph.getNode(wp).getPosition()));
            Debug.DrawLine(old_wp, DroneGraph.getNode(wp).getPosition(), Color.red, 100f);
            old_wp = DroneGraph.getNode(wp).getPosition();
        }


        

        

        my_path = pathHelp(DroneGraph,path);
        setPathTheta(my_path);
        Debug.Log("Theta path:");
        var strings="";
        for (int i = 0; i < my_path.Count; i++)
        {
              strings=(strings +", " + my_path[i].getTheta().ToString());
            
        }
        Debug.Log(strings);
        Debug.Log("Done");
    }

    List<Node> pathHelp(Graph G,List<int> idList){
        List<Node> nodeList = new List<Node>();
        foreach(int i in idList){
            nodeList.Add(G.getNode(i));
        }
        return nodeList;
    }


    private void FixedUpdate()
    {
        int i;
        float radiusMargin = droneCollider.radius + 5.9f;
        DroneController controller = transform.GetComponent<DroneController>();
        RaycastHit rayHit;

        /*for (; lastPointInPath < my_path.Count && !((bool)Physics.SphereCast(transform.position, radiusMargin, my_path[lastPointInPath].getPosition() - transform.position, out rayHit, Vector3.Distance(my_path[lastPointInPath].getPosition(), transform.position)))
         ; lastPointInPath++); // increase to find the farest node

        lastPointInPath--;
        */
        Vector3 target = my_path[lastPointInPath].getPosition();
        
        int targetId = 0;
        float minDistance = 100;
        float actualDistance = 100;


            for (i = 0; i < my_path.Count(); i ++)
            {
                actualDistance = Vector3.Distance(my_path[i].getPosition(), transform.position);
                if ( minDistance> actualDistance)
                {
                    lastPointInPath = i;
                    minDistance = actualDistance;
                }
            }
            Debug.Log("I am close to node " + lastPointInPath.ToString());

        
        if (lastPointInPath != my_path.Count() - 1)
        {
            target = my_path[lastPointInPath + 1].getPosition();
            targetId = lastPointInPath + 1;
            
                        for (i = lastPointInPath + 1; i < my_path.Count(); i ++)
                        {
                            float newDistance = Vector3.Distance(transform.position, my_path[i].getPosition());
                            float distanceToTargetTemp = Vector3.Distance(transform.position, target);
                            if (distanceToTargetTemp > newDistance)
                            {
                                distanceToTargetTemp = newDistance;
                                target = my_path[i].getPosition();
                                targetId = i;
                            }
                        }
           /* for (i=lastPointInPath+1; i < my_path.Count() ; i++)
            {

                var highdronePosition = transform.position;
                var lowdronePosition = transform.position;
                var dronePosition = transform.position;
                var pointDirection = my_path[i].getPosition();
                highdronePosition.y += 1.1f;
                lowdronePosition.y -= 0.9f;
                //pointDirection.y += 1;

                Physics.SphereCast(dronePosition, droneCollider.radius, (pointDirection - dronePosition).normalized  , out rayHit, Vector3.Distance(pointDirection,dronePosition));
                //bool hit2=Physics.CapsuleCast(highdronePosition, lowdronePosition, droneCollider.radius*2, pointDirection - highdronePosition, Vector3.Distance(highdronePosition, pointDirection));
                Debug.DrawRay(transform.position, (my_path[i].getPosition() - transform.position), Color.cyan, 10f);
                //if (rayHit.collider.gameObject.name == "Cube") { 
               
                if (rayHit.collider != null){
                //if (Physics.Raycast(my_path[i].getPosition(),  my_path[i].getPosition()- transform.position, Vector3.Distance(my_path[i].getPosition(), transform.position))) { 
                    break;

                }
            }
            i--;

            target = my_path[i].getPosition();
            targetId = i;
            */
        }
        else
        {
            target = my_path[lastPointInPath].getPosition();
            targetId = lastPointInPath;
        }
           
        
        Debug.Log("Going to node n." + targetId.ToString());
        float breakingDistance = (controller.velocity.magnitude * controller.velocity.magnitude) / (controller.max_acceleration);
        float distanceToTarget = Vector3.Distance(transform.position, target);
        Debug.Log("Distance to target: " + distanceToTarget.ToString());

        Vector3 direction = target - transform.position; //Direction to target!
        bool hit =Physics.SphereCast(transform.position, droneCollider.radius, controller.velocity, out rayHit, breakingDistance );
        if (rayHit.collider!=null) //Slow down if we are going to hit something
        {
            Debug.Log("Breaking distance: " + breakingDistance.ToString());
            Debug.Log("OMG OMG SLOW NOW");
            m_Drone.Move(-controller.velocity.x*100, -controller.velocity.z*100);
            Debug.DrawLine(transform.position, transform.position + controller.acceleration, Color.black);
        }
        
        else
        {
            /*if ( breakingDistance >= distanceToTarget)
            {
                m_Drone.Move(-controller.velocity.x * 100, -controller.velocity.z * 100);
            }*/
            
            if(targetId==my_path.Count-1 && breakingDistance >= distanceToTarget) {
                m_Drone.Move(-controller.velocity.x * 100, -controller.velocity.z * 100);
            }
            else if (controller.velocity.magnitude > 3 && targetId < my_path.Count - 2 && ((my_path[targetId].getTheta() < 130) || (my_path[targetId+1].getTheta() < 130) || (my_path[targetId+2].getTheta() < 130)) && breakingDistance >= distanceToTarget*3)
            {
                m_Drone.Move(-controller.velocity.x * 100, -controller.velocity.z * 100);
            } 
            else if (controller.velocity.magnitude > 3 && targetId < my_path.Count - 2  && ((my_path[targetId].getTheta() < 110) || (my_path[targetId + 1].getTheta() < 110) || (my_path[targetId + 2].getTheta() < 110)) && breakingDistance >= distanceToTarget * 2)
            {
                m_Drone.Move(-controller.velocity.x * 100, -controller.velocity.z * 100);
            }
            else if(controller.velocity.magnitude > 3 && targetId < my_path.Count - 2 && ((my_path[targetId].getTheta() < 90) || (my_path[targetId + 1].getTheta() < 90) ||( my_path[targetId + 2].getTheta() < 90)) && breakingDistance >= distanceToTarget*0.7)
            {
                m_Drone.Move(-controller.velocity.x * 100, -controller.velocity.z * 100);
            }
            else
            {
                Debug.DrawLine(transform.position, transform.position + direction, Color.yellow); //drawing the direction, and it is alwaise correct.
                                                                                                  //Add speed compensation
                Debug.Log("Angle between speed and acc " + Vector3.Angle(direction, controller.velocity).ToString());
                if (controller.velocity.magnitude > 0.5 && Vector3.Angle(direction, controller.velocity) > 1 && Vector3.Angle(direction, controller.velocity) < 50)
                {
                    Vector3 specularSpeed = -controller.velocity + 2 * Vector3.Dot(controller.velocity, direction.normalized) * direction.normalized;
                    Debug.DrawLine(transform.position, transform.position + specularSpeed, Color.white);
                    direction = direction.normalized + 3 * specularSpeed.normalized;
                }
                else if (controller.velocity.magnitude> 0.5 && Vector3.Angle(direction, controller.velocity) >= 50)
                {
                    Debug.Log("Wrong direction, fixing speed of magnitudo " + controller.velocity.magnitude);
                    direction = -controller.velocity * 100;
                    //m_Drone.Move(-controller.velocity.x * 100, -controller.velocity.z * 100);
                }
                //  m_Drone.Move  HERE YOU SHOULD DO ACCELLERATION IN THE DIRECTION OF THE POINT 
                // IF SPEED.DIRECTION != ACCELLERATION. DIRECTION
                // COMPENSATE THE SPEED COMPLEMENTING IT WITH SOME ACCELLERATION IN SPECULAR (NON OPPOSITE) DIRECTION.
                Vector3 high = new Vector3(0, 1, 0);
                Vector3 side1 = Vector3.Cross(direction, high).normalized;
                Vector3 side2 = -side1;
                bool hitside1=Physics.Raycast(transform.position, side1, 4);
                bool hitside2=Physics.Raycast(transform.position, side2, 4);
                if(hitside1 && !hitside2)
                {
                    direction = direction + side2;
                }else if(hitside2 && !hitside1)
                {
                    direction = direction +  side1;
                }

                m_Drone.Move(direction.x, direction.z);
            }
        }

        Debug.DrawLine(transform.position, transform.position + controller.velocity, Color.red);
        Debug.DrawLine(transform.position, transform.position + controller.acceleration, Color.black);
    }
    private void FixedUpdate2(){   
        float radiusMargin = droneCollider.radius + 0.5f;



        DroneController controller=transform.GetComponent<DroneController>();

        // Execute your path here
        // ...

        // this is how you access information about the terrain
        if(lastPointInPath==my_path.Count()-1){
            Vector3 way=Vector3.Lerp(new Vector3(0,0,0),goal_pos,1)-Vector3.Lerp(new Vector3(0,0,0),transform.position,1);
            m_Drone.Move(way.x, way.z);
        }else{
            for(int i=lastPointInPath;i<my_path.Count();i=i+1){
                if (5.0f>=Vector3.Distance(my_path[i].getPosition(),transform.position)){
                    lastPointInPath=i;
                }
            }
            if(lastPointInPath!=my_path.Count()-1){
                
                Vector3 target=my_path[lastPointInPath+1].getPosition();
                float distanceToTargetTemp = Vector3.Distance(transform.position,target);
                int targetId=0;
                for(int i=lastPointInPath+2;i<my_path.Count();i=i+1){
                    float newDistance=Vector3.Distance(transform.position,my_path[i].getPosition());
                    if(distanceToTargetTemp>newDistance){
                        distanceToTargetTemp=newDistance;
                        target=my_path[i].getPosition();
                        targetId=i;
                    }
                }

                
                // this is how you control the car
                Vector3 thirdWay=new Vector3(0,0,0);
                Vector3 secondWay= new Vector3(0,0,0);
                Vector3 way=Vector3.Lerp(new Vector3(0,0,0),target,1)-Vector3.Lerp(new Vector3(0,0,0),transform.position,1);
                if(targetId+1!=my_path.Count()-1){
                    if(targetId+2!=my_path.Count()-1){
                        thirdWay = Vector3.Lerp(new Vector3(0,0,0),my_path[targetId+2].getPosition(),1)-Vector3.Lerp(new Vector3(0,0,0),transform.position,1);

                    }
                    secondWay = Vector3.Lerp(new Vector3(0,0,0),my_path[targetId+1].getPosition(),1)-Vector3.Lerp(new Vector3(0,0,0),transform.position,1);
                }
                


                float breakingDistance = (controller.velocity.magnitude*controller.velocity.magnitude)/(controller.max_acceleration);
                float distanceToTarget = Vector3.Distance(transform.position,target);
                RaycastHit rayHit;
                bool hit = Physics.SphereCast(transform.position,radiusMargin, controller.velocity,out rayHit, breakingDistance+5.0f);
                
                
                if(hit){
                    way=way-controller.velocity*5.0f*controller.velocity.magnitude;
                }else if(targetId+1!=my_path.Count()-1){
                    
                    /*if(targetId+2!=my_path.Count()-1){
                        float s = 6.0f;
                        way=0.1f*(way*(s*s) + secondWay*s + thirdWay);
                        Debug.DrawLine(transform.position,target, Color.white);
                    }*/

                    float targetAngel = Vector3.Angle(controller.acceleration,Vector3.Lerp(target,my_path[targetId+1].getPosition(),1));
                    
                    //controller.acceleration
                    //Vector3.Lerp(target,my_path[targetId+1].getPosition(),1)
                    if(breakingDistance >= distanceToTarget){
                        print("angel:" + targetAngel.ToString());
                        way=secondWay-controller.velocity*(0.01f*(180-targetAngel));//(controller.acceleration*0.01f*(180-targetAngel));

                    }
                    /*else if(lastPointInPath+1!=my_path.Count()-1){
                        way=way;//*0.01f*distanceToTarget;

                        //float L=Vector3.Distance(transform.position,my_path[lastPointInPath+2])-distanceToTarget;
                        //way=Vector3.Lerp(new Vector3(0,0,0),my_path[lastPointInPath+2],1)-Vector3.Lerp(new Vector3(0,0,0),transform.position,1)+way;

                    }*/


                }
                
                
                m_Drone.Move(way.x, way.z);
                /*Vector3 newVelocity=controller.velocity + way.normalized * Time.fixedDeltaTime;
                if(10.0f>newVelocity.magnitude){
                    m_Drone.Move(way.normalized.x, way.normalized.z);
                }*/

                //Debug.DrawLine(transform.position,target, Color.white);
                Debug.DrawLine(transform.position,transform.position+controller.velocity, Color.red);
                Debug.DrawLine(transform.position,transform.position+controller.acceleration, Color.black);
                //Debug.DrawLine(new Vector3(0,0,0),transform.position, Color.white);

            }
        }
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

    bool IsCollidingOnEdge(Vector3 from, Vector3 to,float radiusMargin)
    {
        from.y = 3;
        to.y = 3;

        RaycastHit rayHit;
        bool hit = Physics.SphereCast(from, radiusMargin ,to - from, out rayHit,Vector3.Distance(from,to));
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

        return new Vector3(cordx, 1, cordz);

    }

    public class Node
    {

        private int id;
        private double speedX, speedZ, AccellerationX, AccellerationZ, theta;
        private Vector3 position;
        private float x, z;
        private int color;
        private int parent;
        private float distanceToWall;
        
        public float getDistanceToWall()
        {
            return distanceToWall;
        }
        public void setDistanceToWall(float _d)
        {
            distanceToWall = _d;
        }
        public int getParent()
        {
            return parent;
        }
        public void setParent(int p)
        {
            parent = p;
        }
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
            position = new Vector3(_x, 1, _z);
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


       
        public double computePathCost(List<int> _path)
        {
            //setPathTheta(_path); //Compute the angles of the nodes of that path

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

    public void setPathTheta(List<Node> _path)
    {
        Node A, B, C;
        A = _path[0];
        A.setTheta(0);

        A = _path[0];
        B = _path[1];
        C = _path[2];

        for (int i = 1; i < _path.Count() - 2; i++)
        {
            A = _path[i - 1];
            B = _path[i];
            C = _path[i + 1];
            B.setTheta(computeAngle2(A, B, C));
        }
        C.setTheta(0);

    }
    public float computeAngle2(Node _A, Node _B, Node _C)
    {
        Vector3 aa = _A.getPosition() - _B.getPosition();
        Vector3 bb = _C.getPosition() - _B.getPosition();
        return Vector3.Angle(aa, bb);
    }

    public void RRG(int max_nodes,Graph G)
    {
        float edgeLength = 4f;
        float nodeMinDistance = 1f;
        float addEdgeMaxLength = 9.0f;
        float radiusMargin = droneCollider.radius + 3.0f;

        int max_iter=10000;
        Node close_node=null;
        Vector3 new_coord= new Vector3(0,0,0);
        for (int i = 0; i<max_nodes && max_iter>0; i++)
        {
            //For all the new nodes;

            bool found = false;
            for (max_iter = 10000; !found && max_iter>0; max_iter--)
            {
                Vector3 goal = Pseudo_random(radiusMargin); //Find random point
                close_node = G.FindClosestNode(goal,G); //Find closest node
                float distance = Vector3.Distance(close_node.getPosition(), goal); //And compute the distance
                if (distance > edgeLength)
                {  //skip if B too close 
                    new_coord = Vector3.Lerp(close_node.getPosition(), goal, edgeLength / distance);
                    //distance = Vector3.Distance(new_coord, G.FindClosestNode(new_coord, G).getPosition());
                    //if (distance > nodeMinDistance)
                    //{  //skip if C too close to another point WE DONT FUCKING NEED THIS CHECK, BECAUSE CLOSE_NODE WILL ALWAISE BE THE CLOSEST TO  B
                        if (!position_collision(radiusMargin, new_coord) && !IsCollidingOnEdge(close_node.getPosition(), new_coord,radiusMargin))
                        {
                            found = true;
                        }
                    //}
                }
            }
            //Debug.Log(max_iter);


            int idx = G.addNode(new Node(new_coord));
            G.addEdge(idx, close_node.getId());
            for ( int j =0; j<G.getSize()-1; j++)
            {
                Node temp = G.getNode(j);
                float checkDistance = Vector3.Distance(temp.getPosition(), G.getNode(idx).getPosition());
                if (checkDistance < addEdgeMaxLength && j != idx && !IsCollidingOnEdge(temp.getPosition(), G.getNode(idx).getPosition(),radiusMargin))
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

    public List<int> getBestPath(Graph G,int idx_goal)
    {
        List<int> path = new List<int>();
        path.Add(idx_goal);
        int idx = idx_goal;
        while( idx != 0)
        {
            idx = G.getNode(idx).getParent();
            path.Add(idx);
        }
        path.Reverse();
        return path;
    }

    public void killFuckers(Graph G)
    {
        int i,j,x;
        List<int> adj = new List<int>();
        for (i = 0;i  < G.getSize(); i++)
        {
            adj = G.getAdjList(i);

            for (j=0;j<adj.Count;j++)
            {
                //Debug.Log(Vector3.Distance(G.getNode(i).getPosition(), G.getNode(adj[j]).getPosition()));

                if (Vector3.Distance(G.getNode(i).getPosition(), G.getNode(adj[j]).getPosition())> 10)
                {
                    Debug.Log("KILLLL!");
                    x = adj[j];
                    G.getAdjList(i).Remove(x);
                    G.getAdjList(x).Remove(i);
                    j--;
                }
            }
        }
    }
    public void ASuperStar(Graph G, int idx_goal)
    {
        float radiusMargin = droneCollider.radius + 1.5f;
        priorityQueue Q = new priorityQueue();


        int best_node;
        float best_cost;


        float total_cost;
        Q.enqueue(0, 0);

        while(Q.getSize()!=0)
        {
            best_node = Q.dequeue();
            best_cost = Q.getCost(best_node);
            //Delete node
            Q.removeNode(best_node);

            if (idx_goal == best_node){
                return;
            }

            foreach (int child in G.getAdjList(best_node)){
                total_cost = computeCost(G, best_node, child, idx_goal, radiusMargin) + best_cost;

                if (Q.isInQueue(child))
                {
                    if (Q.getCost(child) > total_cost)
                    {
                        G.getNode(child).setParent(best_node);
                        Q.updateCost(child,total_cost);
                    }
                }
                else
                {
                    if (G.getNode(child).getColor() == 0)
                    {
                        Q.enqueue(child, total_cost);
                        G.getNode(child).setParent(best_node);
                        G.getNode(child).setColor(1);
                    }
                }


            }
        }


    }

    public class priorityQueue
    {
        List<int> values;
        List<float> priority;

        public priorityQueue()
        {
            values = new List<int>();
            priority = new List<float>();
        }
        public void enqueue(int _value, float _p)
        {
            values.Add(_value);
            priority.Add(_p);
        }
        public int getSize()
        {
            return values.Count();
        }
        public int dequeue()
        {
            int best_idx = priority.IndexOf(priority.Min());
            int best_node = values[best_idx];


            return best_node;
        }
        public void removeNode(int node)
        {
            int node_idx = values.IndexOf(node);
            priority.RemoveAt(node_idx);
            values.RemoveAt(node_idx);
        }
        public float getCost(int node)
        {
            int idx = values.IndexOf(node);
            return priority[idx];
        }
        public void updateCost(int node, float p)
        {
            int idx = values.IndexOf(node);
            priority[idx] = p;
        }
        public bool isInQueue(int node)
        {
            int idx = values.IndexOf(node);
            if (idx == -1) { return false; }
            return true;
        }
    }

    public float computeCost(Graph G, int parent, int child, int goal, float radiusMargin)
    {
        //REAL COST:
        float real_cost;
        float h_cost;
        float actual_angle = 0;
        float best_angle=0;
        float max_speed = 15;
        float alpha = 1 / 10;
        
        foreach (var n in G.getAdjList(child))
        {
            actual_angle = computeAngle2(G.getNode(parent), G.getNode(child), G.getNode(n));
            if (actual_angle > best_angle)
            {
                best_angle = actual_angle;
            }
        }
        max_speed = 1 / (1 +  ((180 - best_angle)*alpha) * ((180 - best_angle) * alpha));
        //max_speed = 15;
        real_cost = Vector3.Distance(G.getNode(parent).getPosition(), G.getNode(child).getPosition());//max_speed; /// max_speed;


        RaycastHit rayHit;
        bool hit = Physics.SphereCast(G.getNode(child).getPosition(), radiusMargin, G.getNode(goal).getPosition()- G.getNode(child).getPosition(), out rayHit, Vector3.Distance(G.getNode(child).getPosition(), G.getNode(goal).getPosition()));
        if (!hit)
        {
            h_cost = 0;
        }
        else
        {
            h_cost = Vector3.Distance(G.getNode(child).getPosition(), G.getNode(goal).getPosition());
        }


        return 3 * real_cost + h_cost; //+ (200f / G.getNode(child).getDistanceToWall());

    }

    public void computeDiStanceToWall(Graph G)
    {
        RaycastHit hit;
        float radiusMargin = 1f;

        List<Vector3> radiusHelpMatrix = new List<Vector3>();

        radiusHelpMatrix.Add(new Vector3(-1f, -1f, -1f));
        radiusHelpMatrix.Add(new Vector3(1f, 1f, 1f));
        radiusHelpMatrix.Add(new Vector3(1f, 1f, -1f));
        radiusHelpMatrix.Add(new Vector3(1f, -1f, 1f));
        radiusHelpMatrix.Add(new Vector3(1f, -1f, -1f));
        radiusHelpMatrix.Add(new Vector3(-1f, 1f, 1f));
        radiusHelpMatrix.Add(new Vector3(-1f, 1f, -1f));
        radiusHelpMatrix.Add(new Vector3(-1f, -1f, 1f));

        for (int i =0;i < G.getSize(); i++)
        {
            float minDistance = 5000f;
            float actualDistance; 
            for (int j = 0; j < radiusHelpMatrix.Count; j++)
            {
                Physics.SphereCast(G.getNode(i).getPosition(),2, radiusHelpMatrix[j], out hit, 50f);
                actualDistance=hit.distance;
                if (actualDistance != 0)
                {
                    if (minDistance > actualDistance)
                    {
                        minDistance = actualDistance;
                    }
                }
            }
            G.getNode(i).setDistanceToWall(minDistance);
            Debug.Log(minDistance);
        }
    }
}
