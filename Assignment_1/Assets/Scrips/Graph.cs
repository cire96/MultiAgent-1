using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.Scrips
{
    public class Node
    {

        private int id;
        private double speedX, speedZ, AccellerationX, AccellerationZ, theta;
        private double x, z;



 
        public double getPositionX()
        {
            return x;
        }
        public double getPositionZ()
        {
            return z;
        }
        public int getId()
        {
            return id;
        }
        public Node(double _x, double _z)
        {
            x = _x;
            z = _z;
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
        public void setPositionX(double _x)
        {
            x = _x;
        }
        public void setPositionZ(double _z)
        {
            z = _z;
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

        int size;
        
        int addNode(Node _newNode)
        {
            int id = size++;
            nodes.Add(id, _newNode);
            adjList.Add(id, new List<int>());
            return id;
        }
        int addNode(Node _newNode, List<int> _adjList)
        {
            int id = size++;
            nodes.Add(id, _newNode);
            adjList.Add(id, _adjList);
            return id;
        }
         Node getNode(int _id)
        {
            return  nodes[_id];
        }
        List<int> getAdjList(int _id)
        {
            return adjList[_id];
        }
        void setAdjList(int _id, List<int> _adjList)
        {
            adjList.Add(_id, _adjList);
        }
        void addEdge(int _idA,int _idB )
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
            double angle = Math.Acos(num / (den + 0.000001));
            return angle;
        }
        public void setPathTheta(List<int> _path)
        {
            int A, B, C=0;
            A = _path[0];
            nodes[A].setTheta(0);

            for(int i=1; i<_path.Count()-1; i++){
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
    }

    
    }
