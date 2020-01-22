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
        private float speedX, speedZ, AccellerationX, AccellerationZ, theta;
        private float x, z;
        private List<int> id_children;
        private int id_parent;

        public void addChildren(int _children)
        {
            id_children.Add(_children);
        }
        public ref List<int> getChildren()
        {
            return ref id_children;
        }
        public int getParent()
        {
            return id_parent;
        }
        public void setChildren(List<int> _children)
        {
            id_children = _children;
        }
        public void setParent(int _parent)
        {
            id_parent = _parent;
        }
        public float getPositionX()
        {
            return x;
        }
        public float getPositionZ()
        {
            return z;
        }
        public int getId()
        {
            return id;
        }
        public Node(float _x, float _z)
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
        public void setPositionX(float _x)
        {
            x = _x;
        }
        public void setPositionZ(float _z)
        {
            z = _z;
        }
    }

    public class Tree
    {
        Dictionary<int, Node> nodes = new Dictionary<int, Node>();
        int size;

        int addNode(Node newNode, Node Parent) //Also link it to a parent
    {
            int id = size++;
            newNode.setParent(Parent.getId());

            Parent.addChildren(id);
            nodes.Add(id, newNode);
            return id;
        }
        int addNode(Node newNode)
        {
            int id = size++;
            nodes.Add(id, newNode);
            return id;
        }
        Node getNode(int id)
        {
            return nodes[id];
        }
    }
    }
