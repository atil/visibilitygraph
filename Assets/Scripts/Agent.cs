using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Pathfinding
{
    public delegate void OnPolygonMove(IAgent agent, Vector3 moveVec);

    public interface IAgent
    {
        event OnPolygonMove OnPolygonMove;
        Polygon Polygon { get; }
    }

    public class Agent : IAgent
    {
        public event OnPolygonMove OnPolygonMove;

        public Polygon Polygon { get; private set; }

        public Agent(Vector3[] vertices)
        {
            Polygon = new Polygon(vertices);
        }

        public Agent(Bounds b) : this(GetFloor(b))
        {
        }

        private static Vector3[] GetFloor(Bounds b)
        {
            var bMin = b.min;
            var bMax = b.max;
            var b1 = new Vector3(bMin.x, 0, bMin.z);
            var b2 = new Vector3(bMax.x, 0, bMin.z);
            var b3 = new Vector3(bMax.x, 0, bMax.z);
            var b4 = new Vector3(bMin.x, 0, bMax.z);

            return new[] { b1, b2, b3, b4 };
        }
    }
}
