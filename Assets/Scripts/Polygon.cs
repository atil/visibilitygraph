using UnityEngine;
using System.Collections.Generic;
using System.Linq;

namespace Navigation
{
    public class Polygon
    {
        public readonly List<Vertex> Vertices;
        public readonly Edge[] Edges;
        public float RightmostX { get; private set; }
        public float TopmostZ { get; private set; }
        public float BottommostZ { get; private set; }

        public Polygon(Vector3[] vertices)
        {
            RightmostX = float.MinValue;
            TopmostZ = float.MinValue;
            BottommostZ = float.MaxValue;
            Vertices = new List<Vertex>();
            Edges = new Edge[vertices.Length];

            foreach (var v in vertices)
            {
                if (v.x > RightmostX)
                {
                    RightmostX = v.x;
                }

                if (v.z > TopmostZ)
                {
                    TopmostZ = v.z;
                }
                if (v.z < BottommostZ)
                {
                    BottommostZ = v.z;
                }

                Vertices.Add(new Vertex(v, this));
            }

            for (var i = 1; i < Vertices.Count; i++)
            {
                var e = new Edge(Vertices[i - 1], Vertices[i]);
                Vertices[i - 1].Edge1 = e;
                Vertices[i].Edge2 = e;
                Edges[i - 1] = e;
            }

            var eLast = new Edge(Vertices.Last(), Vertices[0]);
            Vertices.Last().Edge1 = eLast;
            Vertices[0].Edge2 = eLast;

            // Edges are in place, calculate normals
            for (var i = 0; i < Vertices.Count; i++)
            {
                Vertices[i].CalculateNormal();
            }

            Edges[vertices.Length - 1] = eLast;
        }

        public void WarpTo(Vector3[] newPositions)
        {
            // Must be warping to an identical shape
            if (newPositions.Length != Vertices.Count)
            {
                return;
            }

            for (var i = 0; i < newPositions.Length; i++)
            {
                Vertices[i].WarpTo(newPositions[i]);
            }

            foreach (var edge in Edges)
            {
                edge.RecacheVertexPositions();
            }
        }

        public void Move(Vector3 moveVec)
        {
            for (var i = 0; i < Vertices.Count; i++)
            {
                var v = Vertices[i];
                if (v.X > RightmostX)
                {
                    RightmostX = v.X;
                }

                if (v.Z > TopmostZ)
                {
                    TopmostZ = v.Z;
                }

                if (v.Z < BottommostZ)
                {
                    BottommostZ = v.Z;
                }

                v.Move(moveVec);
            }

            for (var i = 0; i < Edges.Length; i++)
            {
                Edges[i].RecacheVertexPositions();
            }
            
        }

        public bool IntersectsWith(float v1X, float v1Z, float v2X, float v2Z)
        {
            for (int i = 0; i < Edges.Length; i++)
            {
                if (Edges[i].IntersectsWith(v1X, v1Z, v2X, v2Z))
                {
                    return true;
                }
            }

            return false;
        }
    }
}
