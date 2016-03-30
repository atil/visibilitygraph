using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace Pathfinding
{
    public class Edge : IComparable<Edge>
    {
        public float DistanceToReference { get; set; } 

        public Vertex Vertex1 { get; private set; }
        public Vertex Vertex2 { get; private set; }

        public Edge(Vertex vertex1, Vertex vertex2)
        {
            Vertex1 = vertex1;
            Vertex2 = vertex2;
        }

        public Vertex GetOther(Vertex v)
        {
            if (v == Vertex1)
            {
                return Vertex2;
            }

            if (v == Vertex2)
            {
                return Vertex1;
            }
            Debug.LogError("Edge.GetOther failure");
            return null;
        }

        public float DistanceTo(Vector3 p)
        {
            return Util.PointLineSegmentDistance(Vertex1.Position, Vertex2.Position, p);
        }

        public bool IntersectsWith(Vector3 other1, Vector3 other2)
        {
            // Stolen from: http://yunus.hacettepe.edu.tr/~burkay.genc/courses/bca608/slides/week3.pdf
            return (Util.Left(Vertex1.Position, Vertex2.Position, other1) ^ Util.Left(Vertex1.Position, Vertex2.Position, other2)) 
                && (Util.Left(other1, other2, Vertex1.Position) ^ Util.Left(other1, other2, Vertex2.Position));
        }

        public bool IntersectsWith(Edge e)
        {
            return IntersectsWith(e.Vertex1.Position, e.Vertex2.Position);
        }

        public bool IntersectsWith(Ray ray)
        {
            return Util.RayLineIntersection(ray, Vertex1.Position, Vertex2.Position);
        }

        public bool IntersectsWith(Ray ray, out float t)
        {
            return Util.RayLineIntersection(ray, Vertex1.Position, Vertex2.Position, out t);
        }

        public int CompareTo(Edge other)
        {
            if (other == null)
            {
                return 1;
            }
            return DistanceToReference.CompareTo(other.DistanceToReference);

        }

        public override string ToString()
        {
            return Vertex1 + " - " + Vertex2;
        }
    }

    public class Vertex
    {
        public Vector3 Position { get; private set; }
        public Edge Edge1 { get; set; }
        public Edge Edge2 { get; set; }

        public List<Vertex> VisibleVertices { get; private set; } 

        public Vertex(Vector3 pos)
        {
            Position = pos;
            VisibleVertices = new List<Vertex>();
        }

        public bool IsNeighbor(Vertex v)
        {
            return Edge1.GetOther(this) == v || Edge2.GetOther(this) == v;
        }

        public List<Edge> GetEdgesOnSide(bool isClockwise, Vertex reference) // TODO: Could be cached?
        {
            var retVal = new List<Edge>();

            var cross1 = Vector3.Cross(Position - reference.Position, Edge1.GetOther(this).Position - reference.Position);
            var cross2 = Vector3.Cross(Position - reference.Position, Edge2.GetOther(this).Position - reference.Position);

            if (isClockwise)
            {
                if (cross1.y > 0)
                {
                    retVal.Add(Edge1);
                }

                if (cross2.y > 0)
                {
                    retVal.Add(Edge2);
                }

            }
            else
            {
                if (cross1.y < 0)
                {
                    retVal.Add(Edge1);
                }

                if (cross2.y < 0)
                {
                    retVal.Add(Edge2);
                }
            }


            return retVal;
        }

        public override string ToString()
        {
            return Position.ToString();
        }
    }

    public class Polygon
    {
        public readonly List<Vertex> Vertices;
        public readonly List<Edge> Edges; 

        public Polygon(Vector3[] vertices)
        {
            Vertices = new List<Vertex>();
            Edges = new List<Edge>();

            foreach (var v in vertices)
            {
                Vertices.Add(new Vertex(v));
            }

            for (var i = 1; i < Vertices.Count; i++)
            {
                var e = new Edge(Vertices[i - 1], Vertices[i]);
                Vertices[i - 1].Edge1 = e;
                Vertices[i].Edge2 = e;
                Edges.Add(e);
            }

            var eLast = new Edge(Vertices.Last(), Vertices[0]);
            Vertices.Last().Edge1 = eLast;
            Vertices[0].Edge2 = eLast;
            Edges.Add(eLast);
        }

        public bool IntersectsWith(Vector3 v1, Vector3 v2)
        {
            foreach (var edge in Edges)
            {
                if (edge.IntersectsWith(v1, v2))
                {
                    return true;
                }
            }

            return false;
        }
    }
}
