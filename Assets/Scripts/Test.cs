using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Pathfinding;
using Pathfinding.SelfBalancedTree;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class PathEdge : IEquatable<PathEdge>
{
    public Vertex Vertex1 { get; private set; }
    public Vertex Vertex2 { get; private set; }

    public PathEdge(Vertex v1, Vertex v2)
    {
        Vertex1 = v1;
        Vertex2 = v2;
    }

    public bool Equals(PathEdge other)
    {
        return (Vertex1 == other.Vertex1 && Vertex2 == other.Vertex2)
               || (Vertex1 == other.Vertex2 && Vertex2 == other.Vertex1);
    }
}

public class Test : MonoBehaviour
{
    private readonly List<Polygon> _polygons = new List<Polygon>();
    private readonly AVLTree<Edge> _bst = new AVLTree<Edge>();
    private readonly Dictionary<int, PathEdge> _pathEdgeDict = new Dictionary<int, PathEdge>(); 
    private List<GameObject> _obstacleGos = new List<GameObject>();

    private List<Vertex> _allVertices;


    void Start()
    {
        _obstacleGos = GameObject.FindGameObjectsWithTag("Obstacle").ToList();
        foreach (var obstacleGo in _obstacleGos)
        {
            var floor = GetFloor(obstacleGo.GetComponent<Collider>().bounds);
            _polygons.Add(new Polygon(floor));
        }

        _allVertices = new List<Vertex>();

        foreach (var polygon in _polygons)
        {
            foreach (var v in polygon.Vertices)
            {
                _allVertices.Add(v);
            }
        }

        foreach (var v in _allVertices)
        {
            CalculateVisiblity(v, _allVertices);
        }
    }

    private Vector3[] GetFloor(Bounds b)
    {
        var bMin = b.min;
        var bMax = b.max;
        var b1 = new Vector3(bMin.x, 0, bMin.z);
        var b2 = new Vector3(bMax.x, 0, bMin.z);
        var b3 = new Vector3(bMax.x, 0, bMax.z);
        var b4 = new Vector3(bMin.x, 0, bMax.z);

        return new[] { b1, b2, b3, b4 };
    }

    void Update()
    {
        //_polygons[0].WarpTo(GetFloor(_obstacleGos[0].GetComponent<Collider>().bounds));

        //CalculateVisiblityGraph(_allVertices);

        if (Input.GetKeyDown(KeyCode.Space))
        {
            MovePolygon(_polygons[0], Vector3.forward);
            //foreach (var v in _allVertices)
            //{
            //    CalculateVisiblity(v, _allVertices);
            //}
            var a = 3;
        }

        foreach (var pathEdge in _pathEdgeDict.Values)
        {
            Debug.DrawLine(pathEdge.Vertex1.Position, pathEdge.Vertex2.Position, Color.red);
        }
    }

    private void MovePolygon(Polygon polygon, Vector3 moveVec)
    {
        // This holds vertices from the polygon's broken visibilities and newly acquired visibilities
        // Not from polygon itself
        var verticesToRecalc = new HashSet<Vertex>();

        // Polygon's existing visibility edges are to be removed
        var polygonVisibilities = new HashSet<int>();
        foreach (var pathEdgeEntry in _pathEdgeDict)
        {
            if (polygon.Vertices.Contains(pathEdgeEntry.Value.Vertex1) || polygon.Vertices.Contains(pathEdgeEntry.Value.Vertex2))
            {
                // We don't want the polygon's vertex
                verticesToRecalc.Add(polygon.Vertices.Contains(pathEdgeEntry.Value.Vertex1) ? pathEdgeEntry.Value.Vertex2 : pathEdgeEntry.Value.Vertex1);
                polygonVisibilities.Add(pathEdgeEntry.Key);
            }
        }

        foreach (var key in polygonVisibilities)
        {
            _pathEdgeDict.Remove(key);
        }

        // Move and recalculate polygon's visiblity
        polygon.Move(moveVec);
        foreach (var vertex in polygon.Vertices)
        {
            CalculateVisiblity(vertex, _allVertices);
        }

        // Polygon's newly acquired visibilities are to be recalculated from scratch,
        // since the polygon might have obstructed their sight
        var newVertexVisibilities = new HashSet<int>();
        foreach (var newEdgeEntry in _pathEdgeDict)
        {
            if (polygon.Vertices.Contains(newEdgeEntry.Value.Vertex1) ||
                polygon.Vertices.Contains(newEdgeEntry.Value.Vertex2))
            {
                var vertToRacalc = polygon.Vertices.Contains(newEdgeEntry.Value.Vertex1)
                    ? newEdgeEntry.Value.Vertex2
                    : newEdgeEntry.Value.Vertex1;

                verticesToRecalc.Add(vertToRacalc);

                // Fetch visibilities for this vertex
                foreach (var pathEdgeEntry in _pathEdgeDict)
                {
                    if (pathEdgeEntry.Value.Vertex1 == vertToRacalc || pathEdgeEntry.Value.Vertex2 == vertToRacalc)
                    {
                        newVertexVisibilities.Add(pathEdgeEntry.Key);
                    }
                }
            }
        }

        // Break off new vertex's visibilities
        foreach (var key in newVertexVisibilities)
        {
            _pathEdgeDict.Remove(key);
        }

        foreach (var vertex in verticesToRecalc)
        {
            CalculateVisiblity(vertex, _allVertices);
        }

    }

    void CalculateVisiblity(Vertex pivot, List<Vertex> allVertices)
    {
        var visibleVertices = GetVisibilePoints(pivot, allVertices);

        foreach (var visibleVertex in visibleVertices)
        {
            var key = pivot.GetHashCode() ^ visibleVertex.GetHashCode();
            if (!_pathEdgeDict.ContainsKey(key))
            {
                _pathEdgeDict.Add(key, new PathEdge(pivot, visibleVertex));
            }
        }
    }
    
    private List<Vertex> GetVisibilePoints(Vertex v, List<Vertex> allVertices)
    {
        var visiblePoints = new List<Vertex>();
        var sortedEvents = Util.SortClockwise(v.Position, allVertices);

        var ray = new Ray(v.Position, Vector3.right);
        foreach (var polygon in _polygons)
        {
            foreach (var edge in polygon.Edges)
            {
                float t;
                if (edge.IntersectsWith(ray, out t))
                {
                    edge.DistanceToReference = t;
                    _bst.Add(edge);
                }
            }
        }

        foreach (var eventPoint in sortedEvents)
        {
            if (eventPoint == v)
            {
                continue;
            }

            if (IsVisible(v, eventPoint))
            {
                visiblePoints.Add(eventPoint);
            }

            // Algorithm adds CW edges, then deletes CCW ones
            // The reverse is done here,
            // Reason is because when an edge is added with a ref distance value "d" already exists in the tree
            // Removing the older "d" also removes the newly added one
            foreach (var edge in eventPoint.GetEdgesOnCCwSide(v))
            {
                if (edge != null)
                {
                    _bst.Delete(edge);
                }
            }
            foreach (var edge in eventPoint.GetEdgesOnCwSide(v))
            {
                if (edge != null)
                {
                    edge.DistanceToReference = edge.DistanceTo(v.Position); // TODO: This line smells
                    _bst.Add(edge);
                }
            }

        }

        _bst.Clear();
        return visiblePoints;
    }

    private bool IsVisible(Vertex from, Vertex to)
    {
        // Neighboring vertices are assumed to be seeing each other
        if (from.IsNeighborWith(to))
        {
            return true;
        }

        // Non-neighbor vertices of the same polygon don't see each other
        if (from.OwnerPolygon.Vertices.Contains(to))
        {
            return false;
        }

        // Check with if intersecting with owner polygon
        // Nudge a little bit away from polygon, so it won't intersect with neighboring edges
        var nudgedFrom = from.Position + (to.Position - from.Position).normalized * 0.0001f;
        if (from.OwnerPolygon.IntersectsWith(nudgedFrom.x, nudgedFrom.z, to.Position.x, to.Position.z))
        {
            return false;
        }

        Edge leftMostEdge;
        _bst.GetMin(out leftMostEdge);

        if (leftMostEdge != null
            && leftMostEdge.IntersectsWith(from.Position.x, from.Position.z, to.Position.x, to.Position.z))
        {
            return false;
        }

        return true;
    }

}
