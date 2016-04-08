using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using Pathfinding.SelfBalancedTree;
using UnityEngine;

namespace Pathfinding
{
    public class VisibilityGraph
    {
        private readonly AVLTree<Edge> _bst = new AVLTree<Edge>();
        private readonly List<Polygon> _polygons = new List<Polygon>();
        private readonly List<Vertex> _allVertices = new List<Vertex>();
        private readonly Dictionary<Vertex, List<Vertex>> _adjList = new Dictionary<Vertex, List<Vertex>>();

        // Cached for pathfinding
        private readonly Dictionary<Vertex, float> _distances = new Dictionary<Vertex, float>();
        private readonly Dictionary<Vertex, Vertex> _prevs = new Dictionary<Vertex, Vertex>();

        public void AddPolygon(Polygon polygon)
        {
            if (_polygons.Contains(polygon))
            {
                return;
            }

            _polygons.Add(polygon);
            foreach (var v in polygon.Vertices)
            {
                _allVertices.Add(v);

                _adjList.Add(v, new List<Vertex>());
            }

            // Place polygon
            foreach (var vertex in polygon.Vertices)
            {
                CalculateVisiblityForVertex(vertex, _allVertices);
            }

            // Recalculate it's touching edges -- it might have obstructed someone's vision
            var touchingVertices = GetTouchingVertices(polygon);

            foreach (var touchingVertex in touchingVertices)
            {
                RemoveEdgesOfVertex(touchingVertex);
            }

            foreach (var touchingVertex in touchingVertices)
            {
                CalculateVisiblityForVertex(touchingVertex, _allVertices);
            }
        }

        public void RemovePolygon(Polygon polygon)
        {
            // Remove the polygon's edges and recalculate the polygon's neighbors
            var touchingVertices = GetTouchingVertices(polygon);

            foreach (var vertex in polygon.Vertices)
            {
                RemoveEdgesOfVertex(vertex);
            }

            _polygons.Remove(polygon);
            foreach (var v in polygon.Vertices)
            {
                _allVertices.Remove(v);
                _adjList.Remove(v);
            }

            foreach (var touchingVertex in touchingVertices)
            {
                CalculateVisiblityForVertex(touchingVertex, _allVertices);
            }

        }

        public void MovePolygon(Polygon polygon, Vector3 moveVec)
        {
            // Store the polygon's existing neighbors
            var touchingVertices = GetTouchingVertices(polygon);

            // Remove the polygon's edges
            for (var i = 0; i < polygon.Vertices.Count; i++)
            {
                RemoveEdgesOfVertex(polygon.Vertices[i]);
            }

            // Move polygon and calculate visibility for it
            polygon.Move(moveVec);
            for (var i = 0; i < polygon.Vertices.Count; i++)
            {
                CalculateVisiblityForVertex(polygon.Vertices[i], _allVertices);
            }

            // Append the polygon's new neighbors
            foreach (var touchingVertex in GetTouchingVertices(polygon))
            {
                touchingVertices.Add(touchingVertex);
            }

            // Recalculate touching vertices
            foreach (var touchingVertex in touchingVertices)
            {
                RemoveEdgesOfVertex(touchingVertex);
            }

            foreach (var touchingVertex in touchingVertices)
            {
                CalculateVisiblityForVertex(touchingVertex, _allVertices);
            }
        }

        private void RemoveEdgesOfVertex(Vertex vertex)
        {
            foreach (var vertexNeighbor in _adjList[vertex])
            {
                _adjList[vertexNeighbor].Remove(vertex);
            }
           
        }

        private HashSet<Vertex> GetTouchingVertices(Polygon polygon)
        {
            var retVal = new HashSet<Vertex>();
            foreach (var vertex in polygon.Vertices)
            {
                foreach (var vertexNeighbor in _adjList[vertex])
                {
                    if (!polygon.Vertices.Contains(vertexNeighbor))
                    {
                        retVal.Add(vertexNeighbor);
                    }
                }
            }

            return retVal;
            
        }

        private void CalculateVisiblityForVertex(Vertex pivot, List<Vertex> allVertices)
        {
            var result = _adjList[pivot];
            GetVisibilePoints(pivot, allVertices, ref result);
        }

        private void GetVisibilePoints(Vertex v, List<Vertex> allVertices, ref List<Vertex> result)
        {
            result.Clear();
            var sortedEvents = Util.SortClockwise(v.Position, allVertices);

            var ray = new Ray(v.Position, Vector3.right);

            
            for (var i = 0; i < _polygons.Count; i++)
            {
                for (var j = 0; j < _polygons[i].Edges.Length; j++)
                {
                    var edge = _polygons[i].Edges[j];
                    float t;
                    if (edge.IntersectsWith(ray, out t))
                    {
                        edge.DistanceToReference = t;
                        _bst.Add(edge);
                    }
                }
            }

            
            for (var i = 0; i < sortedEvents.Length; i++)
            {
                var eventPoint = sortedEvents[i];
                if (eventPoint == v)
                {
                    continue;
                }

                if (IsVisible(v, eventPoint))
                {
                    result.Add(eventPoint);
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
        }

        private bool IsVisible(Vertex from, Vertex to)
        {
            // Neighboring vertices are assumed to be seeing each other
            if (from.IsNeighborWith(to))
            {
                return true;
            }

            // Non-neighbor vertices of the same polygon don't see each other
            if (from.OwnerPolygon != null // There will be stray vertices during pathfinding
                && from.OwnerPolygon.Vertices.Contains(to))
            {
                return false;
            }

            // Check if "to" is behind "from"
            // If not behind, it's certain that it won't intersect with polygon
            // Note that this optimization works only with convex polygons
            var dot = from.NormalX * (from.Position.x - to.Position.x) +
                      from.NormalZ * (from.Position.z - to.Position.z);
            if (dot < 0)
            {
                // Check with if intersecting with owner polygon
                // Nudge a little bit away from polygon, so it won't intersect with neighboring edges
                var nudgedX = from.Position.x - Mathf.Sign(from.Position.x - to.Position.x) * 0.0001f;
                var nudgedZ = from.Position.z - Mathf.Sign(from.Position.z - to.Position.z) * 0.0001f;
                if (from.OwnerPolygon != null
                    && from.OwnerPolygon.IntersectsWith(nudgedX, nudgedZ, to.Position.x, to.Position.z))
                {
                    return false;
                }
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

        #region Shortest path

        public Vector3[] GetPath(Vector3 srcPos, Vector3 destPos)
        {
            if (_allVertices.Count < 1 // No vertices in graph
                || !IntersectsWith(srcPos.x, srcPos.z, destPos.x, destPos.z)) // Doesn't intersect with any polygon
            {
                return new[] { srcPos, destPos };
            }

            // Source and destination points are temporarily in the graph
            var srcVertex = new Vertex(srcPos);
            var destVertex = new Vertex(destPos);

            CalculateVisiblityForVertex(srcVertex, _allVertices);
            CalculateVisiblityForVertex(destVertex, _allVertices);

            // Manually set destination's neighbors
            foreach (var destNeighbor in _adjList[destVertex])
            {
                _adjList[destNeighbor].Add(destVertex);
            }

            var allVertsCopy = new HashSet<Vertex>(_allVertices);
            allVertsCopy.Add(srcVertex);
            allVertsCopy.Add(destVertex);

            // Here be Djikstra
            _distances.Clear();
            _prevs.Clear();

            foreach (var vertex in allVertsCopy)
            {
                _distances.Add(vertex, float.MaxValue);
                _prevs.Add(vertex, null);
            }

            _distances[srcVertex] = 0f;

            while (allVertsCopy.Count > 0)
            {
                Vertex u = null;
                var minDist = float.MaxValue;
                foreach (var vertex in allVertsCopy)
                {
                    if (_distances[vertex] < minDist)
                    {
                        minDist = _distances[vertex];
                        u = vertex;
                    }
                }

                if (u == destVertex) // Destination found
                {
                    break;
                }

                allVertsCopy.Remove(u);

                Debug.Assert(u != null, "Next vertex is null");

                foreach (var v in _adjList[u])
                {
                    if (!allVertsCopy.Contains(v))
                    {
                        continue;
                    }

                    // TODO: PathEdge's Weight should be used instead of Distance() call
                    // No need to calculate it again
                    var altDist = _distances[u] + Vector3.Distance(v.Position, u.Position); 
                    if (altDist < _distances[v])
                    {
                        _distances[v] = altDist;
                        _prevs[v] = u;
                    }
                }
            }

            var path = new List<Vector3>();
            var w = destVertex;

            while (_prevs[w] != null)
            {
                path.Insert(0, w.Position);
                w = _prevs[w];
            }
            path.Insert(0, w.Position);

            // Remove temp vertices from graph
            RemoveEdgesOfVertex(srcVertex);
            RemoveEdgesOfVertex(destVertex);
            
            _adjList.Remove(srcVertex);
            _adjList.Remove(destVertex);

            return path.ToArray();
        }

        private bool IntersectsWith(float v1X, float v1Z, float v2X, float v2Z)
        {
            foreach (var polygon in _polygons)
            {
                if (polygon.IntersectsWith(v1X, v1Z, v2X, v2Z))
                {
                    return true;
                }
            }

            return false;
        }

        #endregion

        public void Draw()
        {
            foreach (var vertListPair in _adjList)
            {
                foreach (var vertex in vertListPair.Value)
                {
                    Debug.DrawLine(vertex.Position, vertListPair.Key.Position, Color.red);
                }

            }

        }
    }
}
