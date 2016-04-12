using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using Navigation.SelfBalancedTree;
using UnityEngine;

namespace Navigation
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
            var list = _adjList[vertex];

            for (var i = 0; i < list.Count; i++)
            {
                _adjList[list[i]].Remove(vertex);
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
            var vX = v.Position.x;
            var vZ = v.Position.z;

            result.Clear();
            var sortedEvents = Util.SortClockwise(vX, vZ, allVertices);

            // Init _bst with all edges that are directly to the right
            for (var i = 0; i < _polygons.Count; i++)
            {
                // If the vertex is on the right of the polygon,
                // then it's certain that the polygon won't be in _bst initially
                var polygon = _polygons[i];
                if (polygon.RightmostX < vX)
                {
                    continue;
                }

                for (var j = 0; j < polygon.Edges.Length; j++)
                {
                    var edge = polygon.Edges[j];
                    float t;
                    if (edge.IntersectsWith(vX, vZ, 1f, 0f, out t))
                    {
                        edge.DistanceToReference = t;
                        _bst.Add(edge);
                    }
                }
            }

            var sortedEventsLen = sortedEvents.Length;
            for (var i = 0; i < sortedEventsLen; i++)
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
                var ccwSide = eventPoint.GetEdgesOnCCwSide(vX, vZ);
                for (int j = 0; j < ccwSide.Length; j++)
                {
                    if (ccwSide[j] != null)
                    {
                        _bst.Delete(ccwSide[j]);
                    }
                }
                var cwSide = eventPoint.GetEdgesOnCwSide(vX, vZ);
                for (int j = 0; j < cwSide.Length; j++)
                {
                    if (cwSide[j] != null)
                    {
                        cwSide[j].DistanceToReference = cwSide[j].DistanceTo(vX, vZ); // TODO: This line smells
                        _bst.Add(cwSide[j]);
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
                && from.OwnerPolygon.HasVertex(to))
            {
                return false;
            }

            var fromX = from.Position.x;
            var fromZ = from.Position.z;
            var toX = to.Position.x;
            var toZ = to.Position.z;

            // Check if "to" is behind "from"
            // If not behind, it's certain that it won't intersect with polygon
            // Note that this optimization works only with convex polygons
            var dot = from.NormalX * (fromX - toX) +
                      from.NormalZ * (fromZ - toZ);
            if (dot < 0)
            {
                // Check with if intersecting with owner polygon
                // Nudge a little bit away from polygon, so it won't intersect with neighboring edges
                var nudgedX = fromX - Mathf.Sign(fromX - toX) * 0.0001f;
                var nudgedZ = fromZ - Mathf.Sign(fromZ - toZ) * 0.0001f;
                if (from.OwnerPolygon != null
                    && from.OwnerPolygon.IntersectsWith(nudgedX, nudgedZ, toX, toZ))
                {
                    return false;
                }
            }
            Edge leftMostEdge;
            _bst.GetMin(out leftMostEdge);

            if (leftMostEdge != null
                && leftMostEdge.IntersectsWith(fromX, fromZ, toX, toZ))
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
            _adjList.Add(srcVertex, new List<Vertex>());
            _adjList.Add(destVertex, new List<Vertex>());

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
                    // TODO: Maybe sqrDistance?
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
