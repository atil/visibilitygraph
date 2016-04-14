using System.Collections.Generic;
using UnityEngine;

namespace Navigation
{
    public interface INavigator
    {
        void Init(IAgent[] agents);
        Vector3[] GetPath(Vector3 src, Vector3 dest);
        void RegisterAgent(IAgent agent);
        void UnregisterAgent(IAgent agent);
        void Update();
        void Draw();
    }

    public class Navigator : INavigator
    {
        private readonly List<IAgent> _agents = new List<IAgent>();
        private VisibilityGraph _visibilityGraph;
        private readonly Dictionary<int, List<IAgent>> _viewBuckets = new Dictionary<int, List<IAgent>>();
        private const int BucketSize = 3;
        private int _currentBucket;

        public void Init(IAgent[] agents)
        {
            _visibilityGraph = new VisibilityGraph();

            foreach (var agent in agents)
            {
                RegisterAgent(agent);
            }

            for (var j = 0; j < Mathf.CeilToInt(agents.Length / 3f); j++)
            {
                _viewBuckets.Add(j, new List<IAgent>());
            }

            var i = 0;
            foreach (var agent in agents)
            {
                if (_viewBuckets[i].Count == BucketSize)
                {
                    i++;
                }
                _viewBuckets[i].Add(agent);
            }
        }

        public void RegisterAgent(IAgent agent)
        {
            agent.OnPolygonMove += OnPolygonMove;
            _agents.Add(agent);
            _visibilityGraph.AddPolygon(agent.Polygon);
        }

        public void UnregisterAgent(IAgent agent)
        {
            agent.OnPolygonMove -= OnPolygonMove;
            _agents.Remove(agent);
            _visibilityGraph.RemovePolygon(agent.Polygon);
        }

        private void OnPolygonMove(IAgent agent, Vector3 moveVec)
        {
            _visibilityGraph.MovePolygon(agent.Polygon, moveVec);
        }
        
        public Vector3[] GetPath(Vector3 srcPos, Vector3 destPos)
        {
            return _visibilityGraph.GetPath(srcPos, destPos);
        }

        public void Update()
        {
            // Update a certain bucket each tick
            var agentsToUpdate = _viewBuckets[_currentBucket];
            foreach (var agent in agentsToUpdate)
            {
                agent.Update();
            }

            if (++_currentBucket > _viewBuckets.Count - 1)
            {
                _currentBucket = 0;
            }

        }

        public void Draw()
        {
            _visibilityGraph.Draw();
        }
    }
}
