using System.Collections.Generic;
using UnityEngine;

namespace Pathfinding
{
    public interface INavigator
    {
        void Init(IAgent[] agents);
        Vector3[] GetPath(Vector3 src, Vector3 dest);
        void RegisterAgent(IAgent agent);
        void UnregisterAgent(IAgent agent);
        void Draw();
    }

    public class Navigator : INavigator
    {
        private readonly List<IAgent> _agents = new List<IAgent>();
        private readonly VisibilityGraph _visibilityGraph = new VisibilityGraph();

        public void Init(IAgent[] agents)
        {
            foreach (var agent in agents)
            {
                RegisterAgent(agent);
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

        public void Draw()
        {
            _visibilityGraph.Draw();
        }
    }
}
