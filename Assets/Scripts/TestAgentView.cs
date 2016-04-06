using UnityEngine;
using System.Collections;
using Pathfinding;

public class TestAgentView : MonoBehaviour
{
    public IAgent Agent;

    void Awake()
    {
        Agent = new Agent(GetComponent<BoxCollider>().bounds);
    }
}
