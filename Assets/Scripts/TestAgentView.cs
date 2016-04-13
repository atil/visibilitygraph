using UnityEngine;
using Navigation;

public class TestAgentView : MonoBehaviour
{
    public IAgent Agent;

    private Transform _thisTransform;

    void Awake()
    {
        _thisTransform = transform;

        if (GetComponent<BoxCollider>() != null)
        {
            Agent = new Agent(GetComponent<BoxCollider>().bounds, _thisTransform.position);
        }
    }


    public void MyUpdate()
    {
        Agent.Update(_thisTransform.position);
    }
}
