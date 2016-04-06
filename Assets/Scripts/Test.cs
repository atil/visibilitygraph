using UnityEngine;
using System.Linq;
using Pathfinding;

public class Test : MonoBehaviour
{
    private INavigator _navigator;

    void Start()
    {
        _navigator = new Navigator();
        var agents = FindObjectsOfType<TestAgentView>().Select(x => x.Agent).ToArray();

        _navigator.Init(agents);
    }

    void Update()
    {
        _navigator.Draw();
    }

}
