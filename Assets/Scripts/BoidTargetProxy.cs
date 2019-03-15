using Unity.Entities;
using UnityEngine;

public struct BoidTarget : IComponentData
{
}

[DisallowMultipleComponent]
public class BoidTargetProxy : ComponentDataProxy<BoidTarget>
{
}