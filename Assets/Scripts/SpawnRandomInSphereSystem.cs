using Unity.Entities;
using Unity.Transforms;

namespace DefaultNamespace
{
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateAfter(typeof(TransformSystemGroup))]
    public class SpawnRandomInSphereSystem : ComponentSystem
    {
        private ComponentGroup _mainGroup;

        protected override void OnCreateManager()
        {
            base.OnCreateManager();
            _mainGroup = GetComponentGroup(ComponentType.ReadOnly<SpawnRandomInSphere>(),
                ComponentType.ReadOnly<LocalToWorld>());
        }

        protected override void OnUpdate()
        {
            
        }
    }
}