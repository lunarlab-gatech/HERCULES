#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "ShaderCore.h"

class FHerculesRayTracingModule : public IModuleInterface
{
public:
    virtual void StartupModule() override
    {
        // Global shaders must be registered before the engine compiles them, hence the PostConfigInit loading phase
        const FString ShaderDirectory = FPaths::Combine(IPluginManager::Get().FindPlugin(TEXT("HerculesRayTracing"))->GetBaseDir(), TEXT("Shaders"));
        AddShaderSourceDirectoryMapping(TEXT("/Plugin/HerculesRayTracing"), ShaderDirectory);
    }
};

IMPLEMENT_MODULE(FHerculesRayTracingModule, HerculesRayTracing)
