#include "HerculesGpuRayCaster.h"

#include "Async/Async.h"
#include "Containers/Queue.h"
#include "Containers/Ticker.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "Engine/World.h"
#include "FXRenderingUtils.h"
#include "GlobalRenderResources.h"
#include "GlobalShader.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CoreDelegates.h"
#include "RHIGPUReadback.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderUtils.h"
#include "RenderingThread.h"
#include "SceneInterface.h"
#include "SceneRendererInterface.h"
#include "SceneUniformBuffer.h"
#include "SceneView.h"
#include "SceneViewExtension.h"
#include "ShaderCompilerCore.h"
#include "ShaderParameterStruct.h"
#include <atomic>

DEFINE_LOG_CATEGORY_STATIC(LogHerculesRayTracing, Log, All);

static_assert(sizeof(FHerculesGpuRay) == 32, "FHerculesGpuRay must match FHerculesRay in HerculesRayTrace.usf");
static_assert(sizeof(FHerculesGpuHit) == 20, "FHerculesGpuHit must match FHerculesHit in HerculesRayTrace.usf");

#if RHI_RAYTRACING

class FHerculesRayTraceCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FHerculesRayTraceCS);
    SHADER_USE_PARAMETER_STRUCT(FHerculesRayTraceCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_STRUCT_REF(FViewUniformShaderParameters, View)
        SHADER_PARAMETER_RDG_UNIFORM_BUFFER(FSceneUniformParameters, Scene)
        SHADER_PARAMETER_RDG_BUFFER_SRV(RaytracingAccelerationStructure, TLAS)
        SHADER_PARAMETER_SRV(StructuredBuffer, RayTracingSceneMetadata)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FHerculesRay>, Rays)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, ExcludedComponentIds)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<FHerculesHit>, Hits)
        SHADER_PARAMETER(FVector3f, TranslatedOrigin)
        SHADER_PARAMETER(uint32, RayCount)
        SHADER_PARAMETER(uint32, ExcludedCount)
        SHADER_PARAMETER(uint32, MaxRetraces)
    END_SHADER_PARAMETER_STRUCT()

    static constexpr int32 ThreadGroupSize = 64;

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsRayTracingEnabledForProject(Parameters.Platform) && RHISupportsRayTracing(Parameters.Platform)
            && RHISupportsInlineRayTracing(Parameters.Platform);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        OutEnvironment.CompilerFlags.Add(CFLAG_InlineRayTracing);
        OutEnvironment.SetDefine(TEXT("THREADGROUP_SIZE"), ThreadGroupSize);
        OutEnvironment.SetDefine(TEXT("VF_SUPPORTS_PRIMITIVE_SCENE_DATA"), 1);
    }
};
IMPLEMENT_GLOBAL_SHADER(FHerculesRayTraceCS, "/Plugin/HerculesRayTracing/Private/HerculesRayTrace.usf", "HerculesRayTraceCS", SF_Compute);

namespace
{
    // Excluded surfaces a ray may pass through before giving up (reported as a miss)
    constexpr uint32 MaxRetraces = 16;
    // Render frames a batch may wait for a frame with a ray tracing structure
    constexpr uint32 MaxWaitFrames = 60;

    struct FBatch
    {
        const FSceneInterface* Scene = nullptr; // compared only, never dereferenced
        FVector Origin = FVector::ZeroVector;
        TArray<FHerculesGpuRay> Rays;
        TArray<uint32> Excluded;
        FHerculesGpuRayCaster::FOnComplete OnComplete;
        uint64 QueuedFrame = 0;
    };

    struct FInFlight
    {
        TUniquePtr<FRHIGPUBufferReadback> Readback;
        int32 RayCount = 0;
        FHerculesGpuRayCaster::FOnComplete OnComplete;
    };

    struct FResult
    {
        bool bCast = false;
        TArray<FHerculesGpuHit> Hits;
        FHerculesGpuRayCaster::FOnComplete OnComplete;
    };

    // Render thread
    TArray<FBatch> PendingBatches;
    TArray<FInFlight> InFlight;
    // Render thread to game thread
    TQueue<FResult, EQueueMode::Spsc> Results;
    // Batches submitted but not yet dispatched; lets the extension stay inactive otherwise
    std::atomic<int32> PendingCount{0};
    // After a batch found no ray tracing structure (nothing in the scene is ray traced), casting is
    // reported unsupported for a while, so callers use the CPU without waiting for each batch
    constexpr double UnavailableSeconds = 30.0;
    std::atomic<double> UnavailableUntilSeconds{0.0};
    std::atomic<bool> WarnedUnavailable{false};

    void DispatchPending(FRDGBuilder& GraphBuilder, const FSceneView& View)
    {
        const FSceneInterface* Scene = View.Family != nullptr ? View.Family->Scene : nullptr;
        if (Scene == nullptr || PendingBatches.IsEmpty() || !UE::FXRenderingUtils::RayTracing::HasRayTracingScene(*Scene))
            return;
        const FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(View.GetFeatureLevel());
        if (ShaderMap == nullptr || !ShaderMap->HasShader(&FHerculesRayTraceCS::GetStaticType(), 0))
            return;
        FRDGBufferSRVRef TLAS = UE::FXRenderingUtils::RayTracing::GetRayTracingSceneViewRDG(*Scene, View);
        if (TLAS == nullptr)
            return;
        TShaderMapRef<FHerculesRayTraceCS> Shader(ShaderMap);
        FRHIShaderResourceView* Metadata = UE::FXRenderingUtils::RayTracing::GetInlineRayTracingBindingDataBuffer(*Scene);
        if (Metadata == nullptr)
            Metadata = GEmptyStructuredBufferWithUAV->ShaderResourceViewRHI.GetReference(); // no normals
        TRDGUniformBufferRef<FSceneUniformParameters> SceneUniformBuffer = GetSceneUniformBufferRef(GraphBuilder, View);
        const FVector PreViewTranslation = View.ViewMatrices.GetPreViewTranslation();

        RDG_EVENT_SCOPE(GraphBuilder, "HerculesRayTrace");
        for (int32 Index = 0; Index < PendingBatches.Num();) {
            FBatch& Batch = PendingBatches[Index];
            if (Batch.Scene != Scene) {
                ++Index;
                continue;
            }
            const uint32 RayCount = Batch.Rays.Num();
            FRDGBufferRef Rays = CreateStructuredBuffer(GraphBuilder, TEXT("HerculesRays"), sizeof(FHerculesGpuRay), RayCount,
                                                        Batch.Rays.GetData(), RayCount * sizeof(FHerculesGpuRay));
            const uint32 NoExclusion = 0;
            FRDGBufferRef Excluded = CreateStructuredBuffer(GraphBuilder, TEXT("HerculesExcludedComponents"), sizeof(uint32),
                                                            FMath::Max(Batch.Excluded.Num(), 1),
                                                            Batch.Excluded.Num() > 0 ? Batch.Excluded.GetData() : &NoExclusion,
                                                            FMath::Max(Batch.Excluded.Num(), 1) * sizeof(uint32));
            FRDGBufferRef Hits = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(FHerculesGpuHit), RayCount), TEXT("HerculesRayHits"));

            FHerculesRayTraceCS::FParameters* Parameters = GraphBuilder.AllocParameters<FHerculesRayTraceCS::FParameters>();
            Parameters->View = View.ViewUniformBuffer;
            Parameters->Scene = SceneUniformBuffer;
            Parameters->TLAS = TLAS;
            Parameters->RayTracingSceneMetadata = Metadata;
            Parameters->Rays = GraphBuilder.CreateSRV(Rays);
            Parameters->ExcludedComponentIds = GraphBuilder.CreateSRV(Excluded);
            Parameters->Hits = GraphBuilder.CreateUAV(Hits);
            // Rays are cast in translated world space (relative to the view), which keeps float precision
            Parameters->TranslatedOrigin = FVector3f(Batch.Origin + PreViewTranslation);
            Parameters->RayCount = RayCount;
            Parameters->ExcludedCount = Batch.Excluded.Num();
            Parameters->MaxRetraces = MaxRetraces;
            FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("HerculesRayTrace (%u rays)", RayCount), ERDGPassFlags::Compute,
                                         Shader, Parameters, FComputeShaderUtils::GetGroupCount(RayCount, FHerculesRayTraceCS::ThreadGroupSize));

            FInFlight Cast;
            Cast.Readback = MakeUnique<FRHIGPUBufferReadback>(TEXT("HerculesRayHitsReadback"));
            AddEnqueueCopyPass(GraphBuilder, Cast.Readback.Get(), Hits, RayCount * sizeof(FHerculesGpuHit));
            Cast.RayCount = RayCount;
            Cast.OnComplete = MoveTemp(Batch.OnComplete);
            InFlight.Add(MoveTemp(Cast));
            PendingBatches.RemoveAt(Index);
            --PendingCount;
        }
    }

    // Render thread, once per game frame: collects finished casts and gives up on batches that
    // found no ray tracing structure
    void PollRenderThread()
    {
        for (int32 Index = 0; Index < InFlight.Num();) {
            FInFlight& Cast = InFlight[Index];
            if (!Cast.Readback->IsReady()) {
                ++Index;
                continue;
            }
            FResult Result;
            Result.bCast = true;
            Result.Hits.SetNumUninitialized(Cast.RayCount);
            const uint32 NumBytes = Cast.RayCount * sizeof(FHerculesGpuHit);
            FMemory::Memcpy(Result.Hits.GetData(), Cast.Readback->Lock(NumBytes), NumBytes);
            Cast.Readback->Unlock();
            Result.OnComplete = MoveTemp(Cast.OnComplete);
            Results.Enqueue(MoveTemp(Result));
            InFlight.RemoveAt(Index);
        }
        for (int32 Index = 0; Index < PendingBatches.Num();) {
            if (GFrameCounterRenderThread - PendingBatches[Index].QueuedFrame <= MaxWaitFrames) {
                ++Index;
                continue;
            }
            FResult Result;
            Result.OnComplete = MoveTemp(PendingBatches[Index].OnComplete);
            Results.Enqueue(MoveTemp(Result));
            PendingBatches.RemoveAt(Index);
            --PendingCount;
            UnavailableUntilSeconds = FPlatformTime::Seconds() + UnavailableSeconds;
            if (!WarnedUnavailable.exchange(true))
                UE_LOG(LogHerculesRayTracing, Warning, TEXT("GPU ray casting: the scene has no ray tracing structure (no ray traced effects are enabled); rays are traced on the CPU, retrying every %.0f s"), UnavailableSeconds);
        }
    }

    class FHerculesRayCastExtension : public FSceneViewExtensionBase
    {
    public:
        FHerculesRayCastExtension(const FAutoRegister& AutoRegister)
            : FSceneViewExtensionBase(AutoRegister)
        {
        }

        virtual ESceneViewExtensionFlags GetFlags() const override
        {
            return ESceneViewExtensionFlags::SubscribesToPostTLASBuild | ESceneViewExtensionFlags::RequiresHardwareInlineRayTracing;
        }

        virtual void PostTLASBuild_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& View) override
        {
            DispatchPending(GraphBuilder, View);
        }

    protected:
        // Only the game viewport (scene captures, such as a segmentation view that shows only some
        // components, are not used), and only while rays wait
        virtual bool IsActiveThisFrame_Internal(const FSceneViewExtensionContext& Context) const override
        {
            return Context.Viewport != nullptr && PendingCount.load() > 0;
        }
    };

    TSharedPtr<FHerculesRayCastExtension, ESPMode::ThreadSafe> Extension;
    FTSTicker::FDelegateHandle TickerHandle;

    bool Tick(float)
    {
        ENQUEUE_RENDER_COMMAND(HerculesPollRayCasts)([](FRHICommandListImmediate&) { PollRenderThread(); });
        FResult Result;
        while (Results.Dequeue(Result))
            Result.OnComplete(Result.bCast, MoveTemp(Result.Hits));
        return true;
    }

    // Releases the readbacks while the renderer still exists
    void Stop()
    {
        if (!Extension.IsValid())
            return;
        FTSTicker::GetCoreTicker().RemoveTicker(TickerHandle);
        ENQUEUE_RENDER_COMMAND(HerculesStopRayCasts)([](FRHICommandListImmediate&) {
            PendingBatches.Empty();
            InFlight.Empty();
        });
        FlushRenderingCommands();
        Results.Empty();
        PendingCount = 0;
        Extension.Reset();
    }

    void EnsureStarted()
    {
        if (Extension.IsValid())
            return;
        Extension = FSceneViewExtensions::NewExtension<FHerculesRayCastExtension>();
        TickerHandle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateStatic(&Tick));
        FCoreDelegates::OnEnginePreExit.AddStatic(&Stop);
        // Sensors need every ray traced object, not only those near the camera
        if (IConsoleVariable* Culling = IConsoleManager::Get().FindConsoleVariable(TEXT("r.RayTracing.Culling"))) {
            if (Culling->GetInt() != 0) {
                Culling->Set(0, ECVF_SetByCode);
                UE_LOG(LogHerculesRayTracing, Log, TEXT("GPU ray casting: r.RayTracing.Culling set to 0, so objects away from the camera stay in the ray tracing scene"));
            }
        }
        UE_LOG(LogHerculesRayTracing, Log, TEXT("GPU ray casting started"));
    }
}

bool FHerculesGpuRayCaster::IsSupported()
{
    // The shader exists only where ray tracing is enabled for the project and inline ray tracing is supported
    const FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
    return IsRayTracingEnabled() && GRHISupportsInlineRayTracing && ShaderMap != nullptr
        && ShaderMap->HasShader(&FHerculesRayTraceCS::GetStaticType(), 0)
        && FPlatformTime::Seconds() >= UnavailableUntilSeconds.load();
}

void FHerculesGpuRayCaster::Submit(UWorld& World, const FVector& Origin, TArray<FHerculesGpuRay>&& Rays,
                                   TArray<uint32> ExcludedComponentIds, FOnComplete&& OnComplete)
{
    if (!IsInGameThread()) {
        // e.g. from AirSim's physics thread
        AsyncTask(ENamedThreads::GameThread, [WeakWorld = TWeakObjectPtr<UWorld>(&World), Origin, Rays = MoveTemp(Rays),
                                              Excluded = MoveTemp(ExcludedComponentIds), OnComplete = MoveTemp(OnComplete)]() mutable {
            if (UWorld* GameWorld = WeakWorld.Get())
                Submit(*GameWorld, Origin, MoveTemp(Rays), MoveTemp(Excluded), MoveTemp(OnComplete));
            else
                OnComplete(false, {});
        });
        return;
    }
    if (Rays.Num() == 0) {
        OnComplete(true, {});
        return;
    }
    if (!IsSupported() || World.Scene == nullptr) {
        OnComplete(false, {});
        return;
    }
    EnsureStarted();
    ExcludedComponentIds.Sort();
    FBatch Batch;
    Batch.Scene = World.Scene;
    Batch.Origin = Origin;
    Batch.Rays = MoveTemp(Rays);
    Batch.Excluded = MoveTemp(ExcludedComponentIds);
    Batch.OnComplete = MoveTemp(OnComplete);
    ++PendingCount;
    ENQUEUE_RENDER_COMMAND(HerculesQueueRays)([Batch = MoveTemp(Batch)](FRHICommandListImmediate&) mutable {
        Batch.QueuedFrame = GFrameCounterRenderThread;
        PendingBatches.Add(MoveTemp(Batch));
    });
}

#else // RHI_RAYTRACING

bool FHerculesGpuRayCaster::IsSupported()
{
    return false;
}

void FHerculesGpuRayCaster::Submit(UWorld&, const FVector&, TArray<FHerculesGpuRay>&&, TArray<uint32>, FOnComplete&& OnComplete)
{
    OnComplete(false, {});
}

#endif // RHI_RAYTRACING
