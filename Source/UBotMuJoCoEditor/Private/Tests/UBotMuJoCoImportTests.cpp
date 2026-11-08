#if WITH_DEV_AUTOMATION_TESTS

#include "Interfaces/IPluginManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "UBotMuJoCoImportFactory.h"
#include "UBotMuJoCoModelData.h"
#include "UBotPhysicsModel.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotMuJoCoEditorImportTest, "UBotMuJoCo.Editor.ImportAndReimport",
                                 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotMuJoCoEditorImportTest::RunTest(const FString& Parameters)
{
    const auto Plugin = IPluginManager::Get().FindPlugin(TEXT("UBotMuJoCo"));
    if (!TestTrue(TEXT("Plugin located"), Plugin.IsValid()))
        return false;
    const FString Filename = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Tests/Native/fixtures/pendulum.xml"));
    auto* Factory = NewObject<UUBotMuJoCoImportFactory>();
    if (!TestTrue(TEXT("Recognizes MJCF document"), Factory->FactoryCanImport(Filename)))
        return false;
    const FString PackageName = TEXT("/Temp/UBotMuJoCoImport_") + FGuid::NewGuid().ToString(EGuidFormats::Digits);
    UPackage* Package = CreatePackage(*PackageName);
    bool bCanceled = false;
    auto* Asset = Cast<UUBotPhysicsModelAsset>(
        Factory->FactoryCreateFile(UUBotPhysicsModelAsset::StaticClass(), Package, TEXT("Imported"),
                                   RF_Public | RF_Standalone, Filename, nullptr, GWarn, bCanceled));
    if (!TestNotNull(TEXT("Creates a common Physics model"), Asset))
        return false;
    auto* Data = Cast<UUBotMuJoCoModelData>(Asset->Model.Extension);
    if (!TestNotNull(TEXT("Native extension exists"), Data))
        return false;
    TestNotNull(TEXT("Global options preset generated"), Data->SuggestedPreset.Get());
#if WITH_EDITORONLY_DATA
    TestEqual(TEXT("Persisted source contains only basename"), Data->SourceFilename, FString(TEXT("pendulum.xml")));
#endif
    TestFalse(TEXT("Native extension is not original MJCF"), Data->NativeData.Contains(TEXT("<mujoco")));
    TestTrue(TEXT("Has common joint descriptions"), Asset->Model.Joints.Num() > 0);
    const auto OldRevision = Asset->Model.Revision;
    const auto OldBodyId = Asset->Model.Bodies[0].Id;
    TArray<FString> Sources;
    TestTrue(TEXT("Reimport recognized"), Factory->CanReimport(Asset, Sources));
    TestEqual(TEXT("Session source retained"), Sources.Num(), 1);
    TestTrue(TEXT("Reimport succeeds"), Factory->Reimport(Asset) == EReimportResult::Succeeded);
    TestTrue(TEXT("Reimport creates a new definition revision"), Asset->Model.Revision != OldRevision);
    TestEqual(TEXT("Stable source body retains identity"), Asset->Model.Bodies[0].Id, OldBodyId);
    const auto ValidRevision = Asset->Model.Revision;
    Factory->SetReimportPaths(Asset, {FPaths::Combine(FPaths::GetPath(Filename), TEXT("missing_input.xml"))});
    AddExpectedError(TEXT("Model reimport failed:"), EAutomationExpectedErrorFlags::Contains, 1);
    TestTrue(TEXT("Missing input fails reimport"), Factory->Reimport(Asset) == EReimportResult::Failed);
    TestTrue(TEXT("Failed reimport keeps published asset"), Asset->Model.Revision == ValidRevision);
    AddExpectedError(TEXT("Model import failed:"), EAutomationExpectedErrorFlags::Contains, 1);
    TestNull(TEXT("Failed overwrite import returns no replacement"),
             Factory->FactoryCreateFile(
                 UUBotPhysicsModelAsset::StaticClass(), Package, TEXT("Imported"), RF_Public | RF_Standalone,
                 FPaths::Combine(FPaths::GetPath(Filename), TEXT("missing_input.xml")), nullptr, GWarn, bCanceled));
    TestTrue(TEXT("Failed overwrite preserves existing definition"), Asset->Model.Revision == ValidRevision);
    TestTrue(TEXT("Failed overwrite keeps the existing asset exported"), Asset->HasAllFlags(RF_Public | RF_Standalone));
    Asset->ClearFlags(RF_Public | RF_Standalone);
    if (Data->SuggestedPreset)
        Data->SuggestedPreset->ClearFlags(RF_Public | RF_Standalone);
    return true;
}

#endif
