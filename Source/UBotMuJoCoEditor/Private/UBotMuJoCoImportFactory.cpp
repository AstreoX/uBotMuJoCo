#include "UBotMuJoCoImportFactory.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/PackageName.h"
#include "Misc/Paths.h"
#include "UBotMuJoCo.h"
#include "UBotMuJoCoImport.h"
#include "UBotMuJoCoModelData.h"
#include "UBotPhysicsModel.h"
#include "XmlFile.h"
#include "UObject/SoftObjectPath.h"

namespace
{
// Shared by factory instances and never serialized into a model asset.
TMap<FSoftObjectPath, FString> SessionSourcePaths;
/** Collect only explicit dependencies. Never bundle the source directory wholesale. */
struct FResourceCollector
{
    FString RootDirectory;
    TArray<FUBotMuJoCoResource> Resources;
    TArray<FString> Visited;
    FString MeshDirectory, TextureDirectory, AssetDirectory;
    FText Error;

    bool AddFile(const FString& AbsolutePath, const FString& VirtualPath, bool bXML)
    {
        FString Key = VirtualPath;
        FPaths::NormalizeFilename(Key);
        FPaths::CollapseRelativeDirectories(Key);
        if (Visited.ContainsByPredicate(
                [&Key](const FString& Name) { return Name.Equals(Key, ESearchCase::CaseSensitive); }))
            return true;
        FUBotMuJoCoResource R;
        R.Name = Key;
        if (!FFileHelper::LoadFileToArray(R.Bytes, *AbsolutePath))
        {
            Error = FText::Format(INVTEXT("Cannot read referenced resource '{0}'."), FText::FromString(Key));
            return false;
        }
        Visited.Add(Key);
        Resources.Add(MoveTemp(R));
        if (!bXML)
            return true;
        FString XML;
        if (!FFileHelper::LoadFileToString(XML, *AbsolutePath))
            return false;
        FXmlFile Document(XML, EConstructMethod::ConstructFromBuffer);
        if (!Document.IsValid())
        {
            Error = FText::FromString(Document.GetLastError());
            return false;
        }
        return Visit(Document.GetRootNode(), AbsolutePath, Key);
    }

    bool Visit(const FXmlNode* Node, const FString& CurrentAbsolute, const FString& CurrentVirtual)
    {
        if (!Node)
            return true;
        if (Node->GetTag() == TEXT("compiler"))
        {
            MeshDirectory = Node->GetAttribute(TEXT("meshdir"));
            TextureDirectory = Node->GetAttribute(TEXT("texturedir"));
            AssetDirectory = Node->GetAttribute(TEXT("assetdir"));
            if ((!MeshDirectory.IsEmpty() && !FPaths::IsRelative(MeshDirectory)) ||
                (!TextureDirectory.IsEmpty() && !FPaths::IsRelative(TextureDirectory)) ||
                (!AssetDirectory.IsEmpty() && !FPaths::IsRelative(AssetDirectory)))
            {
                Error = INVTEXT("Resource directories must be relative so the imported asset remains portable.");
                return false;
            }
        }
        FString File = Node->GetAttribute(TEXT("file"));
        if (File.IsEmpty() && (Node->GetTag() == TEXT("mesh") || Node->GetTag() == TEXT("texture")))
            File = Node->GetAttribute(TEXT("filename"));
        if (!File.IsEmpty())
        {
            if (!FPaths::IsRelative(File) || File.Contains(TEXT("://")))
            {
                Error = INVTEXT("Resource references must use relative files; resolve package URIs before importing.");
                return false;
            }
            const bool bInclude = Node->GetTag() == TEXT("include");
            FString Relative = File;
            FString Absolute;
            if (bInclude)
            {
                Relative = FPaths::Combine(FPaths::GetPath(CurrentVirtual), File);
                Absolute = FPaths::Combine(FPaths::GetPath(CurrentAbsolute), File);
            }
            else
            {
                const FString Directory = Node->GetTag() == TEXT("texture") ? TextureDirectory : MeshDirectory;
                Relative = FPaths::Combine(Directory.IsEmpty() ? AssetDirectory : Directory, File);
                Absolute = FPaths::Combine(RootDirectory, Relative);
            }
            if (!AddFile(Absolute, Relative, bInclude))
                return false;
        }
        for (const FXmlNode* Child : Node->GetChildrenNodes())
            if (!Visit(Child, CurrentAbsolute, CurrentVirtual))
                return false;
        return true;
    }
};

bool ImportAsset(const FString& Filename, UUBotPhysicsModelAsset& Asset, UUBotMuJoCoSolverPreset& Preset, FText& Error)
{
    FResourceCollector Collector;
    Collector.RootDirectory = FPaths::GetPath(FPaths::ConvertRelativePathToFull(Filename));
    const FString Base = FPaths::GetCleanFilename(Filename);
    if (!Collector.AddFile(FPaths::ConvertRelativePathToFull(Filename), Base, true))
    {
        Error = Collector.Error;
        return false;
    }
    FUBotPhysicsModel Candidate;
    if (!UBot::MuJoCo::ImportModel(Base, Collector.Resources, &Asset, &Preset, Candidate, Error))
        return false;
#if WITH_EDITORONLY_DATA
    CastChecked<UUBotMuJoCoModelData>(Candidate.Extension)->SourceFilename = Base;
#endif
    Asset.Modify();
    Preset.Modify();
    Asset.Model = MoveTemp(Candidate);
    Asset.MarkPackageDirty();
    Preset.MarkPackageDirty();
    return true;
}
} // namespace

UUBotMuJoCoImportFactory::UUBotMuJoCoImportFactory()
{
    SupportedClass = UUBotPhysicsModelAsset::StaticClass();
    bCreateNew = false;
    bEditorImport = true;
    Formats.Add(TEXT("xml;MuJoCo MJCF model"));
    Formats.Add(TEXT("urdf;Robot model"));
    ImportPriority = DefaultImportPriority + 10;
}

bool UUBotMuJoCoImportFactory::FactoryCanImport(const FString& Filename)
{
    if (FPaths::GetExtension(Filename).Equals(TEXT("urdf"), ESearchCase::IgnoreCase))
        return true;
    if (!FPaths::GetExtension(Filename).Equals(TEXT("xml"), ESearchCase::IgnoreCase))
        return false;
    FXmlFile Document(Filename);
    return Document.IsValid() && Document.GetRootNode() && Document.GetRootNode()->GetTag() == TEXT("mujoco");
}

UObject* UUBotMuJoCoImportFactory::FactoryCreateFile(UClass* InClass, UObject* Parent, FName Name, EObjectFlags Flags,
                                                     const FString& Filename, const TCHAR* Parms,
                                                     FFeedbackContext* Warn, bool& bCanceled)
{
    bCanceled = false;
    // Compile into transient objects before touching an existing destination or preset.
    auto* Candidate = NewObject<UUBotPhysicsModelAsset>();
    auto* CandidatePreset = NewObject<UUBotMuJoCoSolverPreset>();
    FText Error;
    if (!ImportAsset(Filename, *Candidate, *CandidatePreset, Error))
    {
        UE_LOG(LogUBotMuJoCo, Error, TEXT("Model import failed: %s"), *Error.ToString());
        return nullptr;
    }
    auto* Asset = FindObject<UUBotPhysicsModelAsset>(Parent, *Name.ToString());
    if (!Asset)
        Asset = NewObject<UUBotPhysicsModelAsset>(Parent, Name, Flags);
    const FString PresetPackageName = Parent->GetOutermost()->GetName() + TEXT("_SolverPreset");
    UPackage* PresetPackage = CreatePackage(*PresetPackageName);
    const FString PresetName = FPackageName::GetLongPackageAssetName(PresetPackageName);
    auto* Preset = FindObject<UUBotMuJoCoSolverPreset>(PresetPackage, *PresetName);
    const bool bNewPreset = !Preset;
    if (!Preset)
        Preset = NewObject<UUBotMuJoCoSolverPreset>(PresetPackage, *PresetName,
                                                    RF_Public | RF_Standalone | RF_Transactional);
    Asset->Modify();
    Preset->Modify();
    Preset->StepSeconds = CandidatePreset->StepSeconds;
    Preset->GravityMetersPerSecondSquared = CandidatePreset->GravityMetersPerSecondSquared;
    Preset->NativeOptions = CandidatePreset->NativeOptions;
    auto* Extension =
        DuplicateObject<UUBotMuJoCoModelData>(CastChecked<UUBotMuJoCoModelData>(Candidate->Model.Extension), Asset);
    Extension->SuggestedPreset = Preset;
    Asset->Model = MoveTemp(Candidate->Model);
    Asset->Model.Extension = Extension;
    Asset->MarkPackageDirty();
    Preset->MarkPackageDirty();
    SessionSourcePaths.Add(FSoftObjectPath(Asset), Filename);
    if (bNewPreset)
        FAssetRegistryModule::AssetCreated(Preset);
    return Asset;
}

bool UUBotMuJoCoImportFactory::CanReimport(UObject* Object, TArray<FString>& Filenames)
{
    const auto* Asset = Cast<UUBotPhysicsModelAsset>(Object);
    const auto* Data = Asset ? Cast<UUBotMuJoCoModelData>(Asset->Model.Extension) : nullptr;
    if (!Data)
        return false;
    if (const FString* Path = SessionSourcePaths.Find(FSoftObjectPath(Object)))
        Filenames.Add(*Path);
#if WITH_EDITORONLY_DATA
    else
        Filenames.Add(Data->SourceFilename);
#endif
    return true;
}

void UUBotMuJoCoImportFactory::SetReimportPaths(UObject* Object, const TArray<FString>& Paths)
{
    if (Paths.Num() == 1)
        SessionSourcePaths.Add(FSoftObjectPath(Object), Paths[0]);
}

EReimportResult::Type UUBotMuJoCoImportFactory::Reimport(UObject* Object)
{
    auto* Asset = Cast<UUBotPhysicsModelAsset>(Object);
    auto* Data = Asset ? Cast<UUBotMuJoCoModelData>(Asset->Model.Extension) : nullptr;
    if (!Data || !Data->SuggestedPreset)
        return EReimportResult::Failed;
    FString Source;
    if (const FString* Path = SessionSourcePaths.Find(FSoftObjectPath(Object)))
        Source = *Path;
#if WITH_EDITORONLY_DATA
    else
        Source = Data->SourceFilename;
#endif
    if (Source.IsEmpty() || !IFileManager::Get().FileExists(*Source))
    {
        UE_LOG(LogUBotMuJoCo, Error,
               TEXT("Model reimport failed: the source is unavailable. Use Reimport With New File to locate the source "
                    "document."));
        return EReimportResult::Failed;
    }
    FText Error;
    if (!ImportAsset(Source, *Asset, *Data->SuggestedPreset, Error))
    {
        UE_LOG(LogUBotMuJoCo, Error, TEXT("Model reimport failed: %s"), *Error.ToString());
        return EReimportResult::Failed;
    }
    return EReimportResult::Succeeded;
}
