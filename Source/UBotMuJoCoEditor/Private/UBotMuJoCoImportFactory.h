#pragma once

#include "CoreMinimal.h"
#include "EditorReimportHandler.h"
#include "Factories/Factory.h"
#include "UBotMuJoCoImportFactory.generated.h"

/** Imports native rigid/articulated MJCF or URDF into a self-contained Physics model asset. */
UCLASS()
class UUBotMuJoCoImportFactory : public UFactory, public FReimportHandler
{
    GENERATED_BODY()
  public:
    UUBotMuJoCoImportFactory();
    virtual bool FactoryCanImport(const FString &Filename) override;
    virtual UObject *FactoryCreateFile(UClass *InClass, UObject *InParent, FName InName, EObjectFlags Flags,
                                       const FString &Filename, const TCHAR *Parms, FFeedbackContext *Warn,
                                       bool &bOutOperationCanceled) override;
    virtual bool CanReimport(UObject *Object, TArray<FString> &OutFilenames) override;
    virtual void SetReimportPaths(UObject *Object, const TArray<FString> &NewReimportPaths) override;
    virtual EReimportResult::Type Reimport(UObject *Object) override;
    virtual int32 GetPriority() const override
    {
        return ImportPriority;
    }

  private:
    TMap<TWeakObjectPtr<UObject>, FString> SessionSourcePaths;
};
