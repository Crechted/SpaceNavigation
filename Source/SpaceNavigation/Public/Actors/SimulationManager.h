// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Core/SimTypes.h"
#include "Core/Delegates.h"
#include "SimulationManager.generated.h"

class URoutePlanner;
class AStarBody;
class ACelestialBody;
class ASpacecraft;
class UScenarioGenerator;
class URouteCriterion;
class FJsonObject;
/**
 * Менеджер симуляции.
 *
 * Отвечает за:
 *  - генерацию сценария и создание тел;
 *  - планирование маршрута по выбранному критерию;
 *  - пошаговую симуляцию полёта в приближении patched conics с переключением
 *    центрального тела при пересечении сфер влияния;
 *  - ускорение времени;
 *  - проверку условий успеха и неудачи;
 *  - отладочную визуализацию.
 */
UCLASS(Blueprintable)
class SPACENAVIGATION_API ASimulationManager : public AActor
{
    GENERATED_BODY()

#pragma region ParameterProperies

public:
    /** Множитель ускорения времени. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation", meta = (ClampMin = "0.0"))
    double TimeScale = 1.0;

    /** Максимальный шаг интегрирования, с. Защищает от слишком крупных шагов. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation", meta = (Units = "s"))
    double MaxIntegrationStep = 3600.0;

    /** Останавливать симуляцию автоматически по достижении границы системы. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation")
    bool bAutoStopOnCompletion = true;

    /**
     * Относительный допуск достижения границы системы (доля радиуса границы).
     * Финальный переход к границе — эллипс, апогей которого лишь касается
     * границы; строгое >= практически никогда не срабатывает из-за смещения
     * аппарата на радиус парковочной орбиты и погрешности интегрирования.
     * Граница считается достигнутой, если аппарат приблизился к ней на эту
     * долю (но не менее 100000 км).
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Simulation", meta = (ClampMin = "0.0"))
    double BoundaryToleranceFraction = 1.0e-4;

#pragma endregion

#pragma region DebugDrawingProperties

public:
    // --- Отладочная визуализация ---

    /** Включить отладочную визуализацию. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    bool bDrawDebug = true;

    /** Километров в одной мировой единице Unreal. Задаёт масштаб сцены. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta = (Units = "km"))
    double KmPerWorldUnit = 1.0e6;

    /** Цвет траектории аппарата. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    FColor TrajectoryColor = FColor::Green;

    /** Цвет сфер влияния. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
    FColor SOIColor = FColor(80, 80, 80);

    /**
     * Длина отладочных стрелок у планет, мировых единиц:
     * направление скорости (куда движется планета) и нормаль плоскости орбиты
     * (ось её вращения вокруг звезды).
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta = (ClampMin = "0.1"))
    double DebugArrowLength = 250.0;

    /** Радиус отладочной сферы в позиции аппарата, мировых единиц. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta = (ClampMin = "0.1"))
    double DebugSpacecraftRadius = 5.0;

protected:
    /** Метки событий для отладки и вывода в UI. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Log")
    TArray<FString> Logs;

private:
    /** История пройденного пути для отрисовки траектории. */
    TArray<FVector> TrajectoryHistory;

    /** Признак, что импульс отхода текущего манёвра уже применён. */
    bool bDepartureImpulseFired = false;

    /**
     * Индекс тела, чью сферу влияния игнорируем как центральную до выхода из
     * неё. Задаётся при «прямом» гелиоцентрическом отходе: аппарат мгновенно
     * получает плановую гелиоцентрическую скорость и летит вокруг звезды, и
     * планета отправления не должна его «захватывать» обратно, пока он внутри
     * её сферы влияния.
     */
    int32 DirectDepartureIgnoreBodyIndex = INDEX_NONE;

#pragma endregion

#pragma region StateProperies

public:
    // --- Состояние ---

    /** Текущее состояние симуляции. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
    ESimState SimState = ESimState::Idle;

    /** Итоговая причина завершения. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
    ESimOutcome Outcome = ESimOutcome::None;

    /** Текущее время симуляции, с. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "s"))
    double SimulationTime = 0.0;

    /** Граница системы, км. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "km"))
    double SystemBoundary = 0.0;

    /** Текущее расстояние аппарата от звезды, км. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (Units = "km"))
    double CurrentDistanceFromStar = 0.0;

    /** Удалённость от звезды в процентах от границы системы. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State", meta = (ClampMin = "0.0", ClampMax = "100.0"))
    double ProgressPercent = 0.0;

    /*/** Найденный план маршрута. #1#
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Route")
    FRoutePlan RoutePlan;

    /** Все найденные допустимые маршруты (для сравнения критериев). #1#
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Route")
    TArray<FRoutePlan> CandidateRoutes;*/

    /** Актуальный маршрут, по которому выполняется полёт. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Route")
    TArray<int32> ActiveRoute;

    /** Индекс текущего участка маршрута. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Route")
    int32 CurrentManeuverIndex = 0;

    /** Прошедшее время текущего участка, с. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Route", meta = (Units = "s"))
    double CurrentManeuverElapsed = 0.0;

#pragma endregion

#pragma region ObjectProperties

public:
    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "ObjectProperties")
    TSubclassOf<UScenarioGenerator> ScenarioGeneratorClass;

    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "ObjectProperties")
    TSubclassOf<URoutePlanner> RoutePlannerClass;

    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "ObjectProperties")
    TSubclassOf<ASpacecraft> SpacecraftClass;

    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "ObjectProperties")
    TSubclassOf<ACelestialBody> CelestialBodyClass;

    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "ObjectProperties")
    TSubclassOf<AStarBody> StarClass;

    /** Генератор сценария. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "References")
    TObjectPtr<UScenarioGenerator> ScenarioGenerator = nullptr;

    /** Генератор сценария. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "References")
    TObjectPtr<URoutePlanner> RoutePlanner = nullptr;

    /** Аппарат. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "References")
    TObjectPtr<ASpacecraft> Spacecraft = nullptr;

    /** Созданные тела системы: Bodies[0] — звезда. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "References")
    TArray<TObjectPtr<ACelestialBody>> Bodies;

#pragma endregion

#pragma region DelegateProperties

public:
    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "Delegates")
    FStringDynamicDelegate OnLogAdded;

    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "Delegates")
    FSimpleDynamicDelegate OnScenarioGenerated;

    /** Начало фонового планирования маршрута (поиск запущен). */
    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "Delegates")
    FSimpleDynamicDelegate OnPlanningStarted;

    /**
     * Завершение фонового планирования маршрута.
     * @param bSuccess true, если допустимый маршрут найден.
     */
    UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "Delegates")
    FBoolDynamicDelegate OnPlanningCompleted;

#pragma endregion

#pragma region OverridedFunctions

public:
    ASimulationManager();

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

#pragma endregion

#pragma region ControlFunctions

public:
    // --- Управление ---

    /** Сгенерировать сценарий и создать тела. */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void GenerateScenario();

    /** Проверка, что сценарий был сгенерирован. */
    UFUNCTION(BlueprintPure, Category = "Simulation")
    bool ScenarioWasGenerated() const;

    /** Спланировать маршрут по активному критерию. */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void PlanRoute();

    /** Проверка, что маршрут спланирован. */
    UFUNCTION(BlueprintPure, Category = "Simulation")
    bool HasPlanRoute() const;

    /** Идёт ли сейчас фоновое планирование маршрута. */
    UFUNCTION(BlueprintPure, Category = "Simulation")
    bool IsRoutePlanning() const;

    /** Запустить симуляцию по спланированному маршруту. */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void StartSimulation();

    /** Остановить симуляцию. */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void StopSimulation();

    /** Сбросить состояние к началу. */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void ResetSimulation();

    /** Выполнить один шаг симуляции фиксированной длительности, с. */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void StepSimulation(double DeltaTimeSeconds);

    /** Установить множитель ускорения времени. */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void SetTimeScale(double NewTimeScale);

    /**
     * Заменить набор критериев оптимизации и перепланировать маршрут.
     * Пустой массив восстанавливает пресет "Balanced".
     */
    UFUNCTION(BlueprintCallable, Category = "Simulation")
    void SetCriteria(const TArray<URouteCriterion*>& NewCriteria);

    /**
     * Получение текущий критериев
     */
    UFUNCTION(BlueprintPure, Category = "Simulation")
    TArray<URouteCriterion*> GetCriteria() const;

    /** Получить состояние в удобном виде для интерфейса. */
    UFUNCTION(BlueprintPure, Category = "Simulation")
    FString GetStateSummary() const;

    UFUNCTION(BlueprintCallable, Category = "Logs")
    void AddLog(const FString& LogText);

private:
    /** Обработчик старта фонового поиска: дублирует событие для UI. */
    UFUNCTION()
    void HandlePlannerPlanningStarted();

    /**
     * Обработчик завершения фонового поиска: публикует результаты в RoutePlan
     * и CandidateRoutes, логирует итоговый маршрут и дублирует событие.
     */
    UFUNCTION()
    void HandlePlannerPlanningCompleted(bool bSuccess);

    void RemoveBodies();

    void CreateBodies();

    /** Найти тело, в сферу влияния которого попадает аппарат. */
    ACelestialBody* FindCentralBodyBySOI() const;

    /** Переключить центральное тело, пересчитав относительное состояние. */
    void SwitchCentralBody(ACelestialBody* NewCentralBody);

    bool CheckCollision() const;

    bool CheckBoundaryReached() const;

    bool CheckTimeLimit() const;

    bool CheckFuelExhausted() const;

    void FinishSimulation(ESimState FinalState, ESimOutcome Reason);

    void UpdateBodiesTransforms();

    void UpdateSpacecraftTransform();

    void DrawDebugVisualization();

    void ApplyManeuverImpulse();

    void AdvanceToNextManeuver();

    bool IsFinalManeuver() const;

    void PrepareActiveRoute();

#pragma endregion

#pragma region ConfigFunctions

public:
    /**
     * Открыть системный диалог выбора файла конфигурации.
     * @param bForSaving true — диалог "Сохранить как", false — диалог открытия.
     * @param OutFilePath Результат: полный путь выбранного файла.
     * @return true, если пользователь выбрал файл.
     */
    UFUNCTION(BlueprintCallable, Category = "Simulation|Config")
    bool ChooseConfigFilePath(bool bForSaving, FString& OutFilePath);

    /**
     * Собрать конфигурацию (сценарий, аппарат, планировщик с критериями) в JSON-строку.
     */
    UFUNCTION(BlueprintCallable, Category = "Simulation|Config")
    FString BuildConfigJson();

    /**
     * Применить конфигурацию из JSON-строки и перепланировать маршрут.
     */
    UFUNCTION(BlueprintCallable, Category = "Simulation|Config")
    bool ApplyConfigJson(const FString& Json);

    /**
     * Диалог "Сохранить как": записать конфигурацию в JSON-файл,
     * вернуть собранную JSON-строку.
     */
    UFUNCTION(BlueprintCallable, Category = "Simulation|Config")
    bool ExportConfigToJsonFile(FString& OutJson);

    /**
     * Диалог "Открыть": прочитать JSON-файл, применить конфигурацию,
     * вернуть JSON-строку файла.
     */
    UFUNCTION(BlueprintCallable, Category = "Simulation|Config")
    bool ImportConfigFromJsonFile(FString& OutJson);

private:
    TSharedRef<FJsonObject> BuildScenarioJson() const;
    TSharedRef<FJsonObject> BuildSpacecraftJson() const;
    TSharedRef<FJsonObject> BuildPlannerJson() const;

    bool ApplyScenarioJson(const TSharedPtr<FJsonObject>& Json);
    bool ApplySpacecraftJson(const TSharedPtr<FJsonObject>& Json);
    bool ApplyPlannerJson(const TSharedPtr<FJsonObject>& Json);

    URouteCriterion* CreateCriterionFromJson(const TSharedPtr<FJsonObject>& CriterionJson);

#pragma endregion

#pragma region UtilityFunctions

private:
    bool AllPropertiesIsValid();
#pragma endregion
};