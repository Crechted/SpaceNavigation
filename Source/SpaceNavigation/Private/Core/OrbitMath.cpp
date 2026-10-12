// Copyright Epic Games, Inc. All Rights Reserved.

#include "Core/OrbitMath.h"

#pragma region OrbitMathLibrary

#pragma region GravityFunctions

double UOrbitMath::GravitationalAcceleration(double MassKg, double DistanceKm)
{
    if (DistanceKm <= 0.0)
    {
        return 0.0;
    }

    return SimConstants::GravitationalConstant * MassKg / (DistanceKm * DistanceKm);
}

double UOrbitMath::CircularOrbitalVelocity(double MassKg, double RadiusKm)
{
    if (RadiusKm <= 0.0 || MassKg <= 0.0)
    {
        return 0.0;
    }

    return FMath::Sqrt(SimConstants::GravitationalConstant * MassKg / RadiusKm);
}

double UOrbitMath::EscapeVelocity(double MassKg, double RadiusKm)
{
    if (RadiusKm <= 0.0 || MassKg <= 0.0)
    {
        return 0.0;
    }

    return FMath::Sqrt(2.0 * SimConstants::GravitationalConstant * MassKg / RadiusKm);
}

bool UOrbitMath::IsForceNegligible(double MassKg, double DistanceKm)
{
    return GravitationalAcceleration(MassKg, DistanceKm) < SimConstants::GravitySignificanceThreshold;
}

#pragma endregion

#pragma region SphereOfInfluenceFunctions

double UOrbitMath::ComputeSphereOfInfluence(double BodyMassKg, double CentralMassKg, double SemiMajorAxisKm, double FallbackRadiusKm)
{
    if (BodyMassKg < SimConstants::MinimumBodyMass || CentralMassKg <= 0.0 || SemiMajorAxisKm <= 0.0)
    {
        return FallbackRadiusKm;
    }

    const double MassRatio = BodyMassKg / CentralMassKg;
    if (MassRatio <= 0.0)
    {
        return FallbackRadiusKm;
    }

    return SemiMajorAxisKm * FMath::Pow(MassRatio, SimConstants::SphereOfInfluenceExponent);
}

double UOrbitMath::ComputeSphereOfInfluenceWithThreshold(
    double BodyMassKg, double CentralMassKg, double SemiMajorAxisKm, double FixedRadiusKm, double WeakForceThreshold)
{
    const double Threshold = WeakForceThreshold > 0.0 ? WeakForceThreshold : SimConstants::WeakForceThreshold;

    // Фиксированная сфера влияния значима — оставляем как есть.
    if (FixedRadiusKm >= Threshold)
    {
        return FixedRadiusKm;
    }

    // Значение признано малой силой: пересчитываем по формуле.
    return ComputeSphereOfInfluence(BodyMassKg, CentralMassKg, SemiMajorAxisKm, FixedRadiusKm);
}

#pragma endregion

#pragma region TransferFunctions

FHohmannTransfer UOrbitMath::ComputeHohmannTransfer(double CentralMassKg, double RadiusStartKm, double RadiusEndKm)
{
    FHohmannTransfer Result;

    if (CentralMassKg <= 0.0 || RadiusStartKm <= 0.0 || RadiusEndKm <= 0.0)
    {
        return Result;
    }

    const double Mu = SimConstants::GravitationalConstant * CentralMassKg;

    Result.VelocityStart = CircularOrbitalVelocity(CentralMassKg, RadiusStartKm);
    Result.VelocityEnd = CircularOrbitalVelocity(CentralMassKg, RadiusEndKm);

    // Переходная орбита: перицентр и апоцентр.
    const double TransferPeriapsis = FMath::Min(RadiusStartKm, RadiusEndKm);
    const double TransferApoapsis = FMath::Max(RadiusStartKm, RadiusEndKm);

    Result.TransferVelocityStart = FMath::Sqrt(Mu * (2.0 / TransferPeriapsis - 2.0 / (TransferPeriapsis + TransferApoapsis)));
    Result.TransferVelocityEnd = FMath::Sqrt(Mu * (2.0 / TransferApoapsis - 2.0 / (TransferPeriapsis + TransferApoapsis)));

    Result.DeltaVStart = FMath::Abs(Result.TransferVelocityStart - Result.VelocityStart);
    Result.DeltaVEnd = FMath::Abs(Result.TransferVelocityEnd - Result.VelocityEnd);
    Result.TotalDeltaV = Result.DeltaVStart + Result.DeltaVEnd;

    // Время перехода: половина периода переходной орбиты.
    const double TransferSemiMajorAxis = (TransferPeriapsis + TransferApoapsis) * 0.5;
    const double MeanMotion = FMath::Sqrt(Mu / FMath::Pow(TransferSemiMajorAxis, 3.0));
    Result.TransferTime = UE_PI / MeanMotion;

    return Result;
}

double UOrbitMath::ComputeOptimalTransferDeltaV(double CentralMassKg, double RadiusStartKm, double RadiusEndKm)
{
    if (CentralMassKg <= 0.0 || RadiusStartKm <= 0.0 || RadiusEndKm <= 0.0)
    {
        return 0.0;
    }

    const double HohmannDeltaV = ComputeHohmannTransfer(CentralMassKg, RadiusStartKm, RadiusEndKm).TotalDeltaV;

    // Для больших отношений радиусов дешевле бипараболический переход —
    // предельный случай биэллиптического с бесконечно далёким промежуточным
    // апоцентром. Его суммарный DeltaV равен (sqrt(2) - 1) * (v_c1 + v_c2),
    // где v_c — круговые скорости на начальной и конечной орбитах.
    //
    // Ранее здесь использовалась биэллиптическая формула с промежуточным
    // апоцентром sqrt(Rlow * Rhigh) и удвоенным членом vis-viva (2.0 вместо 1.0
    // при делении на большую полуось). При Rlow < Rhigh подкоренное выражение
    // становилось отрицательным, FMath::Sqrt возвращал NaN, и итоговый DeltaV
    // через FMath::Max(NaN, 0.0) обнулялся — планировщик считал перелёт к
    // границе бесплатным, а аппарат не получал импульса отхода.
    const double Mu = SimConstants::GravitationalConstant * CentralMassKg;
    const double RadiusLow = FMath::Min(RadiusStartKm, RadiusEndKm);
    const double RadiusHigh = FMath::Max(RadiusStartKm, RadiusEndKm);

    const double BiParabolicDeltaV = (FMath::Sqrt(2.0) - 1.0) * (FMath::Sqrt(Mu / RadiusLow) + FMath::Sqrt(Mu / RadiusHigh));

    return FMath::Min(HohmannDeltaV, BiParabolicDeltaV);
}

double UOrbitMath::ComputeTransferTime(double CentralMassKg, double RadiusStartKm, double RadiusEndKm)
{
    return ComputeHohmannTransfer(CentralMassKg, RadiusStartKm, RadiusEndKm).TransferTime;
}

FLambertSolution UOrbitMath::SolveLambert(FVector R1Km, FVector R2Km, double CentralMassKg, double TransferTimeSeconds)
{
    FLambertSolution Result;

    if (CentralMassKg <= 0.0 || TransferTimeSeconds <= 0.0)
    {
        return Result;
    }

    const double Mu = SimConstants::GravitationalConstant * CentralMassKg;
    const double R1M = R1Km.Size();
    const double R2M = R2Km.Size();
    if (R1M <= 0.0 || R2M <= 0.0)
    {
        return Result;
    }

    // Нормаль плоскости дуги: прогрейд (N̂·ẑ ≥ 0), чтобы получить ровно одну дугу.
    const FVector C = FVector::CrossProduct(R1Km, R2Km);
    const double CM = C.Size();
    if (CM < 1.0e-9 * R1M * R2M)
    {
        // Вырождение: угол перелёта ровно 0 или π (коллинеарность).
        // Единственное решение — эллипс Хомана; скорости прогрейд.
        const double A = (R1M + R2M) * 0.5;
        const double VP = FMath::Sqrt(Mu * (2.0 / R1M - 1.0 / A));
        const double VA = FMath::Sqrt(Mu * (2.0 / R2M - 1.0 / A));
        const double Th1 = FMath::Atan2(R1Km.Y, R1Km.X);
        const double Th2 = FMath::Atan2(R2Km.Y, R2Km.X);
        const FVector VDir1(FMath::Cos(Th1 + UE_PI * 0.5), FMath::Sin(Th1 + UE_PI * 0.5), 0.0);
        const FVector VDir2(FMath::Cos(Th2 + UE_PI * 0.5), FMath::Sin(Th2 + UE_PI * 0.5), 0.0);

        Result.DepartureVelocityKmS = VDir1 * VP;
        Result.ArrivalVelocityKmS = VDir2 * VA;
        Result.SemiMajorAxisKm = A;
        Result.Eccentricity = FMath::Abs(R2M - R1M) / (R1M + R2M);
        Result.TransferTime = UE_PI * FMath::Sqrt(A * A * A / Mu);
        Result.DeltaAnomaly = UE_PI;
        Result.bValid = true;
        return Result;
    }

    FVector Nhat = C / CM;
    if (Nhat.Z < 0.0)
    {
        Nhat = -Nhat;
    }

    // Гамма-параметризация (Wikipedia, "Lambert's problem"): семейство
    // кеплеровских переходов между r1 и r2 параметризуется одним числом γ.
    // Эллиптическая ветка (e < 1) существует при γ строго между γ0 и γp:
    //   γ0 = -Q/σ,  γp = √(2Q)/(r1+r2),
    //   Q = r1·r2 из геометрии, σ = N̂·(r1 × r2).
    const double Q = R1M * R2M - FVector::DotProduct(R1Km, R2Km);
    const double Sig = FVector::DotProduct(Nhat, C);
    const double Gamma0 = -Q / Sig;
    const double GammaP = FMath::Sqrt(2.0 * Q) / (R1M + R2M);

    auto ToFKepler = [&Mu, &R1Km, &R2Km, &Nhat, R1M, R2M](double Gamma, double& OutToF, double& OutP, FVector& OutEVec, double& OutE,
                         double& OutA, double& OutNu1, double& OutNu2) -> bool
    {
        const FVector Dr = R2Km - R1Km;
        const double D2 = FVector::DotProduct(Dr, Dr);
        if (D2 <= 0.0)
        {
            return false;
        }
        const double Term =
            R1M * R2M - FVector::DotProduct(R1Km, R2Km) + Gamma * FVector::DotProduct(Nhat, FVector::CrossProduct(R1Km, R2Km));
        const double P = (R1M + R2M) * Term / D2;
        if (P <= 0.0)
        {
            return false;
        }
        const FVector EVec1 = Dr * (R1M - R2M);
        const FVector EVec2 = FVector::CrossProduct(Nhat, Dr) * (-Gamma * (R1M + R2M));
        const FVector EVecVal = (EVec1 + EVec2) / D2;
        const double E = EVecVal.Size();
        if (E >= 1.0)
        {
            return false;
        }
        const double A = P / (1.0 - E * E);

        auto TrueAnomaly = [&Nhat, &EVecVal](const FVector& Rhat) -> double
        {
            const FVector Ex = EVecVal.GetSafeNormal();
            const double CosNu = FVector::DotProduct(Ex, Rhat);
            const double SinNu = FVector::DotProduct(Nhat, FVector::CrossProduct(Ex, Rhat));
            return FMath::Atan2(SinNu, CosNu);
        };

        const FVector R1h = R1Km.GetSafeNormal();
        const FVector R2h = R2Km.GetSafeNormal();
        double Nu1 = TrueAnomaly(R1h);
        double Nu2 = TrueAnomaly(R2h);
        Nu1 = FMath::Fmod(Nu1, 2.0 * UE_PI);
        Nu2 = FMath::Fmod(Nu2, 2.0 * UE_PI);
        if (Nu1 < 0.0) Nu1 += 2.0 * UE_PI;
        if (Nu2 < 0.0) Nu2 += 2.0 * UE_PI;

        // Ожидаемая дуга: прогрейд вокруг N̂ от r1 к r2.
        double DnuGeom = FMath::Acos(FMath::Clamp(FVector::DotProduct(R1h, R2h), -1.0, 1.0));
        if (FVector::DotProduct(Nhat, FVector::CrossProduct(R1h, R2h)) < 0.0)
        {
            DnuGeom = 2.0 * UE_PI - DnuGeom;
        }
        double Dnu = FMath::Fmod(Nu2 - Nu1, 2.0 * UE_PI);
        if (Dnu < 0.0) Dnu += 2.0 * UE_PI;
        if (FMath::Abs(Dnu - DnuGeom) > 1.0e-6 * FMath::Max(1.0, FMath::Max(FMath::Abs(Dnu), FMath::Abs(DnuGeom))))
        {
            return false;  // другая дуга — не наша ветка
        }

        const double S = FMath::Sqrt((1.0 - E) / (1.0 + E));
        double E1 = 2.0 * FMath::Atan2(S * FMath::Sin(Nu1 * 0.5), FMath::Cos(Nu1 * 0.5));
        double E2 = 2.0 * FMath::Atan2(S * FMath::Sin(Nu2 * 0.5), FMath::Cos(Nu2 * 0.5));
        if (E1 < 0.0) E1 += 2.0 * UE_PI;
        if (E2 < 0.0) E2 += 2.0 * UE_PI;
        double DE = FMath::Fmod(E2 - E1, 2.0 * UE_PI);
        if (DE < 0.0) DE += 2.0 * UE_PI;

        const double ToF = FMath::Sqrt(A * A * A / Mu) * (DE - E * (FMath::Sin(E2) - FMath::Sin(E1)));
        if (ToF <= 0.0)
        {
            return false;
        }

        OutToF = ToF;
        OutP = P;
        OutEVec = EVecVal;
        OutE = E;
        OutA = A;
        OutNu1 = Nu1;
        OutNu2 = Nu2;
        return true;
    };

    // Сканирование TOF(γ) на эллиптическом интервале [мин(γ0,γp), макс(γ0,γp)].
    // TOF(γ) монотонно убывает на этом интервале, но численно "взрывается"
    // в бесконечно близкой окрестности параболы γ→γp (e→1, a→∞). Поэтому
    // правая граница рабочей ветки — точка минимума TOF по скану.
    // Scan only the elliptical band between roots of e^2(gamma)=1 to avoid
    // missing the narrow sliver at near-opposition geometry.
    const double GammaLoBound = FMath::Min(Gamma0, GammaP);
    const double GammaHiBound = FMath::Max(Gamma0, GammaP);
    const double D2Dr = 0.0;  // not needed
    const FVector DrVec = R2Km - R1Km;
    const double D2Val = FVector::DotProduct(DrVec, DrVec);
    const double R1MVal = R1M;
    const double R2MVal = R2M;
    const FVector NhatVal = Nhat;
    const FVector A0 = DrVec * ((R1MVal - R2MVal) / (D2Val > 0.0 ? D2Val : 1.0));
    const FVector B0Term = FVector::CrossProduct(NhatVal, DrVec);
    const FVector B0 = B0Term * (-(R1MVal + R2MVal) / (D2Val > 0.0 ? D2Val : 1.0));
    const double AA = FVector::DotProduct(B0, B0);
    const double BB = 2.0 * FVector::DotProduct(A0, B0);
    const double CC = FVector::DotProduct(A0, A0) - 1.0;
    double RootA = GammaLoBound;
    double RootB = GammaHiBound;
    if (FMath::Abs(AA) > 1.0e-18)
    {
        const double Discr = BB * BB - 4.0 * AA * CC;
        if (Discr >= 0.0)
        {
            const double SqrtD = FMath::Sqrt(Discr);
            const double R1r = (-BB + SqrtD) / (2.0 * AA);
            const double R2r = (-BB - SqrtD) / (2.0 * AA);
            RootA = FMath::Min(R1r, R2r);
            RootB = FMath::Max(R1r, R2r);
        }
    }
    double ScanLo = FMath::Min(RootA, RootB);
    double ScanHi = FMath::Max(RootA, RootB);
    if (ScanHi < GammaLoBound) ScanHi = GammaHiBound;
    if (ScanLo > GammaHiBound) ScanLo = GammaLoBound;
    if (ScanLo < GammaLoBound) ScanLo = GammaLoBound;
    if (ScanHi > GammaHiBound) ScanHi = GammaHiBound;
    const double BandWidth = ScanHi - ScanLo;
    const double Margin = BandWidth * 1.0e-3;
    ScanLo -= Margin;
    ScanHi += Margin;
    if (ScanLo < GammaLoBound) ScanLo = GammaLoBound;
    if (ScanHi > GammaHiBound) ScanHi = GammaHiBound;
    const int32 ScanN = 12000;
    double ScanMinToF = TNumericLimits<double>::Max();
    double ScanMaxToF = -TNumericLimits<double>::Max();
    double GammaScanMin = ScanLo;
    double GammaLeft = ScanLo;
    bool bHasValid = false;
    bool bLeftSet = false;
    for (int32 i = 0; i <= ScanN; ++i)
    {
        const double Gamma = ScanLo + (ScanHi - ScanLo) * i / (double)ScanN;
        double ToF = 0.0, P = 0.0, E = 0.0, A = 0.0, Nu1 = 0.0, Nu2 = 0.0;
        FVector EVec = FVector::ZeroVector;
        if (!ToFKepler(Gamma, ToF, P, EVec, E, A, Nu1, Nu2))
        {
            continue;
        }
        if (!bLeftSet)
        {
            GammaLeft = Gamma;
            bLeftSet = true;
        }
        bHasValid = true;
        if (ToF < ScanMinToF)
        {
            ScanMinToF = ToF;
            GammaScanMin = Gamma;
        }
        if (ToF > ScanMaxToF)
        {
            ScanMaxToF = ToF;
        }
    }
    if (!bHasValid || TransferTimeSeconds < ScanMinToF * 0.999 || TransferTimeSeconds > ScanMaxToF * 1.001)
    {
        return Result;
    }

    // Бисекция на убывающей ветке [GammaLeft, GammaScanMin].
    double G1 = GammaLeft;
    double G2 = GammaScanMin;
    const int32 MaxIter = 200;
    double BestToF = 0.0, P = 0.0, A = 0.0, E = 0.0, Nu1 = 0.0, Nu2 = 0.0;
    FVector EVec = FVector::ZeroVector;
    double G = 0.0;
    for (int32 Iter = 0; Iter < MaxIter; ++Iter)
    {
        G = 0.5 * (G1 + G2);
        double ToF = 0.0;
        if (!ToFKepler(G, ToF, P, EVec, E, A, Nu1, Nu2))
        {
            return Result;
        }
        BestToF = ToF;
        if (FMath::Abs(BestToF - TransferTimeSeconds) < 1.0e-10 * TransferTimeSeconds ||
            (G2 - G1) < 1.0e-9 * FMath::Max(1.0, FMath::Abs(G)))
        {
            break;
        }
        if (BestToF > TransferTimeSeconds)
        {
            G1 = G;
        }
        else
        {
            G2 = G;
        }
    }
    G = 0.5 * (G1 + G2);
    if (!ToFKepler(G, BestToF, P, EVec, E, A, Nu1, Nu2))
    {
        return Result;
    }

    auto Velocity = [&P, &Mu, &E, &EVec, &Nhat](const FVector& Rhat, double Nu) -> FVector
    {
        const double VR = FMath::Sqrt(Mu / P) * E * FMath::Sin(Nu);
        const double VT = FMath::Sqrt(Mu / P) * (1.0 + E * FMath::Cos(Nu));
        return Rhat * VR + FVector::CrossProduct(Nhat, Rhat) * VT;
    };

    Result.DepartureVelocityKmS = Velocity(R1Km.GetSafeNormal(), Nu1);
    Result.ArrivalVelocityKmS = Velocity(R2Km.GetSafeNormal(), Nu2);
    Result.SemiMajorAxisKm = A;
    Result.Eccentricity = E;
    Result.TransferTime = BestToF;
    Result.DeltaAnomaly = ((Nu2 - Nu1) > 0.0) ? Nu2 - Nu1 : Nu2 - Nu1 + 2.0 * UE_PI;
    Result.bValid = true;
    return Result;
}

#pragma endregion

#pragma region FuelFunctions

double UOrbitMath::RequiredFuelMass(double DryMassKg, double DeltaVKmS, double SpecificImpulseSeconds)
{
    if (DryMassKg <= 0.0 || SpecificImpulseSeconds <= 0.0 || DeltaVKmS <= 0.0)
    {
        return 0.0;
    }

    // Стандартное ускорение переводим из м/с^2 в км/с^2.
    const double StandardGravityKmS2 = SimConstants::StandardGravity / 1000.0;
    const double ExhaustVelocity = SpecificImpulseSeconds * StandardGravityKmS2;

    return DryMassKg * (FMath::Exp(DeltaVKmS / ExhaustVelocity) - 1.0);
}

double UOrbitMath::MaxDeltaV(double DryMassKg, double FuelMassKg, double SpecificImpulseSeconds)
{
    if (DryMassKg <= 0.0 || FuelMassKg <= 0.0 || SpecificImpulseSeconds <= 0.0)
    {
        return 0.0;
    }

    const double StandardGravityKmS2 = SimConstants::StandardGravity / 1000.0;
    const double ExhaustVelocity = SpecificImpulseSeconds * StandardGravityKmS2;

    return ExhaustVelocity * FMath::Loge((DryMassKg + FuelMassKg) / DryMassKg);
}

double UOrbitMath::FuelConsumptionRate(double ThrustNewtons, double SpecificImpulseSeconds)
{
    if (ThrustNewtons <= 0.0 || SpecificImpulseSeconds <= 0.0)
    {
        return 0.0;
    }

    // Тяга в ньютонах, g0 в м/с^2 — расход в кг/с.
    return ThrustNewtons / (SpecificImpulseSeconds * SimConstants::StandardGravity);
}

#pragma endregion

#pragma region UtilityFunctions

double UOrbitMath::NormalizeRange(double Value, double MinValue, double MaxValue)
{
    const double Range = MaxValue - MinValue;
    if (FMath::IsNearlyEqual(Range, 0.0))
    {
        return 0.0;
    }

    return FMath::Clamp((Value - MinValue) / Range, 0.0, 1.0);
}

double UOrbitMath::SafeDivide(double Numerator, double Denominator, double Fallback)
{
    if (FMath::IsNearlyEqual(Denominator, 0.0, 1.0e-30))
    {
        return Fallback;
    }
    return Numerator / Denominator;
}

#pragma endregion

#pragma endregion
