import ctypes
import ctypes.wintypes

# For Python 3.8 and newer, specify the full path to St7API.dll below.
_ST7API = ctypes.windll.LoadLibrary(r'C:\Program Files (x86)\Strand7 R24\Bin64\St7api.dll')

lmMessageBox = 0
lmWaitRetry = 1
lmAbort = 2
kMaxStrLen = 255

# Array Limits
kMaxEntityTotals = 4
kMaxElementNode = 20
kMaxEntity = 10
kMaxBeamResult = 4096
kNumBeamSectionData = 20
kNumMaterialData = 4
kMaxAttributeDoubles = 12
kMaxAttributeLogicals = 6
kMaxAttributeLongint = 6
kLastUnit = 6
kMaxBGLDimensions = 16

# Selection States
ssSelected = 1
ssUnselected = 2

# Unit Positions
ipLENGTHU = 0
ipFORCEU = 1
ipSTRESSU = 2
ipMASSU = 3
ipTEMPERU = 4
ipENERGYU = 5

# Unit Types - LENGTH
luMETRE = 0
luCENTIMETRE = 1
luMILLIMETRE = 2
luFOOT = 3
luINCH = 4

# Unit Types - FORCE
fuNEWTON = 0
fuKILONEWTON = 1
fuMEGANEWTON = 2
fuKILOFORCE = 3
fuPOUNDFORCE = 4
fuTONNEFORCE = 5
fuKIPFORCE = 6

# Unit Types - STRESS
suPASCAL = 0
suKILOPASCAL = 1
suMEGAPASCAL = 2
suKSCm = 3
suPSI = 4
suKSI = 5
suPSF = 6

# Unit Types - MASS
muKILOGRAM = 0
muTONNE = 1
muGRAM = 2
muPOUND = 3
muSLUG = 4

# Unit Types - TEMPERATURE
tuCELSIUS = 0
tuFAHRENHEIT = 1
tuKELVIN = 2

# Unit Types - ENERGY
euJOULE = 0
euBTU = 1
euFTLBF = 2
euCALORIE = 3

# Unit Types - TIME
tuMilliSec = 0
tuSec = 1
tuMin = 2
tuHour = 3
tuDay = 4

# Entity Types
tyNODE = 0
tyBEAM = 1
tyPLATE = 2
tyBRICK = 3
tyLINK = 4
tyVERTEX = 5
tyGEOMETRYEDGE = 6
tyGEOMETRYFACE = 7
tyLOADPATH = 8
tyGEOMETRYCOEDGE = 9
tyGEOMETRYLOOP = 10

# Link Types
ltMasterSlaveLink = 1
ltSectorSymmetryLink = 2
ltCouplingLink = 3
ltPinnedLink = 4
ltRigidLink = 5
ltShrinkLink = 6
ltTwoPointLink = 7
ltAttachmentLink = 8
ltInterpolatedMultiPointLink = 9
ltReactionMultiPointLink = 10
ltRigidMultiPointLink = 11
ltPinnedMultiPointLink = 12
ltMasterSlaveMultiPointLink = 13
ltUserDefinedMultiPointLink = 14

# Master-Slave Link
msFree = 0
msFix = 1
msFixNegate = -1

# Coupling, Attachment and Multi-Point Links
cpTranslational = 1
cpRotational = 2
cpBoth = 3

# Rigid Link
rlPlaneXYZ = 0
rlPlaneXY = 1
rlPlaneYZ = 2
rlPlaneZX = 3

# 2-Point Link
ipTwoPointDOF1 = 0
ipTwoPointDOF2 = 1
ipTwoPointUCS1 = 2
ipTwoPointUCS2 = 3
ipTwoPointFC = 4
ipTwoPointC1 = 0
ipTwoPointC2 = 1
ipTwoPointConst = 2

# Attachment Link
ipAttachmentElType = 0
ipAttachmentElNum = 1
ipAttachmentBrickFaceNum = 2
ipAttachmentCouple = 3

# Node Temperature Types
ntReferenceTemperature = 0
ntFixedTemperature = 1
ntInitialTemperature = 2
ntTableTemperature = 3

# Beam End Release Constants
brReleased = 0
brFixed = 1
brPartial = 2

# Plate Edge Release Constants
prReleased = 0
prFixed = 1

# Property Types
ptBEAMPROP = 1
ptPLATEPROP = 2
ptBRICKPROP = 3
ptPLYPROP = 4

# Property Totals
ipBeamPropTotal = 0
ipPlatePropTotal = 1
ipBrickPropTotal = 2
ipPlyPropTotal = 3

# Alpha Temperature Types
atIntegrated = 0
atInstantaneous = 1

# Sampling Positions
spCentroid = 0
spGaussPoints = 1
spNodesAverageNever = 2
spNodesAverageAll = 3
spNodesAverageSame = 4

# Limit Envelope Averaging
aoAverageThenEnvelope = 0
aoEnvelopeThenAverage = 1

# Beam Types
btNull = 0
btSpring = 1
btCable = 2
btTruss = 3
btCutoff = 4
btContact = 5
btBeam = 6
btUser = 7
btPipe = 8
btConnection = 9

# Contact Types
ctZeroGap = 0
ctNormal = 1
ctTension = 2
ctTakeup = 3

# Takeup Contact Sub Types
tuTension = 0
tuCompression = 1

# Cutoff Bar Types
cbBrittle = 0
cbDuctile = 1

# Contact Parameters Positions - Integers
ipContactType = 0
ipDynamicStiffness = 1
ipUpdateDirection = 2
ipContactSubType = 3
ipFrictionYieldType = 4
ipFrictionModel = 5
ipTensionLateralStiffness = 6

# Contact Parameters Positions - Doubles
ipContactAxialStiffness = 0
ipFrictionC1 = 1
ipFrictionC2 = 2
ipContactMaxTension = 3
ipContactLateralStiffness = 4

# CutoffBar Parameter Positions
ipCutoffType = 0
ipKeepMass = 1

# Library Types
lbMaterial = 0
lbBeamSection = 1
lbComposite = 2
lbReinforcementLayout = 3
lbCreepDefinition = 4
lbLoadPathTemplate = 5
lbSectionGeometry = 6

# Beam Section Types
bsNullSection = 0
bsCircularSolid = 1
bsCircularHollow = 2
bsSquareSolid = 3
bsSquareHollow = 4
bsLipChannel = 5
bsTopHatChannel = 6
bsISection = 7
bsTSection = 8
bsLSection = 9
bsZSection = 10
bsUserSection = 11
bsTrapezoidSolid = 12
bsTrapezoidHollow = 13
bsTriangleSolid = 14
bsTriangleHollow = 15
bsCruciform = 16

# Beam Geometry Section Types
bgNullSection = 0
bgRectangularHollow = 1
bgISection = 2
bgChannel = 3
bgTSection = 4
bgAngle = 5
bgBulbFlat = 6

# Beam Mirror Types
mtNone = 0
mtTop = 1
mtBot = 2
mtLeft = 3
mtRight = 4
mtLeftAndTop = 5
mtLeftAndBot = 6
mtRightAndTop = 7
mtRightAndBot = 8
mtLeftTopOnly = 9
mtLeftBotOnly = 10
mtRightTopOnly = 11
mtRightBotOnly = 12

# Beam Section Positions
ipAREA = 0
ipI11 = 1
ipI22 = 2
ipJ = 3
ipSL1 = 4
ipSL2 = 5
ipSA1 = 6
ipSA2 = 7
ipXBAR = 8
ipYBAR = 9
ipANGLE = 10
ipD1 = 11
ipD2 = 12
ipD3 = 13
ipT1 = 14
ipT2 = 15
ipT3 = 16
ipGapA = 17
ipGapB = 18

# Beam Load Types
dlConstant = 0
dlLinear = 1
dlTriangular = 2
dlThreePoint0 = 3
dlThreePoint1 = 4
dlTrapezoidal = 5

# Plate Load Patch Types
ptAuto4 = 0
ptAuto3 = 1
ptAuto2 = 2
ptAuto1 = 3
ptAngleSplit = 4
ptManual = 5

# Plate Types
ptNull = 0
ptPlaneStress = 1
ptPlaneStrain = 2
ptAxisymmetric = 3
ptPlateShell = 4
ptShearPanel = 5
ptMembrane = 6
ptLoadPatch = 7

# Geometry Surface Types
suNull = -1
suPlane = 0
suSphere = 1
suTorus = 2
suCone = 3
suBSpline = 4
suRotSur = 5
suPipeSur = 6
suSumSur = 7
suTabCyl = 8
suRuleSur = 9
suCubicSpline = 10

# Material Types
mtNull = 0
mtIsotropic = 1
mtOrthotropic = 2
mtAnisotropic = 3
mtRubber = 4
mtSoil = 5
mtLaminate = 6
mtUserDefined = 7
mtFluid = 10

# Yield Criteria - beams
ycBeamFibre = 0
ycBeamTresca = 1
ycBeamVonMises = 2

# Yield Criteria - plates and bricks
ycTresca = 0
ycVonMises = 1
ycMaxStress = 2
ycMohrCoulomb = 3
ycDruckerPrager = 4

# Nonlinear Types
ntNonlinElastic = 0
ntElastoPlastic = 1

# Rubber Types
rtNeoHookean = 1
rtMooneyRivlin = 2
rtGeneralisedMooneyRivlin = 3
rtOgden = 4

# Material Positions
ipModulus = 0
ipPoisson = 1
ipDensity = 2
ipShearModulus = 3

# Node Result Types
rtNodeDisp = 1
rtNodeVel = 2
rtNodeAcc = 3
rtNodePhase = 4
rtNodeReact = 5
rtNodeTemp = 6
rtNodeFlux = 7
rtNodeInertia = 8
rtNodeInfluence = 1

# Beam Result Types
rtBeamForce = 1
rtBeamAllStrain = 2
rtBeamAllStress = 3
rtBeamCableXYZ = 6
rtBeamFlux = 8
rtBeamGradient = 9
rtBeamCreepStrain = 10
rtBeamEnergy = 11
rtBeamDisp = 12
rtBeamNodeReact = 13
rtBeamBirthDisp = 14
rtBeamNodeFlux = 15
rtBeamAxialStress = 16
rtBeamBendingStress = 17
rtBeamFibreStress = 18
rtBeamAvShearStress = 19
rtBeamShearStress = 20
rtBeamCombinedStress = 21
rtPipeHoopStress = 22
rtBeamYieldAreaRatio = 23
rtBeamUser = 24
rtBeamAllTotalStrain = 25
rtBeamExtraResults = 99

# Beam Result Quantities - BEAMFORCE - Principal
ipBeamSF1 = 0
ipBeamBM1 = 1
ipBeamSF2 = 2
ipBeamBM2 = 3

# Beam Result Quantities - BEAMFORCE - Local
ipBeamSFx = 0
ipBeamBMx = 1
ipBeamSFy = 2
ipBeamBMy = 3

# Beam Result Quantities - BEAMFORCE - Local and Principal
ipBeamAxialF = 4
ipBeamTorque = 5

# Beam Result Quantities - BEAMFORCE - Global
ipBeamFX = 0
ipBeamMX = 1
ipBeamFY = 2
ipBeamMY = 3
ipBeamFZ = 4
ipBeamMZ = 5

# Beam Result Quantities - BEAMSTRESS
ipMinFibreStress = 0
ipMaxFibreStress = 1
ipMaxShearStress1 = 2
ipMaxShearStress2 = 3
ipShearF1MeanShearStress = 4
ipShearF2MeanShearStress = 5
ipShearStressMag = 6
ipMinPrincipalStress = 7
ipMaxPrincipalStress = 8
ipMinPipeHoopStress = 9
ipMaxPipeHoopStress = 10
ipMinAxialStress = 11
ipMaxAxialStress = 12
ipMinBendingStress1 = 13
ipMaxBendingStress1 = 14
ipMinBendingStress2 = 15
ipMaxBendingStress2 = 16
ipYieldAreaRatio = 17
ipVonMisesStress = 18
ipTrescaStress = 19
ipTorqueShearStress = 20
ipShearF1ShearStress = 21
ipShearF2ShearStress = 22

# Beam Result Quantities - BEAMSTRAIN
ipAxialStrain = 0
ipCurvature1 = 1
ipCurvature2 = 2
ipTwist = 3
ipMinFibreStrain = 4
ipMaxFibreStrain = 5

# Beam Result Quantities - BEAMCREEPSTRAIN
ipMinFibreCreepStrain = 0
ipMaxFibreCreepStrain = 1
ipMinFibreCreepStrainRate = 2
ipMaxFibreCreepStrainRate = 3
ipShrinkageStrain = 4

# Beam Result Quantities - BEAMRELEASE
ipRelEnd1Dir1 = 0
ipRelEnd1Dir2 = 1
ipRelEnd1Dir3 = 2
ipRelEnd1Dir4 = 3
ipRelEnd1Dir5 = 4
ipRelEnd1Dir6 = 5
ipRelEnd2Dir1 = 6
ipRelEnd2Dir2 = 7
ipRelEnd2Dir3 = 8
ipRelEnd2Dir4 = 9
ipRelEnd2Dir5 = 10
ipRelEnd2Dir6 = 11

# Beam Result Quantities - BEAMENERGY
ipBeamEnergyStored = 0
ipBeamEnergySpent = 1

# Beam Section Result Types
rtBeamSectionStress = 1
rtBeamSectionStrain = 2
rtBeamSectionCreepStrain = 3
rtBeamSectionTotalStrain = 4

# Beam Section Result Quantities
ipFibreStressXY = 0
ipShearStress1XY = 1
ipShearStress2XY = 2
ipMinPrincipalStressXY = 3
ipMaxPrincipalStressXY = 4
ipAxialStressXY = 5
ipBendingStress1XY = 6
ipBendingStress2XY = 7
ipVonMisesStressXY = 8
ipTrescaStressXY = 9
ipTorqueStressXY = 10
ipShearF1ShearStressXY = 11
ipShearF2ShearStressXY = 12

# Plate Result Types
rtPlateStress = 1
rtPlateStrain = 2
rtPlateEnergy = 3
rtPlateForce = 4
rtPlateMoment = 5
rtPlateCurvature = 6
rtPlatePlyStress = 7
rtPlatePlyStrain = 8
rtPlatePlyReserve = 9
rtPlateFlux = 10
rtPlateGradient = 11
rtPlateRCDesign = 12
rtPlateCreepStrain = 13
rtPlateSoil = 14
rtPlateUser = 15
rtPlateNodeReact = 16
rtPlateNodeDisp = 17
rtPlateNodeBirthDisp = 18
rtPlateEffectiveStress = 19
rtPlateEffectiveForce = 20
rtPlateNodeFlux = 21
rtPlateTotalStrain = 22
rtPlateTotalCurvature = 23

# Plate Surface Definition
psPlateMidPlane = 0
psPlateZMinus = 1
psPlateZPlus = 2

# Brick Result Types
rtBrickStress = 1
rtBrickStrain = 2
rtBrickEnergy = 3
rtBrickFlux = 4
rtBrickGradient = 5
rtBrickCreepStrain = 6
rtBrickSoil = 7
rtBrickUser = 8
rtBrickNodeReact = 9
rtBrickNodeDisp = 10
rtBrickNodeBirthDisp = 11
rtBrickEffectiveStress = 12
rtBrickNodeFlux = 13
rtBrickTotalStrain = 14

# Link Result Types
rtLinkNodeDisp = 0
rtLinkNodeReact = 1
rtLinkNodeFlux = 2
rtLinkNodeBirthDisp = 3

# Beam Result Sub Types
stBeamLocal = 0
stBeamPrincipal = -1
stBeamGlobal = -2

# Plate Result Sub Types
stPlateLocal = 0
stPlateGlobal = -1
stPlateCombined = -2
stPlateSupport = -3
stPlateDevLocal = -4
stPlateDevGlobal = -5
stPlateDevCombined = -6

# Brick Result Sub Types
stBrickLocal = 0
stBrickGlobal = -1
stBrickCombined = -2
stBrickSupport = -3
stBrickDevLocal = -4
stBrickDevGlobal = -5
stBrickDevCombined = -6

# Link Result Sub Types
stLinkGlobal = 1

# PLATESTRESS, PLATESTRAIN, PLATECREEPSTRAIN, PLATEMOMENT, PLATECURVATURE, PLATEFORCE results for STLOCAL
ipPlateLocalxx = 0
ipPlateLocalyy = 1
ipPlateLocalzz = 2
ipPlateLocalxy = 3
ipPlateLocalyz = 4
ipPlateLocalzx = 5
ipPlateLocalxz = 5
ipPlateLocalMean = 0
ipPlateLocalDevxx = 1
ipPlateLocalDevyy = 2
ipPlateEdgeSupport = 0
ipPlateFaceSupport = 1

# PLATESTRESS, PLATESTRAIN, PLATECREEPSTRAIN, PLATEMOMENT, PLATECURVATURE, PLATEFORCE results for STGLOBAL (NOT AXISYMMETRIC)
ipPlateGlobalXX = 0
ipPlateGlobalYY = 1
ipPlateGlobalZZ = 2
ipPlateGlobalXY = 3
ipPlateGlobalYZ = 4
ipPlateGlobalZX = 5
ipPlateGlobalMean = 0
ipPlateGlobalDevXX = 1
ipPlateGlobalDevYY = 2
ipPlateGlobalDevZZ = 3

# PLATESTRESS, PLATESTRAIN, PLATECREEPSTRAIN, PLATEMOMENT, PLATECURVATURE, PLATEFORCE results for STUCS
ipPlateUCSXX = 0
ipPlateUCSYY = 1
ipPlateUCSZZ = 2
ipPlateUCSXY = 3
ipPlateUCSYZ = 4
ipPlateUCSZX = 5

# PLATESTRESS, PLATESTRAIN, PLATECREEPSTRAIN, PLATEFORCE, PLATEMOMENT, PLATECURVATURE results for STCOMBINED (NOT AXISYMMETRIC)
ipPlateCombPrincipal11 = 0
ipPlateCombPrincipal22 = 1
ipPlateCombPrincipalAngle = 3
ipPlateCombVonMises = 4
ipPlateCombTresca = 5
ipPlateCombMohrCoulomb = 6
ipPlateCombDruckerPrager = 7
ipPlateCombMagnitude = 9
ipPlateCombPlasticStrain = 6
ipPlateCombCreepEffRate = 6
ipPlateCombCreepShrinkage = 7
ipPlateCombYieldIndex = 8
ipPlateCombMean = 0
ipPlateCombDev11 = 1
ipPlateCombDev22 = 2

# PLATESTRESS, PLATESTRAIN, PLATECREEPSTRAIN results for STGLOBAL (AXISYMMETRIC)
ipPlateAxiGlobalRR = 0
ipPlateAxiGlobalZZ = 1
ipPlateAxiGlobalTT = 2
ipPlateAxiGlobalRZ = 3
ipPlateAxiGlobalMean = 0
ipPlateAxiGlobalDevRR = 1
ipPlateAxiGlobalDevZZ = 2
ipPlateAxiGlobalDevTT = 3

# PLATESTRESS, PLATESTRAIN, PLATECREEPSTRAIN results for STCOMBINED (AXISYMMETRIC)
ipPlateAxiCombPrincipal11 = 0
ipPlateAxiCombPrincipal22 = 1
ipPlateAxiCombPrincipal33 = 2
ipPlateAxiCombVonMises = 4
ipPlateAxiCombTresca = 5
ipPlateAxiCombMohrCoulomb = 6
ipPlateAxiCombDruckerPrager = 7
ipPlateAxiCombMagnitude = 9
ipPlateAxiCombPlasticStrain = 6
ipPlateAxiCombCreepEffRate = 6
ipPlateAxiCombCreepShrinkage = 7
ipPlateAxiCombYieldIndex = 8
ipPlateAxiCombMean = 0
ipPlateAxiCombDev11 = 1
ipPlateAxiCombDev22 = 2
ipPlateAxiCombDev33 = 3

# PLATEPLYSTRESS
ipPlyStress11 = 0
ipPlyStress22 = 1
ipPlyStress12 = 3
ipPlyILSx = 4
ipPlyILSy = 5

# PLATEPLYSTRAIN
ipPlyStrain11 = 0
ipPlyStrain22 = 1
ipPlyStrain12 = 3

# PLATEPLYRESERVE
ipPlyMaxStress = 0
ipPlyMaxStrain = 1
ipPlyTsaiHill = 2
ipPlyModTsaiWu = 3
ipPlyHoffman = 4
ipPlyInterlam = 5

# PLATESOIL
ipPlateSoilTotalPorePressure = 0
ipPlateSoilExcessPorePressure = 1
ipPlateSoilOCRIndex = 2
ipPlateSoilStateIndex = 3
ipPlateSoilVoidRatio = 4

# PLATEFLUX, PLATEGRADIENT results for STLOCAL
ipPlateFluxLocalx = 0
ipPlateFluxLocaly = 1
ipPlateFluxLocalMagxy = 2

# PLATEFLUX, PLATEGRADIENT results for STGLOBAL
ipPlateFluxGlobalX = 0
ipPlateFluxGlobalY = 1
ipPlateFluxGlobalZ = 2
ipPlateFluxGlobalMagXY = 3
ipPlateFluxGlobalMagYZ = 4
ipPlateFluxGlobalMagZX = 5
ipPlateFluxGlobalMagXYZ = 6

# PLATEFLUX, PLATEGRADIENT results for STUCS
ipPlateFluxUCSX = 0
ipPlateFluxUCSY = 1
ipPlateFluxUCSZ = 2
ipPlateFluxUCSMagXY = 3
ipPlateFluxUCSMagYZ = 4
ipPlateFluxUCSMagZX = 5
ipPlateFluxUCSMagXYZ = 6

# PLATERCDESIGN
ipPlateRCWoodArmerMoment = 0
ipPlateRCWoodArmerForce = 1
ipPlateRCSteelArea = 2
ipPlateRCConcreteStrain = 3
ipPlateRCSteelAreaLessBase = 4
ipPlateRCUserSteelStress = 5
ipPlateRCUserConcreteStrain = 6
ipPlateRCBlockRatio = 7

# PLATEENERGY
ipPlateEnergyStored = 0
ipPlateEnergySpent = 1

# BRICKSTRESS, BRICKSTRAIN, BRICKCREEPSTRAIN results for STLOCAL
ipBrickLocalxx = 0
ipBrickLocalyy = 1
ipBrickLocalzz = 2
ipBrickLocalxy = 3
ipBrickLocalyz = 4
ipBrickLocalzx = 5
ipBrickLocalMean = 0
ipBrickLocalDevxx = 1
ipBrickLocalDevyy = 2
ipBrickLocalDevzz = 3
ipBrickFaceSupport = 0

# BRICKSTRESS, BRICKSTRAIN, BRICKCREEPSTRAIN results for STGLOBAL
ipBrickGlobalXX = 0
ipBrickGlobalYY = 1
ipBrickGlobalZZ = 2
ipBrickGlobalXY = 3
ipBrickGlobalYZ = 4
ipBrickGlobalZX = 5
ipBrickGlobalMean = 0
ipBrickGlobalDevXX = 1
ipBrickGlobalDevYY = 2
ipBrickGlobalDevZZ = 3

# BRICKSTRESS, BRICKSTRAIN, BRICKCREEPSTRAIN results for STUCS
ipBrickUCSXX = 0
ipBrickUCSYY = 1
ipBrickUCSZZ = 2
ipBrickUCSXY = 3
ipBrickUCSYZ = 4
ipBrickUCSZX = 5

# BRICKSTRESS, BRICKSTRAIN, BRICKCREEPSTRAIN results for STCOMBINED
ipBrickCombPrincipal11 = 0
ipBrickCombPrincipal22 = 1
ipBrickCombPrincipal33 = 2
ipBrickCombVonMises = 3
ipBrickCombTresca = 4
ipBrickCombMohrCoulomb = 5
ipBrickCombDruckerPrager = 6
ipBrickCombPlasticStrain = 6
ipBrickCombCreepEffRate = 6
ipBrickCombCreepShrinkage = 7
ipBrickCombMean = 7
ipBrickCombYieldIndex = 8
ipBrickCombMagnitude = 9
ipBrickCombDevMean = 0
ipBrickCombDev11 = 1
ipBrickCombDev22 = 2
ipBrickCombDev33 = 3

# BRICKSOIL
ipBrickSoilTotalPorePressure = 0
ipBrickSoilExcessPorePressure = 1
ipBrickSoilOCRIndex = 2
ipBrickSoilStateIndex = 3
ipBrickSoilVoidRatio = 4

# BRICKFLUX, BRICKGRADIENT results for STLOCAL
ipBrickFluxLocalx = 0
ipBrickFluxLocaly = 1
ipBrickFluxLocalz = 2
ipBrickFluxLocalMagxy = 3
ipBrickFluxLocalMagyz = 4
ipBrickFluxLocalMagzx = 5
ipBrickFluxLocalMagxyz = 6

# BRICKFLUX, BRICKGRADIENT results for STGLOBAL
ipBrickFluxGlobalX = 0
ipBrickFluxGlobalY = 1
ipBrickFluxGlobalZ = 2
ipBrickFluxGlobalMagXY = 3
ipBrickFluxGlobalMagYZ = 4
ipBrickFluxGlobalMagZX = 5
ipBrickFluxGlobalMagXYZ = 6

# BRICKFLUX, BRICKGRADIENT results for STUCS
ipBrickFluxUCSX = 0
ipBrickFluxUCSY = 1
ipBrickFluxUCSZ = 2
ipBrickFluxUCSMagXY = 3
ipBrickFluxUCSMagYZ = 4
ipBrickFluxUCSMagZX = 5
ipBrickFluxUCSMagXYZ = 6

# BRICKENERGY
ipBrickEnergyStored = 0
ipBrickEnergySpent = 1

# MODAL RESULTS NFA
ipFrequencyNFA = 0
ipModalMassNFA = 1
ipModalStiffNFA = 2
ipModalDampNFA = 3
ipModalTMassP1 = 4
ipModalTMassP2 = 5
ipModalTMassP3 = 6
ipModalRMassP1 = 7
ipModalRMassP2 = 8
ipModalRMassP3 = 9

# MODAL RESULTS HRA
ipFrequencyHRA = 0
ipDampRatioHRA = 1
ipAmplitudeHRA = 2
ipPhaseAngleHRA = 3
ipMassPartHRA = 4

# MODAL RESULTS SRA
ipFrequencySRA = 0
ipSpectralValueSRA = 1
ipDampRatioSRA = 2
ipExcitationSRA = 3
ipAmplitudeSRA = 4
ipMassPartSRA = 5

# INERTIA RELIEF RESULTS
ipMassXIRA = 0
ipMassYIRA = 1
ipMassZIRA = 2
ipXcIRA = 3
ipYcIRA = 4
ipZcIRA = 5
ipAccXIRA = 6
ipAccYIRA = 7
ipAccZIRA = 8
ipAngAccXIRA = 9
ipAngAccYIRA = 10
ipAngAccZIRA = 11

# CONTOUR FILE EXTRAPOLATION
eoCentroid = 0
eoNode = 1
eoGaussPoint = 2

# CONTOUR FILE AVERAGING
aoAlways = 0
aoNever = 1
aoSameProp = 2
aoJumps = 3
aoJumpsN = 4
aoRange = 5
aoSamePropAndStage = 6

# CONTOUR FILE INDEXES
ipQuantityRF = 0
ipSystemRF = 1
ipComponentRF = 2
ipLayerRF = 3
ipExtrapolateRF = 4
ipAverageRF = 5
ipAbsoluteRF = 6

# Coordinate System Types
csCartesian = 0
csCylindrical = 1
csSpherical = 2
csToroidal = 3

# Matrix Types
mtCompliance = 1
mtStiffness = 2

# Vertex Types
vtFree = 1
vtFixed = 2

# Beam Distributed Load Projection Options
bpNone = 0
bpProjected = 1

# Edge Types
etInterpolated = 0
etNonInterpolated = 1

# Edge Cluster Origin Types
coAutoClusterOrigin = 0
coManualClusterOrigin = 1

# Plate/Face Global Pressure Projection Options
ppNone = 0
ppProjResultant = 1
ppProjComponents = 2

# Node/Vertex Attribute Types
aoRestraint = 1
aoForce = 2
aoMoment = 3
aoTemperature = 4
aoMTranslation = 5
aoMRotation = 6
aoKTranslation = 7
aoKRotation = 8
aoDamping = 9
aoNSMass = 10
aoNodeInfluence = 11
aoNodeHeatSource = 12
aoNodeVelocity = 13
aoNodeAcceleration = 14
aoVertexMeshSize = 20

# Beam Attribute Types
aoBeamAngle = 21
aoBeamOffset = 22
aoBeamTEndRelease = 23
aoBeamREndRelease = 24
aoBeamSupport = 25
aoBeamPreTension = 26
aoCableFreeLength = 27
aoBeamDLL = 28
aoBeamDLG = 29
aoBeamCFL = 30
aoBeamCFG = 31
aoBeamCML = 32
aoBeamCMG = 33
aoBeamTempGradient = 34
aoBeamConvection = 35
aoBeamRadiation = 36
aoBeamFlux = 37
aoBeamHeatSource = 38
aoBeamRadius = 39
aoPipePressure = 40
aoBeamNSMass = 41
aoPipeTemperature = 42
aoBeamDML = 44
aoBeamStringGroup = 45
aoBeamPreCurvature = 46
aoBeamTaper = 92
aoBeamInfluence = 93
aoBeamSectionFactor = 94
aoBeamCreepLoadingAge = 95
aoBeamEndAttachment = 96
aoBeamConnectionUCS = 97
aoBeamStageProperty = 98
aoBeamSideAttachment = 120

# Plate/Edge/Face Attribute Types
aoPlateAngle = 51
aoPlateOffset = 52
aoPlatePreLoad = 53
aoPlateFacePressure = 54
aoPlateFaceShear = 55
aoPlateEdgeNormalPressure = 56
aoPlateEdgeShear = 57
aoPlateEdgeTransverseShear = 58
aoPlateTempGradient = 59
aoPlateEdgeSupport = 60
aoPlateFaceSupport = 61
aoPlateEdgeConvection = 62
aoPlateEdgeRadiation = 63
aoPlateFlux = 64
aoPlateHeatSource = 65
aoPlateGlobalPressure = 66
aoPlateEdgeRelease = 67
aoPlateReinforcement = 68
aoPlateThickness = 69
aoPlateNSMass = 70
aoLoadPatch = 71
aoPlateEdgeGlobalPressure = 72
aoPlatePreCurvature = 73
aoPlatePointForce = 99
aoPlatePointMoment = 100
aoPlateFaceConvection = 101
aoPlateFaceRadiation = 102
aoPlateInfluence = 103
aoPlateSoilStress = 104
aoPlateSoilRatio = 105
aoPlateCreepLoadingAge = 106
aoPlateEdgeAttachment = 107
aoPlateFaceAttachment = 108
aoPlateStageProperty = 109

# Brick Attribute Types
aoBrickPressure = 81
aoBrickShear = 82
aoBrickFaceFoundation = 83
aoBrickConvection = 84
aoBrickRadiation = 85
aoBrickFlux = 86
aoBrickHeatSource = 87
aoBrickGlobalPressure = 88
aoBrickNSMass = 89
aoBrickLocalAxes = 90
aoBrickPreLoad = 91
aoBrickPointForce = 110
aoBrickInfluence = 111
aoBrickSoilStress = 112
aoBrickSoilRatio = 113
aoBrickCreepLoadingAge = 114
aoBrickFaceAttachment = 115
aoBrickStageProperty = 116

# Path Attribute Types
aoPathPointForce = 117
aoPathDistributedForce = 118
aoPathHeatSource = 119

# Attribute Deletion and AttributeSequence Indexes
ipAttrLocal = 0
ipAttrAxis = 1
ipAttrCase = 2
ipAttrID = 3

# Marker Types
mtCircleMarker = 0
mtSquareMarker = 1
mtTriangleMarker = 2
mtRectangleMarker = 3
mtEntityHighlight = 4
mtBanner = 5

# Marker Styles
msFilled = 0
msOutlined = 1
msFilledOutlined = 2

# Marker Definition Integers Indexes
ipMarkerType = 0
ipMarkerStyle = 1
ipMarkerFillColour = 2
ipMarkerLineColour = 3
ipMarkerLineThickness = 4
ipMarkerSize = 5
ipMarkerHeight = 6
ipMarkerAnchorX = 7
ipMarkerAnchorY = 8
ipMarkerVisible = 9
ipMarkerNumber = 10
ipMarkerLabelled = 11

# Title Block
tbTitle = 0
tbProject = 1
tbReference = 2
tbAuthor = 3
tbCreated = 4
tbModified = 5

# Table Types
ttVsTime = 1
ttVsTemperature = 2
ttVsFrequency = 3
ttStressStrain = 4
ttForceDisplacement = 5
ttMomentCurvature = 6
ttMomentRotation = 8
ttAccVsTime = 9
ttForceVelocity = 10
ttVsPosition = 11
ttStrainTime = 12
ttDispVsTime = 13
ttVelVsTime = 14

# Acceleration Time Table Types
atModelUnits = 0
atGravityUnits = 1

# Frequency Table Types
ftPeriod = 0
ftFrequency = 1

# Beam Prop Table Entries
ptBeamStiffModVsTemp = 1001
ptBeamAlphaVsTemp = 1002
ptBeamConductVsTemp = 1003
ptBeamCpVsTemp = 1004
ptBeamStiffModVsTime = 1005
ptBeamConductVsTime = 1006
ptSpringAxialVsDisp = 1007
ptSpringTorqueVsTwist = 1008
ptSpringAxialVsVelocity = 1009
ptBeamStressVsStrain = 1011
ptBeamMomentK1 = 1012
ptBeamMomentK2 = 1013
ptConnectionShear1 = 1014
ptConnectionShear2 = 1015
ptConnectionAxial = 1016
ptConnectionBend1 = 1017
ptConnectionBend2 = 1018
ptConnectionTorque = 1019
ptBeamYieldVsTemp = 1020

# Plate Prop Table Entries
ptPlateModVsTemp = 2001
ptPlateAlphaVsTemp = 2002
ptPlateConductVsTemp = 2003
ptPlateCpVsTemp = 2004
ptPlateModVsTime = 2005
ptPlateConductVsTime = 2006
ptPlateStressVsStrain = 2007
ptPlateYieldVsTemp = 2008

# Brick Prop Table Entries
ptBrickModVsTemp = 3001
ptBrickAlphaVsTemp = 3002
ptBrickConductVsTemp = 3003
ptBrickCpVsTemp = 3004
ptBrickModVsTime = 3005
ptBrickConductVsTime = 3006
ptBrickStressVsStrain = 3007
ptBrickYieldVsTemp = 3008

# Creep Laws
clConcreteHyperbolic = 0
clConcreteViscoChain = 1
clConcreteUserDefined = 2
clPrimaryPower = 3
clSecondaryPower = 4
clPrimarySecondaryPower = 5
clSecondaryHyperbolic = 6
clSecondaryExponential = 7
clThetaProjection = 8
clGenGraham = 9
clGenBlackburn = 10
clUserDefined = 11

# Load Case Types
lcNoInertia = 0
lcGravity = 1
lcAccelerations = 2
lcSeismic = 3

# Freedom Case Types
fcNormalFreedom = 0
fcFreeBodyInertiaRelief = 1
fcSingleSymmetryInertiaXY = 2
fcSingleSymmetryInertiaYZ = 3
fcSingleSymmetryInertiaZX = 4
fcDoubleSymmetryInertiaX = 5
fcDoubleSymmetryInertiaY = 6
fcDoubleSymmetryInertiaZ = 7

# Linear Combination Options
kNoCombinations = 0
kGenerateNewCombinations = 1
kUseExistingCombinations = 2

# Influence Case Types
icInfluenceMin = 0
icInfluenceMax = 1

# Influence Combination Options
ipInfCaseLabel = 0
ipInfCaseVariable = 1
ipInfCaseLoadCase = 2
ipInfCaseFreedomCase = 3
ipInfCaseResponseType = 4

# Influence Warning Codes
wcInfluenceNoWarning = 0
wcInfluenceUserTerminated = 1
wcInfluenceRanOutOfAttributeID = 2

# Harmonic Combination Warning Codes
wcHarmonicCombineNoWarning = 0
wcHarmonicCombineInvalidLSA = 1

# Global Load Case
ipLoadCaseRefTemp = 0
ipLoadCaseOrigX = 1
ipLoadCaseOrigY = 2
ipLoadCaseOrigZ = 3
ipLoadCaseAccX = 4
ipLoadCaseAccY = 5
ipLoadCaseAccZ = 6
ipLoadCaseAngVelX = 7
ipLoadCaseAngVelY = 8
ipLoadCaseAngVelZ = 9
ipLoadCaseAngAccX = 10
ipLoadCaseAngAccY = 11
ipLoadCaseAngAccZ = 12

# Global Seismic Load Case
ipSeismicCaseAlpha = 0
ipSeismicCasePhi = 1
ipSeismicCaseBeta = 2
ipSeismicCaseK = 3
ipSeismicCaseh0 = 4
ipSeismicCaseDir = 5
ipSeismicCaseLinAcc = 6
ipSeismicCaseV1 = 7
ipSeismicCaseV2 = 8

# Damping Types
dtNoDamping = 0
dtRayleighDamping = 1
dtModalDamping = 2
dtViscousDamping = 3

# Rayleigh Modes
rmSetFrequencies = 0
rmSetAlphaBeta = 1

# Rayleigh Damping Factors
ipRayleighF1 = 0
ipRayleighF2 = 1
ipRayleighR1 = 2
ipRayleighR2 = 3
ipRayleighAlpha = 0
ipRayleighBeta = 1
ipRayleighDisplayF1 = 4
ipRayleighDisplayF2 = 5

# Entity Solver Result Types - HEAT
hrNodeFlux = 1
hrBeamFlux = 2
hrPlateFlux = 3
hrBrickFlux = 4
hrLinkFlux = 22

# Entity Solver Result Types - FREQUENCY
frBeamForcePattern = 5
frBeamStrainPattern = 6
frPlateStressPattern = 7
frPlateStrainPattern = 8
frBrickStressPattern = 9
frBrickStrainPattern = 10

# Entity Solver Result Types - STRUCTURAL
srNodeReaction = 11
srNodeVelocity = 12
srNodeAcceleration = 13
srBeamForce = 14
srBeamMNLStress = 15
srBeamStrain = 16
srPlateStress = 17
srPlateStrain = 18
srBrickStress = 19
srBrickStrain = 20
srElementNodeForce = 21
srLinkForce = 23
srNodeInertia = 24

# Solver Defaults - LOGICALS
spDoSturm = 1
spNonlinearMaterial = 2
spNonlinearGeometry = 4
spAddKg = 6
spCalcDampingRatios = 8
spIncludeLinkReactions = 9
spFullSystemTransient = 10
spNonlinearHeat = 11
spLumpedLoadBeam = 12
spLumpedLoadPlate = 13
spLumpedMassBeam = 15
spLumpedMassPlate = 16
spLumpedMassBrick = 17
spForceDrillCheck = 18
spSaveRestartFile = 20
spSaveIntermediate = 21
spExcludeMassX = 22
spExcludeMassY = 23
spExcludeMassZ = 24
spSaveSRSSSpectral = 25
spSaveCQCSpectral = 26
spDoResidualsCheck = 27
spSuppressAllSingularities = 28
spReducedLogFile = 31
spIncludeRotationalMass = 32
spIgnoreCompressiveBeamKg = 33
spAutoScaleKg = 34
spScaleSupports = 36
spAutoShift = 37
spSaveTableInsertedSteps = 38
spSaveLastRestartStep = 39
spDoInstantNTA = 41
spAllowExtraIterations = 42
spPredictImpact = 43
spAutoWorkingSet = 44
spDampingForce = 45
spLimitDisplacementNLA = 46
spLimitRotationNLA = 47
spSaveFinalSubStep = 48
spCablesAsMultiCase = 49
spShowMessages = 50
spShowProgress = 51
spShowConvergenceGraph = 52
spTimeScaleAbsoluteTemperature = 53
spSpectralBaseExcitation = 54
spSpectralLoadExcitation = 55
spCheckEigenvector = 57
spAppendRemainingTime = 58
spIncludeFollowerLoadKG = 59
spInertiaForce = 60
spSolverGeneratesCombinations = 61

# Solver Defaults - INTEGERS
spTreeStartNumber = 1
spNumFrequency = 2
spNumBucklingModes = 3
spMaxIterationEig = 4
spMaxIterationNonlin = 5
spNumBeamSlicesSpectral = 6
spMaxConjugateGradientIter = 7
spMaxNumWarnings = 8
spFiniteStrainDefinition = 9
spBeamLength = 10
spFormStiffMatrix = 11
spMaxUpdateInterval = 12
spFormNonlinHeatStiffMatrix = 13
spExpandWorkingSet = 14
spMinNumViscoUnits = 15
spMaxNumViscoUnits = 16
spCurveFitTimeUnit = 17
spStaticAutoStepping = 18
spBeamKgType = 19
spDynamicAutoStepping = 20
spMaxIterationHeat = 21

# Solver Defaults - DOUBLES
spEigenTolerance = 1
spFrequencyShift = 2
spBucklingShift = 3
spNonlinDispTolerance = 4
spNonlinResidualTolerance = 5
spTransientReferenceTemperature = 6
spRelaxationFactor = 7
spNonlinHeatTolerance = 8
spMinimumTimeStep = 9
spWilsonTheta = 10
spNewmarkBeta = 11
spGlobalZeroDiagonal = 12
spConjugateGradientTol = 13
spMinimumDimension = 14
spMinimumInternalAngle = 15
spZeroForce = 16
spZeroDiagonal = 17
spZeroContactFactor = 18
spFrictionCutoffStrain = 19
spZeroTranslation = 20
spZeroRotation = 21
spDrillStiffFactor = 22
spMaxNormalsAngle = 24
spMaximumRotation = 26
spZeroDisplacement = 27
spMaximumDispRatio = 28
spMinimumLoadReductionFactor = 29
spMaxDispChange = 30
spMaxResidualChange = 31
spZeroFrequency = 32
spZeroBucklingEigen = 33
spCurveFitTime = 34
spSpacingBias = 35
spTimeStepParam = 36
spMNLTangentRatio = 38
spMinArcLengthFactor = 40
spMaxFibreStrainInc = 41
spMaxDisplacementNLA = 42
spMaxRotationNLA = 43
spClusterZeroDiagonal = 44
spUpdateDirContactCheckPoint = 45
spFrictionModulusRatio = 46

# Spectral Base Load Types
slBaseAcc = 0
slBaseVel = 1
slBaseDisp = 2

# Harmonic Load Types
hlBaseAcc = 0
hlBaseVel = 1
hlBaseDisp = 2
hlAppliedLoad = 3

# Transient Base Excitation Types
beNone = 0
beAcceleration = 1
beVelocity = 2
beDisplacement = 3

# Harmonic Modes
hmVsFrequency = 0
hmVsTime = 1

# Solver Matrix Schemes
stSkyline = 0
stSparse = 1
stIterativePCG = 3

# Solver Temperature Dependence Types
tdNone = 0
tdCombined = 1

# Result File Open Indexes
ipHideUnconvergedLBA = 1
ipHideNegativeLBA = 2
ipHideUnconvergedNFA = 3
ipHideZeroNFA = 4
ipHideModalSRA = 5
ipHideUnconvergedNLA = 6
ipHideSubStepNLA = 7
ipHideUnconvergedNTA = 8
ipHideSubStepNTA = 9
ipHideUnconvergedQSA = 10
ipHideSubStepQSA = 11

# Sort Types
rnNone = 0
rnTree = 1
rnGeometry = 2
rnAMD = 3

# Utility
ztAbsolute = 0
ztRelative = 1

# Boolean Types
btFalse = 0
btTrue = 1

# Error Codes
ERR7_APIAlreadyInitialised = -12
ERR7_LoginExceeded = -11
ERR7_CannotCommunicate = -10
ERR7_CannotFindNetworkLock = -9
ERR7_CannotFindStandaloneLock = -8
ERR7_CannotInitialiseDirectX = -7
ERR7_InvalidRegionalSettings = -6
ERR7_InvalidDLLsPresent = -5
ERR7_APINotInitialised = -4
ERR7_InvalidErrorCode = -3
ERR7_APINotLicensed = -2
ERR7_UnknownError = -1
ERR7_NoError = 0
ERR7_FileAlreadyOpen = 1
ERR7_FileNotFound = 2
ERR7_FileNotSt7 = 3
ERR7_InvalidFileName = 4
ERR7_FileIsNewer = 5
ERR7_CannotReadFile = 6
ERR7_InvalidScratchPath = 7
ERR7_FileNotOpen = 8
ERR7_ExceededTotal = 9
ERR7_DataNotFound = 10
ERR7_InvalidResultFile = 11
ERR7_ResultFileNotOpen = 12
ERR7_ExceededResultCase = 13
ERR7_UnknownResultType = 14
ERR7_UnknownResultLocation = 15
ERR7_UnknownSurfaceLocation = 16
ERR7_UnknownProperty = 17
ERR7_InvalidEntity = 18
ERR7_InvalidBeamPosition = 19
ERR7_InvalidLoadCase = 20
ERR7_InvalidFreedomCase = 21
ERR7_UnknownTitle = 22
ERR7_InvalidResOptsNFADisp = 23
ERR7_TooManyBeamStations = 24
ERR7_UnknownSubType = 25
ERR7_GroupIdDoesNotExist = 26
ERR7_InvalidFileUnit = 27
ERR7_CannotSaveFile = 28
ERR7_ResultFileIsOpen = 29
ERR7_InvalidUnits = 30
ERR7_InvalidEntityNodes = 31
ERR7_InvalidUCSType = 32
ERR7_InvalidUCSID = 33
ERR7_UCSIDAlreadyExists = 34
ERR7_CaseNameAlreadyExists = 35
ERR7_InvalidEntityNumber = 36
ERR7_InvalidBeamEnd = 37
ERR7_InvalidBeamDir = 38
ERR7_InvalidPlateEdge = 39
ERR7_InvalidBrickFace = 40
ERR7_InvalidBeamType = 41
ERR7_InvalidPlateType = 42
ERR7_InvalidMaterialType = 43
ERR7_PropertyAlreadyExists = 44
ERR7_InvalidBeamSectionType = 45
ERR7_PropertyNotSpring = 46
ERR7_PropertyNotCable = 47
ERR7_PropertyNotTruss = 48
ERR7_PropertyNotCutOffBar = 49
ERR7_PropertyNotPointContact = 50
ERR7_PropertyNotBeam = 51
ERR7_PropertyNotPipe = 52
ERR7_PropertyNotConnectionBeam = 53
ERR7_InvalidSectionParameters = 54
ERR7_PropertyNotUserDefinedBeam = 55
ERR7_MaterialIsUserDefined = 56
ERR7_MaterialNotIsotropic = 57
ERR7_MaterialNotOrthotropic = 58
ERR7_InvalidRubberModel = 59
ERR7_MaterialNotRubber = 60
ERR7_InvalidSectionProperties = 61
ERR7_PlateDoesNotHaveThickness = 62
ERR7_IncompatibleMaterialCombination = 63
ERR7_UnknownSolver = 64
ERR7_InvalidSolverMode = 65
ERR7_InvalidMirrorOption = 66
ERR7_SectionCannotBeMirrored = 67
ERR7_InvalidTableType = 68
ERR7_InvalidTableName = 69
ERR7_TableNameAlreadyExists = 70
ERR7_InvalidNumberOfEntries = 71
ERR7_InvalidToleranceType = 72
ERR7_TableDoesNotExist = 73
ERR7_NotFrequencyTable = 74
ERR7_InvalidFrequencyType = 75
ERR7_InvalidTableSetting = 76
ERR7_IncompatibleTableType = 77
ERR7_IncompatibleCriterionCombination = 78
ERR7_InvalidModalFile = 79
ERR7_InvalidCombinationCaseNumber = 80
ERR7_InvalidInitialCaseNumber = 81
ERR7_InvalidInitialFile = 82
ERR7_InvalidModeNumber = 83
ERR7_BeamIsNotBXS = 84
ERR7_InvalidDampingType = 85
ERR7_InvalidRayleighMode = 86
ERR7_CannotReadBXS = 87
ERR7_InvalidResultType = 88
ERR7_InvalidSolverParameter = 89
ERR7_InvalidModalLoadType = 90
ERR7_InvalidTimeRow = 91
ERR7_SparseSolverNotLicensed = 92
ERR7_InvalidSolverScheme = 93
ERR7_InvalidSortOption = 94
ERR7_IncompatibleResultFile = 95
ERR7_InvalidLinkType = 96
ERR7_InvalidLinkData = 97
ERR7_OnlyOneLoadCase = 98
ERR7_OnlyOneFreedomCase = 99
ERR7_InvalidLoadID = 100
ERR7_InvalidBeamLoadType = 101
ERR7_InvalidStringID = 102
ERR7_InvalidPatchType = 103
ERR7_IncrementDoesNotExist = 104
ERR7_InvalidLoadCaseType = 105
ERR7_InvalidFreedomCaseType = 106
ERR7_InvalidHarmonicLoadType = 107
ERR7_InvalidTemperatureType = 108
ERR7_InvalidPatchTypeForPlate = 109
ERR7_InvalidAttributeType = 110
ERR7_MaterialNotAnisotropic = 111
ERR7_InvalidMatrixType = 112
ERR7_MaterialNotUserDefined = 113
ERR7_InvalidIndex = 114
ERR7_InvalidContactType = 115
ERR7_InvalidContactSubType = 116
ERR7_InvalidCutoffType = 117
ERR7_ResultQuantityNotAvailable = 118
ERR7_YieldNotMCDP = 119
ERR7_CombinationDoesNotExist = 120
ERR7_InvalidSeismicCase = 121
ERR7_InvalidImportExportMode = 122
ERR7_CannotReadImportFile = 123
ERR7_InvalidAnsysImportFormat = 124
ERR7_InvalidAnsysArrayStatus = 125
ERR7_CannotWriteExportFile = 126
ERR7_InvalidAnsysExportFormat = 127
ERR7_InvalidAnsysEndReleaseOption = 128
ERR7_InvalidAnsysExportUnits = 129
ERR7_InvalidSt7ExportFormat = 130
ERR7_InvalidUVPos = 131
ERR7_InvalidResponseType = 132
ERR7_InvalidLayoutID = 133
ERR7_InvalidPlateSurface = 134
ERR7_MeshingErrors = 135
ERR7_InvalidTolerance = 136
ERR7_InvalidTaperAxis = 137
ERR7_InvalidTaperType = 138
ERR7_InvalidTaperRatio = 139
ERR7_InvalidPositionType = 140
ERR7_InvalidPreLoadType = 141
ERR7_InvalidVertexType = 142
ERR7_InvalidVertexMeshSize = 143
ERR7_InvalidGeometryEdgeType = 144
ERR7_InvalidPropertyNumber = 145
ERR7_InvalidFaceSurface = 146
ERR7_InvalidModType = 147
ERR7_MaterialNotSoil = 148
ERR7_MaterialNotFluid = 149
ERR7_SoilTypeNotDC = 150
ERR7_SoilTypeNotCC = 151
ERR7_MaterialNotLaminate = 152
ERR7_InvalidLaminateID = 153
ERR7_LaminateNameAlreadyExists = 154
ERR7_LaminateIDAlreadyExists = 155
ERR7_PlyDoesNotExist = 156
ERR7_ExceededMaxNumPlies = 157
ERR7_LayoutIDAlreadyExists = 158
ERR7_InvalidNumModes = 159
ERR7_InvalidLTAMethod = 160
ERR7_InvalidLTASolutionType = 161
ERR7_ExceededMaxNumStages = 162
ERR7_StageDoesNotExist = 163
ERR7_ExceededMaxNumSpectralCases = 164
ERR7_InvalidSpectralCase = 165
ERR7_InvalidSpectrumType = 166
ERR7_InvalidResultsSign = 167
ERR7_InvalidPositionTableAxis = 168
ERR7_InvalidInitialConditionsType = 169
ERR7_ExceededMaxNumNodeHistory = 170
ERR7_NodeHistoryDoesNotExist = 171
ERR7_InvalidTransientTempType = 172
ERR7_InvalidTimeUnit = 173
ERR7_InvalidLoadPath = 174
ERR7_InvalidTempDependenceType = 175
ERR7_InvalidTrigType = 176
ERR7_InvalidUserEquation = 177
ERR7_InvalidCreepID = 178
ERR7_CreepIDAlreadyExists = 179
ERR7_InvalidCreepLaw = 180
ERR7_InvalidCreepHardeningLaw = 181
ERR7_InvalidCreepViscoChainRow = 182
ERR7_InvalidCreepFunctionType = 183
ERR7_InvalidCreepShrinkageType = 184
ERR7_InvalidTableRow = 185
ERR7_ExceededMaxNumRows = 186
ERR7_InvalidLoadPathTemplateID = 187
ERR7_LoadPathTemplateIDAlreadyExists = 188
ERR7_InvalidLoadPathLane = 189
ERR7_ExceededMaxNumLoadPathTemplates = 190
ERR7_ExceededMaxNumLoadPathVehicles = 191
ERR7_InvalidLoadPathVehicle = 192
ERR7_InvalidMobilityType = 193
ERR7_InvalidAxisSystem = 194
ERR7_InvalidLoadPathID = 195
ERR7_LoadPathIDAlreadyExists = 196
ERR7_InvalidPathDefinition = 197
ERR7_InvalidLoadPathShape = 198
ERR7_InvalidLoadPathSurface = 199
ERR7_InvalidNumPathDivs = 200
ERR7_InvalidGeometryCavityLoop = 201
ERR7_InvalidLimitEnvelope = 202
ERR7_ExceededMaxNumLimitEnvelopes = 203
ERR7_InvalidCombEnvelope = 204
ERR7_ExceededMaxNumCombEnvelopes = 205
ERR7_InvalidFactorsEnvelope = 206
ERR7_ExceededMaxNumFactorsEnvelopes = 207
ERR7_InvalidLimitEnvelopeType = 208
ERR7_InvalidCombEnvelopeType = 209
ERR7_InvalidFactorsEnvelopeType = 210
ERR7_InvalidCombEnvelopeAccType = 211
ERR7_InvalidEnvelopeSet = 212
ERR7_ExceededMaxNumEnvelopeSets = 213
ERR7_InvalidEnvelopeSetType = 214
ERR7_InvalidCombResFile = 215
ERR7_ExceededMaxNumCombResFiles = 216
ERR7_CannotCombResFiles = 217
ERR7_InvalidStartEndTimes = 218
ERR7_InvalidNumSteps = 219
ERR7_InvalidLibraryPath = 220
ERR7_InvalidLibraryType = 221
ERR7_InvalidLibraryID = 222
ERR7_InvalidLibraryName = 223
ERR7_InvalidLibraryItemID = 224
ERR7_InvalidLibraryItemName = 225
ERR7_InvalidDisplayOptionsPath = 226
ERR7_InvalidSolverPath = 227
ERR7_InvalidCementHardeningType = 228
ERR7_NoPlateElements = 229
ERR7_CannotMakeBXS = 230
ERR7_CannotCalculateBXSData = 231
ERR7_InvalidSurfaceMeshTargetType = 232
ERR7_InvalidModalNodeReactType = 233
ERR7_InvalidAxis = 234
ERR7_InvalidBeamAxisType = 235
ERR7_InvalidStaadCountryCodeOption = 236
ERR7_InvalidGeometryFormatProtocol = 237
ERR7_InvalidDXFBeamOption = 238
ERR7_InvalidDXFPlateOption = 239
ERR7_InvalidLoadPathLaneFactorType = 240
ERR7_InvalidLoadPathVehicleInstance = 241
ERR7_InvalidNumBeamStations = 242
ERR7_ResFileUnsupportedType = 243
ERR7_ResFileAlreadyOpen = 244
ERR7_ResFileInvalidNumCases = 245
ERR7_ResFileNotOpen = 246
ERR7_ResFileInvalidCase = 247
ERR7_ResFileDoesNotHaveEntity = 248
ERR7_ResFileInvalidQuantity = 249
ERR7_ResFileQuantityNotExist = 250
ERR7_ResFileCantSave = 251
ERR7_ResFileCantClearQuantity = 252
ERR7_ResFileContainsNoElements = 253
ERR7_ResFileContainsNoNodes = 254
ERR7_InvalidName = 255
ERR7_ResFileAssociationNotAllowed = 256
ERR7_ResFileIncompatibleQuantity = 257
ERR7_CannotEditSolverFiles = 258
ERR7_CannotOpenResultFile = 259
ERR7_CouldNotShowModelWindow = 260
ERR7_ModelWindowWasNotShowing = 261
ERR7_CantDoWithModalWindows = 262
ERR7_InvalidSelectionEndEdgeFace = 263
ERR7_CouldNotCreateModelWindow = 264
ERR7_ModelWindowWasNotCreated = 265
ERR7_InvalidImageType = 266
ERR7_InvalidImageDimensions = 267
ERR7_ErrorCreatingImage = 268
ERR7_CannotSaveImageFile = 269
ERR7_InvalidWindowDimensions = 270
ERR7_InvalidResultQuantity = 271
ERR7_InvalidResultSubQuantity = 272
ERR7_InvalidComponent = 273
ERR7_ResultIsNotAvailable = 274
ERR7_InvalidUCSIndex = 275
ERR7_InvalidDiagramAxis = 276
ERR7_InvalidVectorComponents = 277
ERR7_TableTypeIsNotTimeBased = 278
ERR7_InvalidTableID = 279
ERR7_LinkNotMasterSlave = 280
ERR7_LinkNotSectorSymmetry = 281
ERR7_LinkNotCoupling = 282
ERR7_LinkNotPinned = 283
ERR7_LinkNotRigid = 284
ERR7_LinkNotShrink = 285
ERR7_LinkNotTwoPoint = 286
ERR7_LinkNotAttachment = 287
ERR7_LinkNotMultiPoint = 288
ERR7_InvalidCoupleType = 289
ERR7_InvalidRigidPlane = 290
ERR7_InvalidMultiPointType = 291
ERR7_InvalidMultiPointLink = 292
ERR7_InvalidAttachmentType = 293
ERR7_ExceededMaxNumColumns = 294
ERR7_CouldNotDestroyModelWindow = 295
ERR7_CannotSetWindowParent = 296
ERR7_InvalidLoadCaseFilePath = 297
ERR7_InvalidStaadLengthUnit = 298
ERR7_InvalidStaadForceUnit = 299
ERR7_InvalidDuplicateFaceType = 300
ERR7_InvalidNodeCoordinateKeepType = 301
ERR7_CommentDoesNotExist = 302
ERR7_InvalidFilePath = 303
ERR7_InvalidContactYieldType = 304
ERR7_InvalidNumMeshingLoops = 305
ERR7_InvalidMeshPositionOnUCS = 306
ERR7_InvalidK0Expression = 307
ERR7_InvalidK1Expression = 308
ERR7_InvalidNumCopies = 309
ERR7_InvalidCurvedPipesAsOption = 310
ERR7_InvalidResOptsRotationUnit = 311
ERR7_RayleighNotApplicable = 312
ERR7_InvalidAttributeSetting = 313
ERR7_InvalidToolOptsZipOptions = 314
ERR7_InvalidToolOptsSubdivideOptions = 315
ERR7_InvalidToolOptsCopyOptions = 316
ERR7_InvalidBackgroundMode = 317
ERR7_InvalidAttachPartsParams = 318
ERR7_InvalidDrawParameters = 319
ERR7_FilesStillOpen = 320
ERR7_SolverStillRunning = 321
ERR7_InvalidFaceFromBeamPolygonParameters = 322
ERR7_InvalidResOptsStrainUnit = 323
ERR7_FunctionNotSupported = 324
ERR7_SoilTypeNotMC = 325
ERR7_SoilTypeNotDP = 326
ERR7_TooManyAnimations = 327
ERR7_InvalidAnimationFile = 328
ERR7_InvalidAnimationMode = 329
ERR7_InsufficientFrames = 330
ERR7_AnimationDimensionsTooSmall = 331
ERR7_AnimationDimensionsTooLarge = 332
ERR7_ReducedAnimation = 333
ERR7_InvalidAnimationType = 334
ERR7_InvalidEntityID = 335
ERR7_CouldNotSaveAnimationFile = 336
ERR7_AnimationHandleOutOfRange = 337
ERR7_AnimationNotRunning = 338
ERR7_SoilTypeNotLS = 339
ERR7_InvalidPlane = 340
ERR7_InvalidAlphaTempType = 341
ERR7_InvalidGravityDirection = 342
ERR7_InvalidAttachmentDirection = 343
ERR7_InvalidHardeningType = 344
ERR7_ResultCaseNotInertiaRelief = 345
ERR7_InvalidNumLayers = 346
ERR7_PlateDoesNotHaveLayers = 347
ERR7_OperationFailed = 348
ERR7_InvalidEntityContourFileType = 349
ERR7_InvalidBrickIntegrationPoints = 350
ERR7_InvalidDirection = 351
ERR7_InvalidAttachConnectionType = 352
ERR7_CannotSaveIniFile = 353
ERR7_InvalidDivisionParameters = 354
ERR7_InvalidContourIndex = 355
ERR7_InvalidProjectFlag = 356
ERR7_InvalidSegmentsPerCircle = 357
ERR7_InvalidArcLength = 358
ERR7_InvalidDivisionTargets = 359
ERR7_InvalidProcessingMode = 360
ERR7_InvalidDigits = 361
ERR7_InvalidNumericStyle = 362
ERR7_InvalidExponentFormat = 363
ERR7_InvalidExportParameters = 364
ERR7_InsituCalculationFailed = 365
ERR7_ModelMixesAxiNonAxi = 366
ERR7_InvalidInsituRunMode = 367
ERR7_InvalidGradeType = 368
ERR7_InvalidGradeRatio = 369
ERR7_InvalidSplitData = 370
ERR7_CannotMorphEdges = 371
ERR7_TJunctionsFound = 372
ERR7_FreeEdgesFound = 373
ERR7_InvalidSTLFileFormat = 374
ERR7_InvalidSTLGroupingOption = 375
ERR7_InvalidSTLBeamOption = 376
ERR7_InvalidSTLPlateOption = 377
ERR7_InvalidNodeExtrudeTarget = 378
ERR7_InvalidBeamExtrudeTarget = 379
ERR7_InvalidLinkTarget = 380
ERR7_InvalidSourceAction = 381
ERR7_InvalidLinePoints = 382
ERR7_InvalidLineID = 383
ERR7_InvalidPlanePoints = 384
ERR7_InvalidPlaneID = 385
ERR7_InvalidSortMethod = 386
ERR7_InvalidDirectionVector = 387
ERR7_InvalidRCLayers = 388
ERR7_InvalidConnectionType = 389
ERR7_InvalidQuadraticAsOption = 390
ERR7_InvalidGeometryAsOption = 391
ERR7_InvalidSplitRatio = 392
ERR7_InvalidLength = 393
ERR7_InvalidEdgeTolerance = 394
ERR7_InvalidRadius = 395
ERR7_IncompatibleSections = 396
ERR7_UCSMustBeDifferent = 397
ERR7_InvalidNumCutFaces = 398
ERR7_InvalidNumRepeats = 399
ERR7_InvalidP1P2 = 400
ERR7_InvalidP1P2P3 = 401
ERR7_InvalidP1P2P3P4 = 402
ERR7_IntersectionNotFound = 403
ERR7_CantGenerateFillet = 404
ERR7_InvalidR1R2 = 405
ERR7_InvalidR2 = 406
ERR7_InvalidPLTarget = 407
ERR7_InvalidScaleAbout = 408
ERR7_InvalidProjectionDirection = 409
ERR7_InvalidCollectionID = 410
ERR7_InvalidDivisions = 411
ERR7_InvalidLineDefinition = 412
ERR7_InvalidOriginMethod = 413
ERR7_InvalidInfluenceFile = 414
ERR7_InvalidResponseVariable = 415
ERR7_NoMultiVariableInfluenceCases = 416
ERR7_InvalidMultiVariableCaseID = 417
ERR7_InvalidMultiVariableType = 418
ERR7_NoInfluenceCombinationsDefined = 419
ERR7_NothingSelected = 420
ERR7_InvalidPasteOption = 421
ERR7_InvalidResultCase = 422
ERR7_InvalidEntitySet = 423
ERR7_InvalidResOptsReactionLinkGNL = 424
ERR7_FileIsProtected = 425
ERR7_InvalidHRAMode = 426
ERR7_InvalidBGLData = 427
ERR7_InvalidWindowMode = 428
ERR7_UnexpectedSolverTermination = 429
ERR7_InvalidReferenceNode = 430
ERR7_InvalidDetachMode = 431
ERR7_InvalidResOptsBaseMode = 432
ERR7_InvalidMarkerType = 433
ERR7_InvalidMarkerStyle = 434
ERR7_InvalidMarkerLineThickness = 435
ERR7_InvalidMarkerSize = 436
ERR7_MarkerNotFound = 437
ERR7_PseudoTimeNotDefined = 438
ERR7_EquationDoesNotExist = 439
ERR7_InvalidOption = 440
ERR7_InvalidIterationNumber = 441
ERR7_InvalidAveragingOption = 442
ERR7_InvalidContourFileIndex = 443
ERR7_ContourFileNotLoaded = 444
ERR7_NoLoadPathsFound = 445
ERR7_NoElementsOnLoadPaths = 446
ERR7_NoResponsesFound = 447
ERR7_NoActiveResponseVariables = 448
ERR7_NoSoilElementsFound = 449
ERR7_OperationUserTerminated = 450
ERR7_InvalidDefaultsMode = 451
ERR7_InvalidFontName = 452

# Solver Error Codes
SE_NoLoadCaseSelected = 1001
SE_IncompatibleRestartFile = 1002
SE_ElementUsesInvalidProperty = 1003
SE_InvalidElement = 1004
SE_NeedNonlinearHeatSolver = 1005
SE_TableNotFound = 1006
SE_InvalidRestartFile = 1007
SE_InvalidInitialFile = 1008
SE_InvalidSolverResultFile = 1009
SE_InvalidLink = 1010
SE_InvalidPlateCohesionValue = 1011
SE_InvalidBrickCohesionValue = 1012
SE_NonlinearSolverRequired = 1013
SE_NoLoadTablesDefined = 1014
SE_NoVelocityDataInInitialFile = 1015
SE_NoModesIncluded = 1016
SE_InvalidTimeStep = 1017
SE_LoadIncrementsNotDefined = 1018
SE_NoFreedomCaseInIncrements = 1019
SE_InvalidInitialTemperatureFile = 1020
SE_InvalidFrequencyRange = 1021
SE_ModelMixesAxiNonAxi = 1022
SE_CompositeModuleNotAvailable = 1023
SE_CannotFindSolver = 1024
SE_UnknownException = 1025
SE_DuplicateLinks = 1026
SE_CannotAppendToFile = 1027
SE_CannotOverwriteFile = 1028
SE_CannotWriteToResultFile = 1029
SE_CannotWriteToLogFile = 1030
SE_CannotReadRestartFile = 1031
SE_InitialConditionsNotValid = 1032
SE_InvalidRayleighFactors = 1033
SE_SpectralExcitationsAllZero = 1034
SE_ShearPanelMustBeQuad4 = 1035
SE_SingularPlateMatrix = 1036
SE_SingularBrickMatrix = 1037
SE_NoBeamProperties = 1038
SE_NoPlateProperties = 1039
SE_NoBrickProperties = 1040
SE_MoreLoadIncrementsNeeded = 1041
SE_RubberRequiresGNL = 1042
SE_NoFreedomCaseSelected = 1043
SE_SpectralCasesNotDefined = 1044
SE_NoSpectralResultsSelected = 1045
SE_SpectralLoadExcitationsAllZero = 1046
SE_SpectralBaseExcitationsAllZero = 1047
SE_NoTimeStepsSaved = 1048
SE_InvalidDirectionVector = 1049
SE_HarmonicFactorsAllZero = 1050
SE_TemperatureDependenceCaseNotSet = 1051
SE_ZeroLengthRigidLinkGenerated = 1052
SE_InvalidStringGroupDefinition = 1053
SE_InvalidPreTensionOnString = 1054
SE_StringOrderHasChanged = 1055
SE_BadTaperData = 1056
SE_TaperedPlasticBeams = 1057
SE_NoMovingLoadPathsInCases = 1058
SE_NoResponseVariablesDefined = 1059
SE_InvalidPlateVariableRequested = 1060
SE_InvalidGravityCase = 1061
SE_InvalidUserPlateCreepDefinition = 1062
SE_InvalidUserBrickCreepDefinition = 1063
SE_InvalidPlateShrinkageDefinition = 1064
SE_InvalidBrickShrinkageDefinition = 1065
SE_InvalidLaminateID = 1066
SE_CannotReadWriteScratchPath = 1067
SE_CannotConvertAttachmentLink = 1068
SE_SoilRequiresMNL = 1069
SE_ActiveStageHasNoIncrements = 1070
SE_ConcreteCreepMNL = 1071
SE_CannotConvertInterpMultiPoint = 1072
SE_MissingInsituStress = 1073
SE_InvalidMaterialNonlinearString = 1074
SE_TensileInsituPlateStress = 1075
SE_TensileInsituBrickStress = 1076
SE_IncompatibleRestartUnits = 1077
SE_CreepTimeTooShort = 1078
SE_InvalidElements = 1079
SE_InsufficientRestartFileSteps = 1080
SE_NeedNodeTempNTASolver = 1081
SE_SingleShotRestartFile = 1082
SE_SkylineUsesBadSort = 1083
SE_StagedSolutionFileNotFound = 1084
SE_NeedTemperatureTables = 1085
SE_AttachmentsInWrongGroup = 1086
SE_StagingHasChanged = 1087
SE_NoNodes = 1088
SE_CQCRequiresDamping = 1089
SE_HaveLinearCables = 1090
SE_CableRequiresGNL = 1091
SE_BeamRequiresPoisson = 1092
SE_BeamPoissonOutOfRange = 1093
SE_CableRequiresMultiCaseOption = 1094
SE_InitialSolutionFileIsBad = 1095
SE_BeamPropertiesMayHaveChanged = 1096
SE_NeedElementNodeForce = 1097
SE_LinksHaveNoFreedomCase = 1098

# Solver Termination Error Codes
ST_NoError = 0
ST_Abnormal = -1
ST_UserStop = -2
ST_Internal = -3
ST_NoDisk = -4
ST_NoRam = -5
ST_OpenLog = -6
ST_CreateLog = -7
ST_WriteLog = -8
ST_MemError = -9
ST_Scratch = -10
ST_NoLicence = -11

# Other Constants
kMaxPlateResult = 1024
kMaxBrickResult = 1024
kMaxBeamRelease = 12
kMaxDisp = 6

# UCS
kMaxUCSDoubles = 10

# Solvers
stLinearStatic = 1
stLinearBuckling = 2
stNonlinearStatic = 3
stNaturalFrequency = 4
stHarmonicResponse = 5
stSpectralResponse = 6
stLinearTransientDynamic = 7
stNonlinearTransientDynamic = 8
stSteadyHeat = 9
stTransientHeat = 10
stLoadInfluence = 11
stQuasiStatic = 12

# Solver Modes
smNormalRun = 1
smProgressRun = 2
smBackgroundRun = 3
smNormalCloseRun = 4

# Result File Validation Bits
ibResFileNotFound = 1
ibResFileCannotOpen = 2
ibResFileNotResultFile = 3
ibResFileOldVersion = 4
ibResFileFutureVersion = 5
ibResFileWrongNumNodes = 6
ibResFileWrongNumBeams = 7
ibResFileWrongNumPlates = 8
ibResFileWrongNumBricks = 9
ibResFileWrongModelID = 10
ibResFileUnknownError = 11
ibResFileIsCombination = 12
ibResFileIsMultiFile = 13
ibResFileTruncated = 14

# Import/Export Modes
ieQuietRun = 0
ieProgressRun = 1

# NASTRAN
ipNASTRANImportUnits = 0
ipNASTRANFreedomCase = 0
ipNASTRANLoadCaseNSMass = 1
ipNASTRANSolver = 2
ipNASTRANExportUnits = 3
ipNASTRANBeamStressSections = 4
ipNASTRANBeamSectionGeometry = 5
ipNASTRANExportHeatTransfer = 6
ipNASTRANExportNSMass = 7
ipNASTRANExportUnusedProps = 8
ipNASTRANTemperatureCase = 9
ipNASTRANPreLoadCase = 10
ipNASTRANNInc = 11
ipNASTRANMaxIter = 12
ipNASTRANDoEPSU = 13
ipNASTRANDoEPSP = 14
ipNASTRANDoEPSW = 15
ipNASTRANExportPyramid = 16
ipNASTRANExportQuad4 = 17
ipNASTRANExportZeroFields = 0
ipNASTRANEPSU = 1
ipNASTRANEPSP = 2
ipNASTRANEPSW = 3
ieNASTRANSolverLSA = 0
ieNASTRANSolverNFA = 1
ieNASTRANSolverLBA = 2
ieNASTRANSolverNLA = 3
ieNASTRANExportGeometryProps = 0
ieNASTRANExportPropsOnly = 1
ieNASTRANExportPyramidAsHexa = 0
ieNASTRANExportPyramidAsPyram = 1
ieNASTRANExportCQUAD4 = 0
ieNASTRANExportCQUADR = 1
usNASTRAN_kg_N_m = 0
usNASTRAN_T_N_mm = 1
usNASTRAN_sl_lbf_ft = 2
usNASTRAN_lbm_lbf_in = 3
usNASTRAN_sl_lbf_in = 4
usNASTRAN_None = 5

# ANSYS
ipANSYSImportFormat = 0
ipANSYSArrayParameters = 1
ipANSYSImportLoadCaseFiles = 2
ipANSYSImportIGESEntities = 3
ipANSYSFixElementConnectivity = 4
ipANSYSRemoveDuplicateProps = 5
ipANSYSExportFormat = 0
ipANSYSFreedomCase = 1
ipANSYSLoadCase = 2
ipANSYSUnits = 3
ipANSYSEndRelease = 4
ipANSYSExportNonlinearMat = 5
ipANSYSExportHeatTransfer = 6
ipANSYSExportPreLoadNSMass = 7
ipANSYSExportTetraOption = 8
ipANSYSExportQuad8Option = 9
ieANSYSBatchImport = 0
ieANSYSCDBImport = 1
ieANSYSBatchCDBImport = 2
ieANSYSBatch1Export = 0
ieANSYSBatch3Export = 1
ieANSYSBlockedCDBExport = 2
ieANSYSUnblockedCDBExport = 3
ieANSYSArrayOverwrite = 0
ieANSYSArrayIgnore = 1
ieANSYSArrayPrompt = 2
ieANSYSEndReleaseFixed = 0
ieANSYSEndReleaseFull = 1
usANSYS_None = 0
usANSYS_kg_m_C = 1
usANSYS_g_cm_C = 2
usANSYS_T_mm_C = 3
usANSYS_sl_ft_F = 4
usANSYS_lbm_in_F = 5

# STAAD
ipSTAADCountryType = 0
ipSTAADIncludeSectionLibrary = 1
ipSTAADStripUnderscore = 2
ipSTAADStripSectionSpaces = 3
ipSTAADStripCaseQualifiers = 4
ipSTAADLengthUnit = 5
ipSTAADForceUnit = 6
ieSTAADAmericanCode = 0
ieSTAADAustralianCode = 1
ieSTAADBritishCode = 2
luSTAADInch = 0
luSTAADFoot = 1
luSTAADCentimetre = 2
luSTAADMetre = 3
luSTAADMillimetre = 4
luSTAADDecimetre = 5
luSTAADKilometre = 6
fuSTAADKip = 0
fuSTAADPoundForce = 1
fuSTAADKilogramForce = 2
fuSTAADMegatonneForce = 3
fuSTAADNewton = 4
fuSTAADKilonewton = 5
fuSTAADMeganewton = 6
fuSTAADDecanewton = 7

# SAP2000
ipSAP2000DecimalSeparator = 0
ipSAP2000ThousandSeparator = 1
ipSAP2000MergeDuplicateFreedomSets = 2
ieSAP2000Period = 0
ieSAP2000Comma = 1
ieSAP2000Space = 2
ieSAP2000None = 3

# ST7
ipSt7ImportRemoveCases = 0
ieSt7ExportCurrent = 0
ieSt7Export106 = 1
ieSt7Export21x = 2
ieSt7Export22x = 3
ieSt7Export23x = 4
ieSt7Export24x = 5

# STL
ipSTLImportProperty = 0
ipSTLImportLengthUnit = 1
ipSTLExportFormat = 0
ipSTLExportGrouping = 1
ipSTLExportBeams = 2
ipSTLExportPlates = 3
ipSTLExportBricks = 4
ipSTLExportGeometryFaces = 5
ipSTLExportBeamsAs = 6
ipSTLExportPlatesAs = 7
ipSTLExportBeamOffsets = 8
ipSTLExportPlateOffsets = 9
ipSTLExportInternalBrickFaces = 10
luSTLNone = 0
luSTLMillimetre = 1
luSTLCentimetre = 2
luSTLMetre = 3
luSTLInch = 4
luSTLFoot = 5
ieSTLText = 0
ieSTLBinary = 1
ieSTLGroupByNone = 0
ieSTLGroupByEntityType = 1
ieSTLGroupByGroups = 2

# GEOMETRY
ipGeomImportProperty = 0
ipGeomImportCurvesToBeams = 1
ipGeomImportGroupsAs = 2
ipGeomImportColourAsProperty = 3
ipGeomImportLengthUnit = 4
ipGeomExportColour = 0
ipGeomExportGroupsAsLevels = 1
ipGeomExportFullGroupPath = 2
ipGeomExportFormatProtocol = 3
ipGeomExportCurve = 4
ipGeomExportPeriodicFace = 5
ipGeomExportKeepAnalytic = 6
ipGeomImportTol = 0
luGeomNone = -1
luGeomInch = 0
luGeomMillimetre = 1
luGeomFoot = 2
luGeomMile = 3
luGeomMetre = 4
luGeomKilometre = 5
luGeomMil = 6
luGeomMicron = 7
luGeomCentimetre = 8
luGeomMicroinch = 9
luGeomUnspecified = 10

# IGES Formats
ieIGESBoundedSurface = 0
ieIGESTrimmedParametricSurface = 1
ieIGESOpenShell = 2
ieIGESManifoldSolidBRep = 3

# STEP Protocols
ieSTEPConfigControlDesign = 0
ieSTEPAutomotiveDesign = 1

# Geometry Export Format Options
ieGeomModelOnly = 0
ieGeomParameterOnly = 1
ieGeomModelPreferred = 2
ieGeomParameterPreferred = 3
ieGeomSeamOnlyAsRequired = 0
ieGeomSplitOnFaceBoundary = 1
ieGeomSplitIntoHalves = 2
ieGeomColourNone = 0
ieGeomFaceColour = 1
ieGeomGroupColour = 2
ieGeomPropertyColour = 3

# DXF Options
ipDXFImportFrozenLayers = 0
ipDXFImportLayersAsGroups = 1
ipDXFImportColoursAsProps = 2
ipDXFImportPolylineAsPlates = 3
ipDXFImportPolygonAsBricks = 4
ipDXFImportSegmentsPerCircle = 5
ipDXFImportUseSegmentsPerCircle = 6
ipDXFImportLengthUnit = 7
ipDXFImportProperty = 8
ipDXFExportPlatesBricks3DFaces = 0
ipDXFExportGroupsAsLayers = 1
ipDXFExportPropColoursAsEntityColours = 2
ipDXFExportBeamsAs = 3
ipDXFExportPlatesAs = 4
ipDXFExportBeamOffsets = 5
ipDXFExportPlateOffsets = 6
ipDXFExportInternalBrickFaces = 7
ipDXFImportArcLength = 0

# DXF and STL
ieBeamAsLine = 0
ieBeamAsSection = 1
ieBeamAsSolid = 2
iePlateAsSurface = 0
iePlateAsSolid = 1

# Geometry Groups
ggNone = 0
ggAuto = 1
ggSubfigures = 2
ggLevels = 3
ggAssemblies = 1
ggBlocks = 2
ggLayers = 3
ggBodies = 1

# BXS
ipBXSXBar = 0
ipBXSYBar = 1
ipBXSArea = 2
ipBXSI11 = 3
ipBXSI22 = 4
ipBXSAngle = 5
ipBXSZ11Plus = 6
ipBXSZ11Minus = 7
ipBXSZ22Plus = 8
ipBXSZ22Minus = 9
ipBXSS11 = 10
ipBXSS22 = 11
ipBXSr1 = 12
ipBXSr2 = 13
ipBXSSA1 = 14
ipBXSSA2 = 15
ipBXSSL1 = 16
ipBXSSL2 = 17
ipBXSIXX = 18
ipBXSIYY = 19
ipBXSIXY = 20
ipBXSIxxL = 21
ipBXSIyyL = 22
ipBXSIxyL = 23
ipBXSZxxPlus = 24
ipBXSZxxMinus = 25
ipBXSZyyPlus = 26
ipBXSZyyMinus = 27
ipBXSSxx = 28
ipBXSSyy = 29
ipBXSrx = 30
ipBXSry = 31
ipBXSJ = 32
ipBXSIw = 33
ipBXSrdA = 34
ipBXSPC1 = 35
ipBXSPC2 = 36
ipBXSPCx = 37
ipBXSPCy = 38

# BXS Loop Types
ltUnknown = 0
ltOuter = 1
ltInner = 2

# Geometry Clean - Doubles
ipGeometryFeatureLength = 0
ipGeometryEdgeMergeAngle = 1

# Geometry Clean - Integers
ipGeometryFeatureType = 0
ipGeometryActOnWholeModel = 1
ipGeometryFreeEdgesOnly = 2
ipGeometryDuplicateFaces = 3
ipGeometryWithinGroups = 4
dfLeaveAll = 0
dfLeaveOne = 1
dfLeaveNone = 2

# Mesh Clean - Doubles
ipMeshTolerance = 0

# Mesh Clean - Integers
ipMeshToleranceType = 0
ipZipNodes = 1
ipRemoveDuplicateElements = 2
ipFixElementConnectivity = 3
ipDeleteFreeNodes = 4
ipDoBeams = 5
ipDoPlates = 6
ipDoBricks = 7
ipDoLinks = 8
ipZeroLengthLinks = 9
ipZeroLengthBeams = 10
ipNodeAttributeKeep = 11
ipNodeCoordinates = 12
ipAllowDifferentProps = 13
ipActOnWholeModel = 14
ipAllowDifferentGroups = 15
ipPackStringGroupIDs = 16

# Attribute keep
naLower = 0
naHigher = 1
naAccumulate = 2

# Node coordinates
ncAverage = 0
ncLowerNode = 1
ncHigherNode = 2
ncSelectedNode = 3

# Surface Meshing - Integers
ipSurfaceMeshMode = 0
ipSurfaceMeshSizeMode = 1
ipSurfaceMeshTargetNodes = 2
ipSurfaceMeshTargetPropertyID = 3
ipSurfaceMeshAutoCreateProperties = 4
ipSurfaceMeshMinEdgesPerCircle = 5
ipSurfaceMeshApplyTransitioning = 6
ipSurfaceMeshAllowUserStop = 7
ipSurfaceMeshConsiderNearVertex = 8
ipSurfaceMeshSelectedFaces = 9
ipSurfaceMeshApplySurfaceCurvature = 10
mmAuto = 0
mmCustom = 1
smPercentage = 0
smAbsolute = 1

# Surface Meshing - Doubles
ipSurfaceMeshSize = 0
ipSurfaceMeshLengthRatio = 1
ipSurfaceMeshMaximumIncrease = 2
ipSurfaceMeshOnEdgesLongerThan = 3

# Tetra Meshing
ipTetraMeshSize = 0
ipTetraMeshProperty = 1
ipTetraMeshInc = 2
ipTetraMesh10 = 3
ipTetraMeshGroupsAsSolids = 4
ipTetraMeshSmooth = 5
ipTetraMeshAutoCreateProperties = 7
ipTetraMeshDeletePlates = 8
ipTetraMeshMultiBodyOption = 9
ipTetraMeshAllowUserStop = 10
ipTetraMeshCheckSelfIntersect = 11

# Direct Tetra Meshing
ipDirectTetraMeshMode = 0
ipDirectTetraMeshSizeMode = 1
ipDirectTetraMinEdgesPerCircle = 2
ipDirectTetraApplyTransitioning = 3
ipDirectTetraApplySurfaceCurvature = 4
ipDirectTetraAllowUserStop = 5
ipDirectTetraConsiderNearVertex = 6
ipDirectTetraMeshSelectedGroups = 7
ipDirectTetraMeshSize = 8
ipDirectTetraMesh10 = 9
ipDirectTetraMeshSmooth = 10
ipDirectTetraAutoCreateProperties = 11
msFine = 1
msMedium = 2
msCoarse = 3
mbCancelMeshing = 0
mbCavity = 1
mbSeparateSolids = 2

# Polygon Meshing
ipMeshTargetNodes = 0
ipMeshTargetPropertyID = 1
ipMeshUCSID = 2
ipMeshGroupID = 3
ipMeshPositionUCS = 0

# Image Types
itBitmap8Bit = 1
itBitmap16Bit = 2
itBitmap24Bit = 3
itJPEG = 4
itPNG = 5

# Window State
wsModelWindowNotCreated = 0
wsModelWindowVisible = 1
wsModelWindowMaximised = 2
wsModelWindowMinimised = 3
wsModelWindowHidden = 4

# DISPLAY SETTINGS DEFAULTS

# Defaults Mode
mdFactoryDefaults = 0
mdUserDefaults = 1

# Model Defaults
mdViewOptions = 0
mdEntityOptions = 1
mdBeamPreContourOptions = 2
mdPlatePreContourOptions = 3
mdBrickPreContourOptions = 4
mdAttributeOptions = 5
mdResultOptions = 6
mdBeamResultContourOptions = 7
mdPlateResultContourOptions = 8
mdBrickResultContourOptions = 9
mdLinkResultContourOptions = 10
mdPrintOptions = 11

# mdViewOptions
ipDefBackgroundTab = 0
ipDefAxisTab = 1
ipDefRotationTab = 2
ipDefDrawingTab = 3
ipDefPreNumbersTab = 4
ipDefFreeEdgeTab = 5
ipDefSelectionTab = 6

# mdEntityOptions
ipDefNodeTab = 0
ipDefBeamTab = 1
ipDefPlateTab = 2
ipDefBrickTab = 3
ipDefLinkTab = 4
ipDefPathTab = 5
ipDefVertexTab = 6
ipDefFaceTab = 7

# mdPreContourOptions, mdResultContourOptions
ipDefContourStyleTab = 0
ipDefContourLimitsTab = 1
ipDefContourLegendTab = 2
ipDefContourDiagramTab = 3

# mdAttributeOptions
ipDefNodeAttribTab = 0
ipDefBeamAttribTab = 1
ipDefPlateAttribTab = 2
ipDefBrickAttribTab = 3
ipDefPathAttribTab = 4

# mdResultOptions
ipDefResShowHideTab = 0
ipDefResPostNumbersTab = 1
ipDefResCombinationsTab = 2
ipDefResEnvelopesTab = 3
ipDefResDisplacementTab = 4

# mdPrintOptions
ipHeaderFooterTab = 0
ipPageSetupTab = 1
ipFontsTab = 2

# ENTITY DISPLAY SETTINGS

# Label Style
lsNone = 0
lsEntityNumber = 1
lsIDNumber = 2
lsPropertyNumber = 3
lsPropertyName = 4
lsPropertyType = 5
lsLinkType = 3
lsLaneNumber = 2

# Outline Thickness Limits
kMinThickness = 1
kMaxThickness = 5

# Element Outline Style
omEdge = 0
omPropertyBoundary = 1
omGroupBoundary = 2
omFacetAngle = 3
omFacetProperty = 4
omFacetGroup = 5

# Shrink Limits
kMinShrink = 0
kMaxShrink = 95

# Point Styles
psCircle = 0
psSquare = 1

# Point Size Limits
kMinPointSize = 0
kMaxPointSize = 5

# NODE ENTITY DISPLAY

# Node Show
nsFreeNodeAll = 0
nsFreeNodeNone = 1
nsFreeNodeGroup = 2

# Node Colour Indexes
clNodeUnselected = 0
clNodeSelected = 1

# BEAM ENTITY DISPLAY

# Beam Display Style
bsLine = 0
bsSection = 1
bsSolid = 2
bsSlice = 3

# Beam Fill Colour Type
bfNone = 0
bfProperty = 1
bfGroup = 2
bfColour = 3
bfOrientation = 4
bfContour = 5

# Beam Outline Colour Type
blNone = 0
blProperty = 1
blGroup = 2
blColour = 3
blOrientation = 4
blContour = 5

# Beam Colour Indexes
ipBeamFillColour = 0
ipBeamLineColour = 1
ipBeamOrientation1Colour = 2
ipBeamOrientation2Colour = 3
ipBeamNRefColour = 4

# Beam Spring Coil Limits
kMinSpringCoils = 5
kMaxSpringCoils = 30
kMinSpringAspect = 5
kMaxSpringAspect = 50

# Beam Round Facets Limits
kMinFacets = 8
kMaxFacets = 32
kMinSlices = 4
kMaxSlices = 20

# PLATE ENTITY DISPLAY

# Plate Display Style
psSurface = 0
psSolid = 1

# Plate Fill Colour Type
pfNone = 0
pfProperty = 1
pfGroup = 2
pfColour = 3
pfOrientation = 4
pfContour = 5

# Plate Outline Colour Type
plNone = 0
plProperty = 1
plGroup = 2
plColour = 3

# Plate Colour Indexes
ipPlateFillColour = 0
ipPlateLineColour = 1
ipPlateOrientation1Colour = 2
ipPlateOrientation2Colour = 3
ipPlateOrientation3Colour = 4
ipPlateOffsetColour = 5

# BRICK ENTITY DISPLAY

# Brick Fill Colour Type
kfNone = 0
kfProperty = 1
kfGroup = 2
kfColour = 3
kfContour = 4

# Brick Outline Colour Type
klNone = 0
klProperty = 1
klGroup = 2
klColour = 3

# Brick Colour Indexes
ipBrickFillColour = 0
ipBrickLineColour = 1

# LINK ENTITY DISPLAY

# Link Outline Colour Type
llType = 0
llGroup = 1
llGlobal = 2

# Link Colour Indexes
ipLinkColour = 0
ipMasterSlaveColour = 1
ipSectorSymmetryColour = 2
ipCouplingColour = 3
ipPinnedColour = 4
ipRigidColour = 5
ipShrinkColour = 6
ipTwoPointColour = 7
ipAttachmentColour = 8
ipInterpolatedMPLColour = 9
ipMasterSlaveMPLColour = 10
ipPinnedMPLColour = 11
ipRigidMPLColour = 12
ipUserMPLColour = 13
ipReactionMPLColour = 14

# VERTEX ENTITY DISPLAY

# Vertex Show
vsFreeVertexAll = 0
vsFreeVertexNone = 1
vsFreeVertexGroup = 2

# Vertex Colours Indexes
ipVertexFreeColour = 0
ipVertexFixedColour = 1
ipVertexSelectedColour = 2

# FACE ENTITY DISPLAY

# Face Fill Style
fdNone = 0
fdWireframe = 1
fdSolid = 2

# Face Fill Colour Type
ffProperty = 0
ffGroup = 1
ffFaceNumber = 2
ffColour = 3
ffOrientation = 4
ffFaceID = 5

# Face Line Colour Type
flNone = 0
flProperty = 1
flGroup = 2
flFaceNumber = 3
flColour = 4
flFaceID = 5

# Face Colour Indexes
ipFaceFillColour = 0
ipFaceLineColour = 1
ipFaceOrientation1Colour = 2
ipFaceOrientation2Colour = 3
ipFaceNIEdgesColour = 4
ipFaceCPuColour = 5
ipFaceCPvColour = 6
ipFaceNormalsColour = 7

# PATH ENTITY DISPLAY

# Path Fill Colour Type
tfNone = 0
tfTemplate = 1
tfGroup = 2
tfPathNumber = 3
tfColour = 4
tfOrientation = 5

# Path Outline Colour Type
tlNone = 0
tlTemplate = 1
tlGroup = 2
tlPathNumber = 3
tlColour = 4

# Path Colour Indexes
ipPathFillColour = 0
ipPathLineColour = 1
ipPathOrientation1Colour = 2
ipPathOrientation2Colour = 3

# ATTRIBUTE DISPLAY
ipAttribDisplayShow = 0
ipAttribDisplayLabel = 1
ipAttribDisplayResultant = 2
ipAttribDisplayAnchorTail = 3
ipAttribDisplayScaled = 4
ipAttribDisplaySize = 5
ipAttribDisplayThickness = 6
ipAttribDisplayCol1 = 7
ipAttribDisplayCol2 = 8
ipAttribDisplayCol3 = 9

# Window Background Modes
bgSolid = 0
bgImage = 1
bgGradient = 2

# Window Display Modes
wmPreProcessing = 0
wmPostProcessing = 1

# Numeric Modes
nmPreProcessing = 0
nmPostProcessing = 1

# Numeric Styles
nsFixed = 0
nsEngineering = 1
nsScientific = 2
nsAuto = 3

# Exponent formats
efLowered = 0
efRaised = 1

# Entity Display Settings - Beam Contour Types
ctBeamNone = 0
ctBeamLength = 1
ctBeamAxis1 = 2
ctBeamAxis2 = 3
ctBeamAxis3 = 4
ctBeamEA = 5
ctBeamEI11 = 6
ctBeamEI22 = 7
ctBeamGJ = 8
ctBeamEAFactor = 9
ctBeamEI11Factor = 10
ctBeamEI22Factor = 11
ctBeamGJFactor = 12
ctBeamOffset1 = 13
ctBeamOffset2 = 14
ctBeamStiffnessFactor1 = 15
ctBeamStiffnessFactor2 = 16
ctBeamStiffnessFactor3 = 17
ctBeamStiffnessFactor4 = 18
ctBeamStiffnessFactor5 = 19
ctBeamStiffnessFactor6 = 20
ctBeamMassFactor = 21
ctBeamSupportM1 = 22
ctBeamSupportP1 = 23
ctBeamSupportM2 = 24
ctBeamSupportP2 = 25
ctBeamSupportGapM1 = 26
ctBeamSupportGapP1 = 27
ctBeamSupportGapM2 = 28
ctBeamSupportGapP2 = 29
ctBeamTemperature = 30
ctBeamPreTension = 31
ctBeamPreStrain = 32
ctBeamPreCurvature1 = 33
ctBeamPreCurvature2 = 34
ctBeamTempGradient1 = 35
ctBeamTempGradient2 = 36
ctBeamPipePressureIn = 37
ctBeamPipePressureOut = 38
ctBeamPipeTempIn = 39
ctBeamPipeTempOut = 40
ctBeamConvectionCoeff = 41
ctBeamConvectionAmbient = 42
ctBeamRadiationCoeff = 43
ctBeamRadiationAmbient = 44
ctBeamHeatFlux = 45
ctBeamHeatSource = 46
ctBeamAgeAtFirstLoading = 47

# Entity Display Settings - Plate Contour Types
ctPlateNone = 0
ctPlateAspectRatioMin = 1
ctPlateAspectRatioMax = 2
ctPlateWarping = 3
ctPlateInternalAngle = 4
ctPlateInternalAngleRatio = 5
ctPlateDiscreteThicknessM = 6
ctPlateContinuousThicknessM = 7
ctPlateDiscreteThicknessB = 8
ctPlateContinuousThicknessB = 9
ctPlateOffset = 10
ctPlateArea = 11
ctPlateAxis1 = 12
ctPlateAxis2 = 13
ctPlateAxis3 = 14
ctPlateTemperature = 15
ctPlateEdgeNormalSupport = 16
ctPlateEdgeLateralSupport = 17
ctPlateEdgeSupportGap = 18
ctPlateFaceNormalSupportMZ = 19
ctPlateFaceNormalSupportPZ = 20
ctPlateFaceLateralSupportMZ = 21
ctPlateFaceLateralSupportPZ = 22
ctPlateFaceSupportGapMZ = 23
ctPlateFaceSupportGapPZ = 24
ctPlatePreStressX = 25
ctPlatePreStressY = 26
ctPlatePreStressZ = 27
ctPlatePreStressMagnitude = 28
ctPlatePreStrainX = 29
ctPlatePreStrainY = 30
ctPlatePreStrainZ = 31
ctPlatePreStrainMagnitude = 32
ctPlatePreCurvatureX = 33
ctPlatePreCurvatureY = 34
ctPlatePreCurvatureMagnitude = 35
ctPlateTempGradient = 36
ctPlateEdgeNormalPressure = 37
ctPlateEdgeShear = 38
ctPlateEdgeTransverseShear = 39
ctPlateEdgeGlobalPressure = 40
ctPlateEdgeGlobalPressureX = 41
ctPlateEdgeGlobalPressureY = 42
ctPlateEdgeGlobalPressureZ = 43
ctPlatePressureNormalMZ = 44
ctPlatePressureNormalPZ = 45
ctPlatePressureGlobalMZ = 46
ctPlatePressureGlobalXMZ = 47
ctPlatePressureGlobalYMZ = 48
ctPlatePressureGlobalZMZ = 49
ctPlatePressureGlobalPZ = 50
ctPlatePressureGlobalXPZ = 51
ctPlatePressureGlobalYPZ = 52
ctPlatePressureGlobalZPZ = 53
ctPlateFaceShearX = 54
ctPlateFaceShearY = 55
ctPlateFaceShearMagnitude = 56
ctPlateNSMass = 57
ctPlateDynamicFactor = 58
ctPlateConvectionCoeff = 59
ctPlateConvectionAmbient = 60
ctPlateRadiationCoeff = 61
ctPlateRadiationAmbient = 62
ctPlateHeatFlux = 63
ctPlateConvectionCoeffZPlus = 64
ctPlateConvectionCoeffZMinus = 65
ctPlateConvectionAmbientZPlus = 66
ctPlateConvectionAmbientZMinus = 67
ctPlateRadiationCoeffZPlus = 68
ctPlateRadiationCoeffZMinus = 69
ctPlateRadiationAmbientZPlus = 70
ctPlateRadiationAmbientZMinus = 71
ctPlateHeatSource = 72
ctPlateSoilStressSV = 73
ctPlateSoilStressKO = 74
ctPlateSoilStressSH = 75
ctPlateSoilRatioOCR = 76
ctPlateSoilRatioEO = 77
ctPlateSoilFluidLevel = 78
ctPlateAgeAtFirstLoading = 79

# Entity Display Settings - Brick Contour Types
ctBrickNone = 0
ctBrickAspectRatioMin = 1
ctBrickAspectRatioMax = 2
ctBrickVolume = 3
ctBrickDeterminant = 4
ctBrickInternalAngle = 5
ctBrickMixedProduct = 6
ctBrickDihedral = 7
ctBrickAxis1 = 8
ctBrickAxis2 = 9
ctBrickAxis3 = 10
ctBrickTemperature = 11
ctBrickNormalSupport = 12
ctBrickLateralSupport = 13
ctBrickSupportGap = 14
ctBrickPreStressX = 15
ctBrickPreStressY = 16
ctBrickPreStressZ = 17
ctBrickPreStressMagnitude = 18
ctBrickPreStrainX = 19
ctBrickPreStrainY = 20
ctBrickPreStrainZ = 21
ctBrickPreStrainMagnitude = 22
ctBrickNormalPressure = 23
ctBrickGlobalPressure = 24
ctBrickGlobalPressureX = 25
ctBrickGlobalPressureY = 26
ctBrickGlobalPressureZ = 27
ctBrickShearX = 28
ctBrickShearY = 29
ctBrickShearMagnitude = 30
ctBrickNSMass = 31
ctBrickDynamicFactor = 32
ctBrickConvectionCoeff = 33
ctBrickConvectionAmbient = 34
ctBrickRadiationCoeff = 35
ctBrickRadiationAmbient = 36
ctBrickHeatFlux = 37
ctBrickHeatSource = 38
ctBrickSoilStressSV = 39
ctBrickSoilStressKO = 40
ctBrickSoilStressSH = 41
ctBrickSoilRatioOCR = 42
ctBrickSoilRatioEO = 43
ctBrickSoilFluidLevel = 44
ctBrickAgeAtFirstLoading = 45

# Beam/Plate/Brick/Link Result Display Type - INDEXED BY ipResultType
rtAsNone = 0
rtAsContour = 1
rtAsDiagram = 2
rtAsVector = 3

# Node Output Display Quantity - Indexed by ipResultQuantity
rqDispC = 101
rqVelC = 102
rqAccC = 103
rqPhaseC = 104
rqReactC = 105
rqTempC = 106
rqNodeForceC = 107
rqNodeFluxC = 108

# Beam Output Display Quantity - Indexed by ipResultQuantity
rqBeamForceC = 201
rqBeamStrainC = 202
rqBeamStressC = 203
rqBeamCreepStrainC = 204
rqBeamEnergyC = 205
rqBeamFluxC = 206
rqBeamTGradC = 207
rqBeamTotalStrainC = 208

# Plate Output Display Quantity - Indexed by ipResultQuantity
rqPlateForceC = 301
rqPlateMomentC = 302
rqPlateStressC = 303
rqPlateStrainC = 304
rqPlateCurvatureC = 305
rqPlateCreepStrainC = 306
rqPlateEnergyC = 307
rqPlateFluxC = 308
rqPlateTGradC = 309
rqPlateRCDesignC = 310
rqPlatePlyStressC = 311
rqPlatePlyStrainC = 312
rqPlatePlyReserveC = 313
rqPlateSoilC = 314
rqPlateTotalStrainC = 315
rqPlateTotalCurvatureC = 316

# Brick Output Display Quantity - Indexed by ipResultQuantity
rqBrickStressC = 401
rqBrickStrainC = 402
rqBrickCreepStrainC = 403
rqBrickEnergyC = 404
rqBrickFluxC = 405
rqBrickTGradC = 406
rqBrickSoilC = 407
rqBrickTotalStrainC = 408

# Link Output Display Quantity - Indexed by ipResultQuantity
rqLinkForceC = 501
rqLinkFluxC = 502
rqLinkMPLReactionC = 503

# Plate RC Output Display Sub-quantity - Indexed by ipResultComponent
rcWoodArmerMoment = 0
rcWoodArmerForce = 1
rcSteelRequirementMin = 2
rcConcreteStrain = 3
rcSteelRequirementLessBase = 4
rcUserSteelStress = 5
rcUserConcreteStrain = 6
rcBlockRatio = 7

# Plate RC Area Output Display Sub-quantity - Indexed by ipResultSystem
rsAreaPerLength = 0
rsBarSpacing = 1
rsBarDiameter = 2
rsAreaPerAreaSlab = 3
rsAreaPerAreaBase = 4

# Plate Composite Output Display Sub-quantity - Indexed by ipResultSystem
rsPlyMinValue = -1
rsPlyMaxValue = -2
rsPlyMaxMag = -3
rsPlyMinValueActivePlies = -4
rsPlyMaxValueActivePlies = -5
rsPlyMaxMagActivePlies = -6

# Vector Styles - Indexed by ipVectorStyle
vtVectorTranslationMag = 1
vtVectorRotationMag = 2
vtVectorTranslationComponents = 3
vtVectorRotationComponents = 4

# Result Display Indexes
ipResultType = 0
ipResultQuantity = 1
ipResultSystem = 2
ipResultComponent = 3
ipResultSurface = 4
ipVectorStyle = 5
ipReferenceNode = 6
ipAbsoluteValue = 7
ipDiagram1 = 7
ipDiagram2 = 8
ipDiagram3 = 9
ipDiagram4 = 10
ipDiagram5 = 11
ipDiagram6 = 12
ipVector1 = 7
ipVector2 = 8
ipVector3 = 9
ipVector4 = 10
ipVector5 = 11
ipVector6 = 12

# Contour Settings - Style Constants
csRainbow = 0
csRainbowEnds = 1
csMono = 2
csLines = 3
csBands = 4
vaTail = 0
vaHead = 1

# Contour Settings - Style Indexes
ipContourStyle = 0
ipReverse = 1
ipSeparator = 2
ipBand1Colour = 3
ipBand2Colour = 4
ipSeparatorColour = 5
ipLineBackColour = 6
ipMonoColour = 7
ipMinColour = 8
ipMaxColour = 9
ipLimitMin = 10
ipLimitMax = 11
ipVectorThickness = 12
ipVectorLength = 13
ipVectorAnchor = 14

# Contour Settings - Limits Constants
clDefault = 0
clUserRange = 1
clRounded = 2
clUserSpecified = 3
cmContinuous = 0
cmDiscrete = 1

# Contour Settings - Limits Indexes
ipContourLimit = 0
ipContourMode = 1
ipNumContours = 2
ipSetMinLimit = 3
ipSetMaxLimit = 4
ipMinLimit = 0
ipMaxLimit = 1

# Contour Settings - Legend Constants
lpNone = 0
lpTopLeft = 1
lpTopRight = 2
lpBottomLeft = 3
lpBottomRight = 4
lpFloating = 5

# Contour Settings - Legend Indexes
ipLegendPosition = 0
ipOpaqueLegend = 1
ipShowMinMax = 2
ipHistogram = 3
ipLegendWidth = 4
ipLegendHeight = 5

# Contour Settings - Diagram Constants
dsSingleLine = 0
dsHatched = 1

# Contour Settings - Diagram Indexes
ipDiagramStyle = 0
ipDiagramAxialDir = 1
ipDiagramTorqueDir = 2
ipDiagramRelativeLength = 3
ipDiagramThickness = 4

# Font Settings
ipFontSize = 0
ipFontColour = 1
ipFontStyleBold = 2
ipFontStyleItalic = 3
ipFontStyleUnderline = 4

# Displacement Scales
dsPercent = 0
dsAbsolute = 1

# Reference Displacement Modes
rdNone = 0
rdPreviousCase = -1

# User Contour File Types
ucNode = 0
ucElement = 1

# Utility
auRadian = 0
auDegree = 1

# Transient Base Modes
bmRelative = 0
bmTotal = 1

# Result Options
ipResOptsRotationUnit = 0
ipResOptsStrainUnit = 1
ipResOptsAddGNLDisp = 2
ipResOptsOffsetDisp = 3
ipResOptsNFADisp = 4
ipResOptsReactionLinkGNL = 5
ipResOptsBaseDisp = 6
ipResOptsBaseVel = 7
ipResOptsBaseAcc = 8

# Result Options - Strain Units
suUnit = 0
suPercent = 1
suMicro = 2

# Result Options - NFA Displacement Modes
dmUnitModalMass = 0
dmEngModalMass = 1

# Tool Options - Doubles
ipToolOptsElementTol = 0
ipToolOptsGeometryAccuracy = 1
ipToolOptsGeometryFeatureLength = 2

# Tool Options - Integers
ipToolOptsElementTolType = 0
ipToolOptsGeometryAccuracyType = 1
ipToolOptsGeometryFeatureType = 2
ipToolOptsZipMesh = 3
ipToolOptsNodeCoordinate = 4
ipToolOptsNodeAttributeKeep = 5
ipToolOptsAllowZeroLengthLinks = 6
ipToolOptsAllowZeroLengthBeams = 7
ipToolOptsAllowSameProperty = 8
ipToolOptsAllowSameGroup = 9
ipToolOptsSubdivideBeams = 10
ipToolOptsInterpSideAttachments = 11
ipToolOptsCompatibleTriangle = 12
ipToolOptsAdjustMidsideNodes = 13
ipToolOptsEvaluateFormulas = 14
ipToolOptsPlateAxisAlign = 15
ipToolOptsWedgeSubdivision = 16
ipToolOptsCopyMode = 17
ipToolOptsAutoCreateProperties = 18
ipToolOptsInsertMPLNodes = 19
ipToolOptsSubdivideDroopedCables = 20

# Tool Options - Mesh Zipping
zmAsNeeded = 0
zmOnSave = 1
zmOnRequest = 2

# Tool Options - Copy Mode
cmRoot = 0
cmSibling = 1

# Tool Options - Axis Alignment
paCentroid = 0
paCurvilinear = 1

# Tool Options - Wedge Subdivision
wsUseAB = 0
wsUseAC = 1

# Tool Options - Source Action
saLeave = 0
saErase = 1
saCopy = 2
saMove = 3

# Tool Options - Extrude Target - Node
etBeam2 = 0
etBeam3 = 1
etMasterSlaveLink = 2
etPinnedLink = 3
etRigidLink = 4
etShrinkLink = 5

# Tool Options - Scale by UCS - Scale About
saMiddle = 0
saOrigin = 1
saPoint = 2

# Tool Options - Extrude Target - Beam
etPlateQuad4 = 0
etPlateQuad8 = 1
etPlateQuad9 = 2

# Tool Options - Detach Elements - Connection Type
ctNone = 0
ctMasterSlaveLink = 1
ctBeam2 = 2

# Tool Options - Points and Lines - Target
plNode = 0
plBeam2 = 1
plBeam3 = 2

# Tool Options - Subdivide Target - Plate
stPlateTri3 = 0
stPlateTri6 = 1
stPlateQuad4 = 2
stPlateQuad8 = 3
stPlateQuad9 = 4
stPlateSource = 5
stPlateTri = 6
stPlateQuad = 7

# Tool Options - Subdivide Target - Brick
stBrickTetra4 = 0
stBrickTetra10 = 1
stBrickWedge6 = 2
stBrickWedge15 = 3
stBrickHexa8 = 4
stBrickHexa16 = 5
stBrickHexa20 = 6
stBrickSource = 7
stBrickTetra = 8
stBrickWedge = 9
stBrickHexa = 10

# Tool Options - Grade Type
gt1x2Grade = 0
gt1x2TriGrade = 1
gt1x3Grade = 2
gt2x3Grade = 3
gt2x3TriGrade = 4
gtQuarterQuadGrade = 5
gtQuarterCircleCut = 6
gtQuarterAnnulusCut = 7
gtFullQuarterCircleCut = 8
gtTriGrade2 = 9
gtTriGrade1 = 10
gtTriGrade3 = 11
gt2x4Grade = 12
gtBrickCornerGrade = 13
gtQuadTriGrade1 = 14
gtTriGrade5 = 15
gtQuadCutOut = 16
gtTriGrade4 = 17
gtFullQuarterCircleGrade = 18
gtQuadGradeTri = 19

# Tool Options - Beams on Edges
eeSplit = 0
eeIgnoreMid = 1
eeBeam3 = 2
geBeam2 = 0
geBeam3 = 1

# Tool Options - Create Entity UCS
puCylindrical = 0
puCartesian = 1

# Tool Options - Create Entity UCS
ulAtMin = 0
ulAtMax = 1
ulAtMean = 2

# Tools Options - Align Beam Offsets - Sections
ipCircularSection = 0
ipSquareSection = 1
ipCSection = 2
ipISection = 3
ipTSection = 4
ipLSection = 5
ipZSection = 6
ipBXSSection = 7
ipTrapezoidalSection = 8
ipTriangularSection = 9
ipCruciformSection = 10
ipUndefinedSection = 11

# Tools Options - Align Beam Offsets - Section Offsets
soNoChange = 0
soTopLeft = 1
soTopMid = 2
soTopRight = 3
soMidLeft = 4
soGeometricCenter = 5
soMidRight = 6
soBottomLeft = 7
soBottomMid = 8
soBottomRight = 9
soCentroid = 10
soShearCenter = 11

# Tools Options - Merge Line of Beams
mbStatic = 0
mbDynamic = 1

# Tool Options - Reinforcement Alignment
raLayer13 = 1
raLayer24 = 2

# Tool Options - Extrude by Line Direction
ldAuto = 0
ldReversed = 1

# Tool Options - Create Beams on Element Edges
beBasedOnModel = 0
beBasedOnSelected = 1

# Tool Options - ReactionMPL at specified origin
ocUseOrigin = -1
ocUseNodeAverage = 0

# Tool Options - Detach modes
dmDetachIndividual = 0
dmDetachAsCluster = 1
dmDetachGroups = 2

# Copy-Paste - Constants
poCasesInOrder = 0
poCasesMatchNames = 1
poPropertiesUsePropertyID = 0
poPropertiesMatchExisting = 1
poPropertiesCreateNew = 2
poLoadPathUseTemplateID = 0
poLoadPathCreateNew = 1

# Copy-Paste - Indexes
ipPasteCases = 0
ipPasteProperties = 1
ipPasteLoadPaths = 2
ipPasteAttributes = 3
ipPasteGroups = 4
ipPasteGlobals = 5
ipPasteTables = 6

# Insitu Parameters
ipGravityCaseIndex = 0
ipFreedomCaseIndex = 1
ipStageIndex = 2
ipUseExisting = 3
ipReplaceK0 = 4
ipMaxIterations = 5
ipAllowIterations = 6

# Insitu Warning Codes
wcInsituNoWarning = 0
wcInsituUnconverged = 1
wcInsituTensileStress = 2

# LSA Combinations Warning Codes
wcLSACombineNoWarning = 0
wcLSACombineInvalidSRA = 1

# Axis Definitions
axLocalX = 1
axLocalY = 2
axPrincipal1 = 1
axPrincipal2 = 2
axBeamPrincipal = 0
axBeamLocal = 1

# Beam Taper
btSymm = 0
btTop = 1
btBottom = 2

# Pre-load
plBeamPreTension = 0
plBeamPreStrain = 1
plPlatePreStress = 0
plPlatePreStrain = 1
plBrickPreStress = 0
plBrickPreStrain = 1

# Attachment Attribute
alRigid = 0
alFlexible = 1
alDirect = 2
alMoment = 0
alPinned = 1

# LTA Methods
ltWilson = 0
ltNewmark = 1

# Spectral
stResponse = 0
stPSD = 1

# Spectral Results Sign
rsAuto = 0
rsAbsolute = 1

# LTA
stFullSystem = 0
stSuperposition = 1

# Create Attachments - Brick Target
ktFreeFaces = 0
ktAllFaces = 1
ktInsideBricks = 2

# Transient Initial Conditions
icNone = 0
icAppliedVectors = 1
icNodalVelocity = 2
icFromFile = 3

# Transient and QuasiStatic Temperature
ttNodalTemp = 0
ttFromFile = 1

# Envelopes
etLimitEnvelopeAbs = 0
etLimitEnvelopeMin = 1
etLimitEnvelopeMax = 2
etCombEnvelopeMin = 0
etCombEnvelopeMax = 1
etFactEnvelopeMin = 0
etFactEnvelopeMax = 1
esCombEnvelopeOn = 0
esCombEnvelopeOff = 1
esCombEnvelopeCheck = 2
stExclusiveOR = 0
stExclusiveAND = 1

# Frequency Table Units
fuNone = 0
fuDispResponse = 1
fuVelResponse = 2
fuAccelResponse = 3
fuDispPSD = 4
fuVelPSD = 5
fuAccelPSD = 6
fuAccelResponseG = 7
fuAccelPSDG = 8

# Temp/Time Types
mtElastic = 0
mtPlastic = 1

# Material Hardening Types
htIsotropic = 0
htKinematic = 1
htTakeda = 2

# Spring-damper
ipSpringAxialStiff = 0
ipSpringLateralStiff = 1
ipSpringTorsionStiff = 2
ipSpringAxialDamp = 3
ipSpringLateralDamp = 4
ipSpringTorsionDamp = 5
ipSpringMass = 6

# Truss
ipTrussIncludeTorsion = 0

# Cable - Integers
ipCablePreStrainScalesMass = 0

# Cable - Doubles
ipCableDiameter = 0

# Cutoff Bar
ipCutoffTension = 0
ipCutoffCompression = 1

# Contact
cfElastic = 0
cfPlastic = 1
cyRectangular = 0
cyElliptical = 1

# Thermal data
ipThermalArea = 0
ipThermalMass = 1

# Ply Material - Integers
ipPlyWeaveType = 0
wtPlyUniDirectional = 0
wtPlyBiDirectional = 1
wtPlyTriDirectional = 2
wtPlyQuasiIsotropic = 3

# Ply Material - Doubles
ipPlyModulus1 = 0
ipPlyModulus2 = 1
ipPlyPoisson = 2
ipPlyShear12 = 3
ipPlyShear13 = 4
ipPlyShear23 = 5
ipPlyAlpha1 = 6
ipPlyAlpha2 = 7
ipPlyDensity = 8
ipPlyThickness = 9
ipPlyS1Tension = 10
ipPlyS2Tension = 11
ipPlyS1Compression = 12
ipPlyS2Compression = 13
ipPlySShear = 14
ipPlyE1Tension = 15
ipPlyE2Tension = 16
ipPlyE1Compression = 17
ipPlyE2Compression = 18
ipPlyEShear = 19
ipPlyInterLaminaShear = 20

# Laminate Material
ipLaminateViscosity = 0
ipLaminateDampingRatio = 1
ipLaminateConductivity1 = 2
ipLaminateConductivity2 = 3
ipLaminateSpecificHeat = 4
ipLaminateDensity = 5
ipLaminateAlphax = 6
ipLaminateAlphay = 7
ipLaminateAlphaxy = 8
ipLaminateBetax = 9
ipLaminateBetay = 10
ipLaminateBetaxy = 11
ipLaminateModulusx = 12
ipLaminateModulusy = 13
ipLaminateShearxy = 14
ipLaminatePoissonxy = 15
ipLaminatePoissonyx = 16
ipLaminateThickness = 17

# Laminate Plies
ipLaminatePlyAngle = 0
ipLaminatePlyThickness = 1

# Laminate Matrices
ipLaminateIgnoreCoupling = 0
ipLaminateAutoTransverseShear = 1

# Concrete Reinforcement Layouts - Integers
ipRCLayoutType = 0
ipRCColour13 = 1
ipRCColour24 = 2
ipRCCalcMethod = 3
ipRCConsiderMembrane = 4
ipRCAllowCompressionReo = 5
ipRCCode = 6
ipRCLimitConcreteStrain = 7
crRCSymmetric = 0
crRCAntiSymmetric = 1
crRCSimplified = 0
crRCElastoPlasticIter = 1

# Concrete Reinforcement Layouts - Doubles
ipRCDiam1 = 0
ipRCDiam2 = 1
ipRCDiam3 = 2
ipRCDiam4 = 3
ipRCCover1 = 4
ipRCCover2 = 5
ipRCSpacing1 = 6
ipRCSpacing2 = 7
ipRCSpacing3 = 8
ipRCSpacing4 = 9
ipRCConcreteModulus = 10
ipRCConcreteStrain = 11
ipRCConcreteStress = 12
ipRCConcreteAlpha = 13
ipRCConcreteGamma = 14
ipRCSteelModulus = 15
ipRCSteelStress = 16
ipRCSteelGamma = 17
ipRCSteelMinArea = 18
ipRCSteelPhi = 19

# Creep Hardening
ipCreepHardeningType = 0
ipCreepHardeningCyclic = 1
crHardeningTime = 0
crHardeningStrain = 1

# Hyperbolic Creep - Doubles
ipCreepHyberbolicAlpha = 0
ipCreepHyperbolicBeta = 1
ipCreepHyperbolicDelta = 2
ipCreepHyperbolicPhi = 3

# Hyperbolic Creep - Integers
ipCreepHyperbolicTimeTable = 0
ipCreepHyperbolicConstModulus = 1

# Visco-elastic Creep - Integers
ipCreepViscoTimeTable = 0
ipCreepViscoTempTable = 1

# Visco-elastic Creep - Doubles
ipCreepViscoDamper = 0
ipCreepViscoStiffness = 1

# Creep Concrete Functions
crCreepFunction = 0
crRelaxationFunction = 1

# Creep Shrinkage
crCreepShrinkageTable = 0
crCreepShrinkageFormula = 1
ipCreepShrinkageAlpha = 0
ipCreepShrinkageBeta = 1
ipCreepShrinkageDelta = 2
ipCreepShrinkageStrain = 3

# Creep Temperature - Integers
ipIncludeCreepTemperature = 0
ipIncludeRateTemperature = 1
ipIncludeShrinkageTemperature = 2

# Creep Temperature - Doubles
ipCreepCAAge = 0
ipCreepTRefAge = 1
ipCreepCCCreep = 2
ipCreepTRefCreep = 3
ipCreepCAShrink = 4
ipCreepTRefShrink = 5

# Cement Curing - Integers
ipCreepIncludeCuring = 0
ipCreepCuringTimeTable = 1
ipCreepCuringType = 2
crCementCuringRapid = 0
crCementCuringNormal = 1
crCementCuringSlow = 2

# Cement Curing - Doubles
ipCreepCuringCT = 0
ipCreepCuringTRef = 1
ipCreepCuringT0 = 2

# Stage Data
ipStageMorph = 0
ipStageMoveFixedNodes = 1
ipStageRotateClusters = 2
ipStageSetFluidLevel = 3

# Node Response Variables
rvNodeDisplacement = 0
rvNodeReaction = 1

# Beam Response Variables
ipBeamResponseSF1 = 0
ipBeamResponseSF2 = 1
ipBeamResponseAxial = 2
ipBeamResponseBM1 = 3
ipBeamResponseBM2 = 4
ipBeamResponseTorque = 5

# Plate Response Variables
rvPlateForce = 0
rvPlateMoment = 1

# Pipe Properties
ipPipeFlexibility = 0
ipPipeFluidDensity = 1
ipPipeOuterDiameter = 2
ipPipeThickness = 3

# Connection Properties
ipConnectionShear1 = 0
ipConnectionShear2 = 1
ipConnectionAxial = 2
ipConnectionBend1 = 3
ipConnectionBend2 = 4
ipConnectionTorque = 5

# Beam Materials
ipBeamModulus = 0
ipBeamShear = 1
ipBeamPoisson = 2
ipBeamDensity = 3
ipBeamAlpha = 4
ipBeamViscosity = 5
ipBeamDampingRatio = 6
ipBeamConductivity = 7
ipBeamSpecificHeat = 8

# Plate Isotropic Materials
ipPlateIsoModulus = 0
ipPlateIsoPoisson = 1
ipPlateIsoDensity = 2
ipPlateIsoAlpha = 3
ipPlateIsoViscosity = 4
ipPlateIsoDampingRatio = 5
ipPlateIsoConductivity = 6
ipPlateIsoSpecificHeat = 7

# Brick Isotropic Materials
ipBrickIsoModulus = 0
ipBrickIsoPoisson = 1
ipBrickIsoDensity = 2
ipBrickIsoAlpha = 3
ipBrickIsoViscosity = 4
ipBrickIsoDampingRatio = 5
ipBrickIsoConductivity = 6
ipBrickIsoSpecificHeat = 7

# Plate Orthotropic Materials
ipPlateOrthoModulus1 = 0
ipPlateOrthoModulus2 = 1
ipPlateOrthoModulus3 = 2
ipPlateOrthoShear12 = 3
ipPlateOrthoShear23 = 4
ipPlateOrthoShear31 = 5
ipPlateOrthoPoisson12 = 6
ipPlateOrthoPoisson23 = 7
ipPlateOrthoPoisson31 = 8
ipPlateOrthoDensity = 9
ipPlateOrthoAlpha1 = 10
ipPlateOrthoAlpha2 = 11
ipPlateOrthoAlpha3 = 12
ipPlateOrthoViscosity = 13
ipPlateOrthoDampingRatio = 14
ipPlateOrthoConductivity1 = 15
ipPlateOrthoConductivity2 = 16
ipPlateOrthoSpecificHeat = 17

# Brick Orthotropic Materials
ipBrickOrthoModulus1 = 0
ipBrickOrthoModulus2 = 1
ipBrickOrthoModulus3 = 2
ipBrickOrthoShear12 = 3
ipBrickOrthoShear23 = 4
ipBrickOrthoShear31 = 5
ipBrickOrthoPoisson12 = 6
ipBrickOrthoPoisson23 = 7
ipBrickOrthoPoisson31 = 8
ipBrickOrthoDensity = 9
ipBrickOrthoAlpha1 = 10
ipBrickOrthoAlpha2 = 11
ipBrickOrthoAlpha3 = 12
ipBrickOrthoViscosity = 13
ipBrickOrthoDampingRatio = 14
ipBrickOrthoConductivity1 = 15
ipBrickOrthoConductivity2 = 16
ipBrickOrthoConductivity3 = 17
ipBrickOrthoSpecificHeat = 18

# Plate Anisotropic Materials

# 0..9 ansi matrix
ipPlateAnisoTransShear1 = 10
ipPlateAnisoTransShear2 = 11
ipPlateAnisoTransShear3 = 12
ipPlateAnisoDensity = 13
ipPlateAnisoAlpha1 = 14
ipPlateAnisoAlpha2 = 15
ipPlateAnisoAlpha3 = 16
ipPlateAnisoAlpha12 = 17
ipPlateAnisoViscosity = 18
ipPlateAnisoDampingRatio = 19
ipPlateAnisoConductivity1 = 20
ipPlateAnisoConductivity2 = 21
ipPlateAnisoSpecificHeat = 22

# Plate User Defined Materials

# 0..20 user matrix
ipPlateUserTransShearxz = 21
ipPlateUserTransShearyz = 22
ipPlateUserTransShearcz = 23
ipPlateUserDensity = 24
ipPlateUserAlphax = 25
ipPlateUserAlphay = 26
ipPlateUserAlphaxy = 27
ipPlateUserBetax = 28
ipPlateUserBetay = 29
ipPlateUserBetaxy = 30
ipPlateUserViscosity = 31
ipPlateUserDampingRatio = 32
ipPlateUserConductivity1 = 33
ipPlateUserConductivity2 = 34
ipPlateUserSpecificHeat = 35

# Brick Anisotropic Materials

# 0..20 user matrix
ipBrickAnisoDensity = 21
ipBrickAnisoAlpha1 = 22
ipBrickAnisoAlpha2 = 23
ipBrickAnisoAlpha3 = 24
ipBrickAnisoAlpha12 = 25
ipBrickAnisoAlpha23 = 26
ipBrickAnisoAlpha31 = 27
ipBrickAnisoViscosity = 28
ipBrickAnisoDampingRatio = 29
ipBrickAnisoConductivity1 = 30
ipBrickAnisoConductivity2 = 31
ipBrickAnisoConductivity3 = 32
ipBrickAnisoSpecificHeat = 33

# Duncan-Chang Soil Materials - Integers
ipSoilDCUsePoisson = 0
ipSoilDCSetLevel = 1

# Duncan-Chang Soil Materials - Doubles
ipSoilDCModulusK = 0
ipSoilDCModulusKUR = 1
ipSoilDCModulusN = 2
ipSoilDCPoisson = 3
ipSoilDCBulkK = 4
ipSoilDCBulkM = 5
ipSoilDCFrictionAngle = 6
ipSoilDCDeltaAngle = 7
ipSoilDCCohesion = 8
ipSoilDCFailureRatio = 9
ipSoilDCFailureMod = 10
ipSoilDCReferenceP = 11
ipSoilDCDensity = 12
ipSoilDCHorizontalRatio = 13
ipSoilDCER = 14
ipSoilDCConductivity = 15
ipSoilDCSpecificHeat = 16
ipSoilDCFluidLevel = 17
ipSoilDCViscosity = 18
ipSoilDCDampingRatio = 19

# Cam-Clay Soil Materials - Integers
ipSoilCCUsePoisson = 0
ipSoilCCDrainedState = 1
ipSoilCCUseOCR = 2
ipSoilCCSetLevel = 3

# Cam-Clay Soil Materials - Doubles
ipSoilCCCriticalStateLine = 0
ipSoilCCConsolidationLine = 1
ipSoilCCSwellingLine = 2
ipSoilCCDensity = 3
ipSoilCCPoisson = 4
ipSoilCCModulusG = 5
ipSoilCCModulusB = 6
ipSoilCCHorizontalRatio = 7
ipSoilCCER = 8
ipSoilCCPR = 9
ipSoilCCPC0 = 10
ipSoilCCOCR = 11
ipSoilCCConductivity = 12
ipSoilCCSpecificHeat = 13
ipSoilCCFluidLevel = 14
ipSoilCCViscosity = 15
ipSoilCCDampingRatio = 16

# Mohr-Coulomb Soil Materials - Integers
ipSoilMCSetLevel = 0

# Mohr-Coulomb Soil Materials - Doubles
ipSoilMCModulus = 0
ipSoilMCPoisson = 1
ipSoilMCDensity = 2
ipSoilMCHorizontalRatio = 3
ipSoilMCER = 4
ipSoilMCConductivity = 5
ipSoilMCSpecificHeat = 6
ipSoilMCFluidLevel = 7
ipSoilMCViscosity = 8
ipSoilMCDampingRatio = 9
ipSoilMCCohesion = 10
ipSoilMCFrictionAngle = 11

# Drucker-Prager Soil Materials - Integers
ipSoilDPSetLevel = 0

# Drucker-Prager Soil Materials - Doubles
ipSoilDPModulus = 0
ipSoilDPPoisson = 1
ipSoilDPDensity = 2
ipSoilDPHorizontalRatio = 3
ipSoilDPER = 4
ipSoilDPConductivity = 5
ipSoilDPSpecificHeat = 6
ipSoilDPFluidLevel = 7
ipSoilDPViscosity = 8
ipSoilDPDampingRatio = 9
ipSoilDPCohesion = 10
ipSoilDPFrictionAngle = 11

# Linear Elastic Soil Materials - Integers
ipSoilLSSetLevel = 0

# Linear Elastic Soil Materials - Doubles
ipSoilLSModulus = 0
ipSoilLSPoisson = 1
ipSoilLSDensity = 2
ipSoilLSHorizontalRatio = 3
ipSoilLSER = 4
ipSoilLSConductivity = 5
ipSoilLSSpecificHeat = 6
ipSoilLSFluidLevel = 7
ipSoilLSViscosity = 8
ipSoilLSDampingRatio = 9

# Fluid Materials
ipFluidModulus = 0
ipFluidPenaltyParam = 1
ipFluidDensity = 2
ipFluidAlpha = 3
ipFluidViscosity = 4
ipFluidDampingRatio = 5
ipFluidConductivity = 6
ipFluidSpecificHeat = 7

# Mohr-Coulomb, Drucker-Prager
ipFrictionAngle = 0
ipCohesion = 1

# Rubber Materials
ipRubberBulk = 0
ipRubberDensity = 1
ipRubberAlpha = 2
ipRubberViscosity = 3
ipRubberDampingRatio = 4
ipRubberConductivity = 5
ipRubberSpecificHeat = 6
ipRubberConstC1 = 7

# Load Case Types
ltLoadCase = 0
ltSpectralCase = 2

# Beam Property
ipBeamPropBeamType = 0
ipBeamPropUsePoisson = 1
ipBeamPropSectionType = 2
ipBeamPropMirrorType = 3
ipBeamPropCompatibleTwist = 4

# Element Axis Types
axUCS = 0
axLocal = 1

# Load Path Template - Integers
ipLPTColour = 0
ipLPTNumLanes = 1
ipLPTMultiLaneType = 2
ipLPTTransitionLoad = 3
lpAllSameFactors = 0
lpAllDifferentFactors = 1

# Load Path Template - Doubles
ipLPTTolerance = 0
ipLPTMinLaneWidth = 1

# Load Path Template Vehicle - Integers
ipLPTVehicleInstance = 0
ipLPTVehicleDirection = 1
lpVehicleSingleLane = 0
lpVehicleDoubleLane = 1
lpVehicleForward = 0
lpVehicleBackward = 1

# Load Path Template Vehicle - Doubles
ipLPTVehicleVelocity = 0
ipLPTVehicleStartTime = 1

# Load Path Template Forces - Integers
ipLPTMobility = 0
ipLPTAxisSystem = 1
ipLPTAdjacency = 2
ipLPTCentrifugal = 3
lpPointForceMobilityGrouped = 0
lpPointForceMobilityFloating = 1
lpDistrForceMobilityGrouped = 0
lpDistrForceMobilityLeading = 1
lpDistrForceMobilityTrailing = 2
lpDistrForceMobilityFullLength = 3
lpDistrForceMobilityFloating = 4
lpAxisLocal = 0
lpAxisGlobal = 1

# Load Path Templates - Integers
ipLPTLimitK1 = 0
ipLPTLengthUnit = 1
ipLPTForceUnit = 2

# Load Path Templates - Doubles
ipLPTMinK1 = 0
ipLPTMaxK1 = 1

# Combined Result Files
rfCombFactors = 0
rfCombSRSS = 1

# Load Path
ipLoadPathCase = 0
ipLoadPathTemplate = 1
ipLoadPathShape = 2
ipLoadPathSurface = 3
ipLoadPathTarget = 4
ipLoadPathDivisions = 5
ipLoadPathSet = 6
lpShapeStraight = 0
lpShapeCurved = 1
lpShapeQuadratic = 2
lpSurfaceFlat = 0
lpSurfaceCurved = 1
lpAnyEntity = 0
lpEntitySet = 1
lpBeamElement = 2
lpPlateElement = 3
lpBrickElement = 4

# Animation
ipAniCase = 0
ipNumFrames = 1
ipAniWidth = 2
ipAniHeight = 3
ipAniType = 4
afAniSAF = 0
afAniEXE = 1
afAniAVI = 2

# Custom Result Files - NODEDISP, NODEREACT
ipNodeResFileDX = 0
ipNodeResFileDY = 1
ipNodeResFileDZ = 2
ipNodeResFileRX = 3
ipNodeResFileRY = 4
ipNodeResFileRZ = 5

# Custom Result Files - NODETEMP, NODEFLUX
ipNodeResTemp = 0

# Custom Result Files - BEAMFORCE
ipBeamResFileSF1 = 0
ipBeamResFileSF2 = 1
ipBeamResFileAxial = 2
ipBeamResFileBM1 = 3
ipBeamResFileBM2 = 4
ipBeamResFileTorque = 5
kBeamResFileForceSize = 6

# Custom Result Files - BEAMSTRAIN
ipBeamResFileAxialStrain = 2
ipBeamResFileCurvature1 = 3
ipBeamResFileCurvature2 = 4
ipBeamResFileTwist = 5
kBeamResFileStrainSize = 6

# Custom Result Files - BEAMNODEREACT
ipBeamResFileFX = 0
ipBeamResFileFY = 1
ipBeamResFileFZ = 2
ipBeamResFileMX = 3
ipBeamResFileMY = 4
ipBeamResFileMZ = 5
kBeamResFileReactSize = 6

# Custom Result Files - BEAMFLUX
ipBeamResFileF = 0
ipBeamResFileG = 1
kBeamResFileFluxSize = 2

# Custom Result Files - PLATESTRESS for PlateShell - Local system
ipPlateShellResFileNxx = 0
ipPlateShellResFileNyy = 1
ipPlateShellResFileNxy = 2
ipPlateShellResFileMxx = 3
ipPlateShellResFileMyy = 4
ipPlateShellResFileMxy = 5
ipPlateShellResFileQxz = 6
ipPlateShellResFileQyz = 7
ipPlateShellResFileZMinusSxx = 8
ipPlateShellResFileZMinusSyy = 9
ipPlateShellResFileZMinusSxy = 10
ipPlateShellResFileMidPlaneSxx = 11
ipPlateShellResFileMidPlaneSyy = 12
ipPlateShellResFileMidPlaneSxy = 13
ipPlateShellResFileZPlusSxx = 14
ipPlateShellResFileZPlusSyy = 15
ipPlateShellResFileZPlusSxy = 16
kPlateShellResFileStressSize = 17

# Custom Result Files - PLATESTRAIN for PlateShell - Local system
ipPlateShellResFileExx = 0
ipPlateShellResFileEyy = 1
ipPlateShellResFileExy = 2
ipPlateShellResFileEzz = 3
ipPlateShellResFileKxx = 4
ipPlateShellResFileKyy = 5
ipPlateShellResFileKxy = 6
ipPlateShellResFileTxz = 7
ipPlateShellResFileTyz = 8
ipPlateShellResFileStoredE = 9
ipPlateShellResFileSpentE = 10
kPlateShellResFileStrainSize = 11

# Custom Result Files - PLATESTRESS for 3D Membrane - Local system
ipPlateMembraneResFileSXX = 0
ipPlateMembraneResFileSYY = 1
ipPlateMembraneResFileSXY = 2
kPlateMembraneResFileStressSize = 3

# Custom Result Files - PLATESTRAIN for 3D Membrane - Local system
ipPlateMembraneResFileExx = 0
ipPlateMembraneResFileEyy = 1
ipPlateMembraneResFileExy = 2
ipPlateMembraneResFileEzz = 3
ipPlateMembraneResFileStoredE = 4
ipPlateMembraneResFileSpentE = 5
kPlateMembraneResFileStrainSize = 6

# Custom Result Files - PLATESTRESS for 2D Plates - Global system
ipPlate2DResFileSXX = 0
ipPlate2DResFileSYY = 1
ipPlate2DResFileSXY = 2
ipPlate2DResFileSZZ = 3
kPlate2DResFileStressSize = 4

# Custom Result Files - PLATESTRAIN for 2D Plates - Global system
ipPlate2DResFileEXX = 0
ipPlate2DResFileEYY = 1
ipPlate2DResFileEXY = 2
ipPlate2DResFileEZZ = 3
ipPlate2DResFileStoredE = 4
ipPlate2DResFileSpentE = 5
kPlate2DResFileStrainSize = 6

# Custom Result Files - PLATESTRESS for Axi Plates - Axisymmetric system
ipPlateAxiResFileSRR = 0
ipPlateAxiResFileSZZ = 1
ipPlateAxiResFileSTT = 2
ipPlateAxiResFileSRT = 3
kPlateAxiResFileStressSize = 4

# Custom Result Files - PLATESTRAIN for Axi Plates - Axisymmetric system
ipPlateAxiResFileERR = 0
ipPlateAxiResFileEZZ = 1
ipPlateAxiResFileETT = 2
ipPlateAxiResFileERT = 3
ipPlateAxiResFileStoredE = 4
ipPlateAxiResFileSpentE = 5
kPlateAxiResFileStrainSize = 6

# Custom Result Files - PLATESTRESS for Shear Panel - Local system
ipPlateShearPanelResFileNxy = 0
kPlateShearPanelResFileStressSize = 1

# Custom Result Files - PLATESTRAIN for Shear Panel - Local system
ipPlateShearPanelResFileExy = 0
ipPlateShearPanelResFileStoredE = 1
ipPlateShearPanelResFileSpentE = 2
kPlateShearPanelResFileStrainSize = 3

# Custom Result Files - PLATENODEREACT
ipPlateResFileFX = 0
ipPlateResFileFY = 1
ipPlateResFileFZ = 2
ipPlateResFileMX = 3
ipPlateResFileMY = 4
ipPlateResFileMZ = 5
kPlateResFileReactSize = 6

# Custom Result Files - PLATEFLUX
ipPlateResFileFxx = 0
ipPlateResFileFyy = 1
ipPlateResFileGxx = 2
ipPlateResFileGyy = 3
kPlateResFileFluxSize = 4

# Custom Result Files - BRICKSTRESS
ipBrickResFileSXX = 0
ipBrickResFileSYY = 1
ipBrickResFileSZZ = 2
ipBrickResFileSXY = 3
ipBrickResFileSYZ = 4
ipBrickResFileSZX = 5
kBrickResFileStressSize = 6

# Custom Result Files - BRICKSTRAIN
ipBrickResFileExx = 0
ipBrickResFileEyy = 1
ipBrickResFileEzz = 2
ipBrickResFileExy = 3
ipBrickResFileEyz = 4
ipBrickResFileEzx = 5
ipBrickResFileStoredE = 6
ipBrickResFileSpentE = 7
kBrickResFileStrainSize = 8

# Custom Result Files - BRICKNODEREACT
ipBrickResFileFX = 0
ipBrickResFileFY = 1
ipBrickResFileFZ = 2
kBrickResFileReactSize = 3

# Custom Result Files - BRICKFLUX
ipBrickResFileFXX = 0
ipBrickResFileFYY = 1
ipBrickResFileFZZ = 2
ipBrickResFileGXX = 3
ipBrickResFileGYY = 4
ipBrickResFileGZZ = 5
kBrickResFileFluxSize = 6

# Plate Edge Attachment Direction
adPlanar = 0
adMinusZ = 1
adPlusZ = 2

# Beam Side Direction
adMinus1 = 0
adPlus1 = 1
adMinus2 = 2
adPlus2 = 3

# GLOBAL INTEGERS
ivTessellationsFailed = 1
ivSeamsAdded = 2
ivIntersectionsFound = 3
ivPlateEdgesAssigned = 4
ivPlateEdgesNotFullyAssigned = 5
ivAttachmentsCreated = 6
ivAttachmentsFailed = 7
ivNodesCreated = 8
ivNodesDeleted = 9
ivNodesMoved = 10
ivBeamsChanged = 11
ivBeamsCollapsed = 12
ivBeamsCreated = 13
ivBeamsDeleted = 14
ivBeamsFailed = 15
ivBeamsMoved = 16
ivBeamsSplit = 17
ivBeamsSubdivided = 18
ivPlatesChanged = 19
ivPlatesCollapsed = 20
ivPlatesCreated = 21
ivPlatesDeleted = 22
ivPlatesFailed = 23
ivPlatesGraded = 24
ivPlatesMoved = 25
ivPlatesSplit = 26
ivPlatesSubdivided = 27
ivBricksChanged = 28
ivBricksCollapsed = 29
ivBricksCreated = 30
ivBricksDeleted = 31
ivBricksFailed = 32
ivBricksGraded = 33
ivBricksMoved = 34
ivBricksSplit = 35
ivBricksSubdivided = 36
ivLinksChanged = 37
ivLinksCollapsed = 38
ivLinksCreated = 39
ivLinksDeleted = 40
ivLinksMoved = 41
ivLoadPathsChanged = 42
ivLoadPathsCreated = 43
ivLoadPathsMoved = 44
ivFacesChanged = 45
ivFacesCreated = 46
ivFacesDeleted = 47
ivFacesFailed = 48
ivFacesMoved = 49
ivEdgesMorphed = 50
ivEdgesSubdivided = 51
ivLoopsDeleted = 52
ivAttributesApplied = 53
ivUCSCreated = 54
ivPatchPlatesCreated = 55
ivLoadCasesCreated = 56
ivFilletsCreated = 57
ivFilletsFailed = 58
ivLoftSeriesFound = 59
ivDuplicateBeamsDeleted = 60
ivDuplicatePlatesDeleted = 61
ivDuplicateBricksDeleted = 62
ivDuplicateLinksDeleted = 63
ivStringGroupsPacked = 64
ivClipboardNodes = 65
ivClipboardBeams = 66
ivClipboardPlates = 67
ivClipboardBricks = 68
ivClipboardLinks = 69
ivClipboardLoadPaths = 70
ivClipboardFaces = 71
ivClipboardVertices = 72
ivFacesMeshed = 73
ivFacesPartiallyMeshed = 74
ivFacesNotMeshed = 75
ivSolverTerminationCode = 76
ivSolidsMeshed = 77
ivSolidsPartiallyMeshed = 78
ivSolidsNotMeshed = 79

# GLOBAL LOGICALS
lvFormulaParseError = 1

# GLOBAL STRINGS
svInfluenceCombinationLog = 1

c_char = ctypes.c_char
c_char_p = ctypes.c_char_p
c_bool = ctypes.c_bool
c_long = ctypes.c_long
c_double = ctypes.c_double
HWND = ctypes.wintypes.HWND
create_string_buffer = ctypes.create_string_buffer

St7SetLicenceOptions = _ST7API.St7SetLicenceOptions
St7SetLicenceOptions.argtypes = [c_long, c_long, c_long]
St7GetLicenceOptions = _ST7API.St7GetLicenceOptions
St7GetLicenceOptions.argtypes = [ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7Init = _ST7API.St7Init
St7Init.argtypes = []
St7Release = _ST7API.St7Release
St7Release.argtypes = []
St7Version = _ST7API.St7Version
St7Version.argtypes = [ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7BuildString = _ST7API.St7BuildString
St7BuildString.argtypes = [c_char_p, c_long]
St7GetListSeparatorCode = _ST7API.St7GetListSeparatorCode
St7GetListSeparatorCode.argtypes = [ctypes.POINTER(c_long)]
St7GetDecimalSeparatorCode = _ST7API.St7GetDecimalSeparatorCode
St7GetDecimalSeparatorCode.argtypes = [ctypes.POINTER(c_long)]
St7SetIconSize = _ST7API.St7SetIconSize
St7SetIconSize.argtypes = [c_long]
St7GetIconSize = _ST7API.St7GetIconSize
St7GetIconSize.argtypes = [ctypes.POINTER(c_long)]
St7FileVersion = _ST7API.St7FileVersion
St7FileVersion.arSsgtypes = [c_char_p, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7OpenFile = _ST7API.St7OpenFile
St7OpenFile.argtypes = [c_long, c_char_p, c_char_p]
St7OpenFileReadOnly = _ST7API.St7OpenFileReadOnly
St7OpenFileReadOnly.argtypes = [c_long, c_char_p, c_char_p]
St7CloseFile = _ST7API.St7CloseFile
St7CloseFile.argtypes = [c_long]
St7NewFile = _ST7API.St7NewFile
St7NewFile.argtypes = [c_long, c_char_p, c_char_p]
St7SaveFile = _ST7API.St7SaveFile
St7SaveFile.argtypes = [c_long]
St7SaveFileCopy = _ST7API.St7SaveFileCopy
St7SaveFileCopy.argtypes = [c_long, c_char_p]
St7SaveViewOnlyCopy = _ST7API.St7SaveViewOnlyCopy
St7SaveViewOnlyCopy.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long)]
St7SaveDeformedCopy = _ST7API.St7SaveDeformedCopy
St7SaveDeformedCopy.argtypes = [c_long, c_char_p, c_long, c_double, c_long]
St7SaveSubModel = _ST7API.St7SaveSubModel
St7SaveSubModel.argtypes = [c_long, c_char_p]
St7ValidateResultFile = _ST7API.St7ValidateResultFile
St7ValidateResultFile.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetResultFileOpenFlag = _ST7API.St7SetResultFileOpenFlag
St7SetResultFileOpenFlag.argtypes = [c_long, c_long, c_bool]
St7GetResultFileOpenFlag = _ST7API.St7GetResultFileOpenFlag
St7GetResultFileOpenFlag.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetNFAFileOpenMinMass = _ST7API.St7SetNFAFileOpenMinMass
St7SetNFAFileOpenMinMass.argtypes = [c_long, c_double]
St7GetNFAFileOpenMinMass = _ST7API.St7GetNFAFileOpenMinMass
St7GetNFAFileOpenMinMass.argtypes = [c_long, ctypes.POINTER(c_double)]
St7OpenResultFile = _ST7API.St7OpenResultFile
St7OpenResultFile.argtypes = [c_long, c_char_p, c_char_p, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GenerateLSACombinations = _ST7API.St7GenerateLSACombinations
St7GenerateLSACombinations.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GenerateEnvelopes = _ST7API.St7GenerateEnvelopes
St7GenerateEnvelopes.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7CloseResultFile = _ST7API.St7CloseResultFile
St7CloseResultFile.argtypes = [c_long]
St7SetDisplayOptionsPath = _ST7API.St7SetDisplayOptionsPath
St7SetDisplayOptionsPath.argtypes = [c_char_p]
St7GetDisplayOptionsPath = _ST7API.St7GetDisplayOptionsPath
St7GetDisplayOptionsPath.argtypes = [c_char_p, c_long]
St7SetLibraryPath = _ST7API.St7SetLibraryPath
St7SetLibraryPath.argtypes = [c_char_p]
St7GetLibraryPath = _ST7API.St7GetLibraryPath
St7GetLibraryPath.argtypes = [c_char_p, c_long]
St7GetPath = _ST7API.St7GetPath
St7GetPath.argtypes = [c_char_p, c_long]
St7GetLastError = _ST7API.St7GetLastError
St7GetLastError.argtypes = []
St7GetAPIErrorString = _ST7API.St7GetAPIErrorString
St7GetAPIErrorString.argtypes = [c_long, c_char_p, c_long]
St7GetSolverErrorString = _ST7API.St7GetSolverErrorString
St7GetSolverErrorString.argtypes = [c_long, c_char_p, c_long]
St7TransformToUCS = _ST7API.St7TransformToUCS
St7TransformToUCS.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7TransformToXYZ = _ST7API.St7TransformToXYZ
St7TransformToXYZ.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7VectorTransformToUCS = _ST7API.St7VectorTransformToUCS
St7VectorTransformToUCS.argtypes = [c_long, c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7VectorTransformToXYZ = _ST7API.St7VectorTransformToXYZ
St7VectorTransformToXYZ.argtypes = [c_long, c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7GetPlateUV = _ST7API.St7GetPlateUV
St7GetPlateUV.argtypes = [c_long, c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7GetBrickUVW = _ST7API.St7GetBrickUVW
St7GetBrickUVW.argtypes = [c_long, c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7GetNumElementResultGaussPoints = _ST7API.St7GetNumElementResultGaussPoints
St7GetNumElementResultGaussPoints.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7ConvertElementResultNodeToGaussPoint = _ST7API.St7ConvertElementResultNodeToGaussPoint
St7ConvertElementResultNodeToGaussPoint.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7PlateHullVolume = _ST7API.St7PlateHullVolume
St7PlateHullVolume.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetEntitySelectState = _ST7API.St7SetEntitySelectState
St7SetEntitySelectState.argtypes = [c_long, c_long, c_long, c_long, c_bool]
St7GetEntitySelectState = _ST7API.St7GetEntitySelectState
St7GetEntitySelectState.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7GetEntitySelectCount = _ST7API.St7GetEntitySelectCount
St7GetEntitySelectCount.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetAllEntitySelectState = _ST7API.St7SetAllEntitySelectState
St7SetAllEntitySelectState.argtypes = [c_long, c_long, c_bool]
St7SetEntitySelectStateByProperty = _ST7API.St7SetEntitySelectStateByProperty
St7SetEntitySelectStateByProperty.argtypes = [c_long, c_long, c_long, c_bool]
St7SetEntitySelectStateByGroup = _ST7API.St7SetEntitySelectStateByGroup
St7SetEntitySelectStateByGroup.argtypes = [c_long, c_long, c_long, c_bool]
St7SetEntitySelectStateByEntitySet = _ST7API.St7SetEntitySelectStateByEntitySet
St7SetEntitySelectStateByEntitySet.argtypes = [c_long, c_long, c_long, c_bool]
St7SetBrickSelectState = _ST7API.St7SetBrickSelectState
St7SetBrickSelectState.argtypes = [c_long, c_long, c_long, c_long, c_bool]
St7GetBrickSelectState = _ST7API.St7GetBrickSelectState
St7GetBrickSelectState.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7SetModelWindowRefreshMode = _ST7API.St7SetModelWindowRefreshMode
St7SetModelWindowRefreshMode.argtypes = [c_long, c_bool]
St7CreateModelWindow = _ST7API.St7CreateModelWindow
St7CreateModelWindow.argtypes = [c_long]
St7DestroyModelWindow = _ST7API.St7DestroyModelWindow
St7DestroyModelWindow.argtypes = [c_long]
St7GetModelWindowState = _ST7API.St7GetModelWindowState
St7GetModelWindowState.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetModelWindowHandle = _ST7API.St7GetModelWindowHandle
St7GetModelWindowHandle.argtypes = [c_long, ctypes.POINTER(HWND)]
St7SetModelWindowParent = _ST7API.St7SetModelWindowParent
St7SetModelWindowParent.argtypes = [c_long, HWND]
St7GetModelWindowParent = _ST7API.St7GetModelWindowParent
St7GetModelWindowParent.argtypes = [c_long, ctypes.POINTER(HWND)]
St7ShowModelWindow = _ST7API.St7ShowModelWindow
St7ShowModelWindow.argtypes = [c_long]
St7HideModelWindow = _ST7API.St7HideModelWindow
St7HideModelWindow.argtypes = [c_long]
St7ShowWindowCombos = _ST7API.St7ShowWindowCombos
St7ShowWindowCombos.argtypes = [c_long]
St7HideWindowCombos = _ST7API.St7HideWindowCombos
St7HideWindowCombos.argtypes = [c_long]
St7ShowWindowEntityPanel = _ST7API.St7ShowWindowEntityPanel
St7ShowWindowEntityPanel.argtypes = [c_long]
St7HideWindowEntityPanel = _ST7API.St7HideWindowEntityPanel
St7HideWindowEntityPanel.argtypes = [c_long]
St7ShowWindowStatusBar = _ST7API.St7ShowWindowStatusBar
St7ShowWindowStatusBar.argtypes = [c_long]
St7HideWindowStatusBar = _ST7API.St7HideWindowStatusBar
St7HideWindowStatusBar.argtypes = [c_long]
St7EnableWindowStatusBar = _ST7API.St7EnableWindowStatusBar
St7EnableWindowStatusBar.argtypes = [c_long]
St7DisableWindowStatusBar = _ST7API.St7DisableWindowStatusBar
St7DisableWindowStatusBar.argtypes = [c_long]
St7SetWindowStatusBarRefreshMode = _ST7API.St7SetWindowStatusBarRefreshMode
St7SetWindowStatusBarRefreshMode.argtypes = [c_long, c_bool]
St7EnableWindowEntityInspector = _ST7API.St7EnableWindowEntityInspector
St7EnableWindowEntityInspector.argtypes = [c_long]
St7DisableWindowEntityInspector = _ST7API.St7DisableWindowEntityInspector
St7DisableWindowEntityInspector.argtypes = [c_long]
St7ShowWindowSelectionToolbar = _ST7API.St7ShowWindowSelectionToolbar
St7ShowWindowSelectionToolbar.argtypes = [c_long]
St7HideWindowSelectionToolbar = _ST7API.St7HideWindowSelectionToolbar
St7HideWindowSelectionToolbar.argtypes = [c_long]
St7ShowWindowCaption = _ST7API.St7ShowWindowCaption
St7ShowWindowCaption.argtypes = [c_long]
St7HideWindowCaption = _ST7API.St7HideWindowCaption
St7HideWindowCaption.argtypes = [c_long]
St7ShowWindowViewToolbar = _ST7API.St7ShowWindowViewToolbar
St7ShowWindowViewToolbar.argtypes = [c_long]
St7HideWindowViewToolbar = _ST7API.St7HideWindowViewToolbar
St7HideWindowViewToolbar.argtypes = [c_long]
St7ShowWindowResultsToolbar = _ST7API.St7ShowWindowResultsToolbar
St7ShowWindowResultsToolbar.argtypes = [c_long]
St7HideWindowResultsToolbar = _ST7API.St7HideWindowResultsToolbar
St7HideWindowResultsToolbar.argtypes = [c_long]
St7ShowWindowShowHideToolbar = _ST7API.St7ShowWindowShowHideToolbar
St7ShowWindowShowHideToolbar.argtypes = [c_long]
St7HideWindowShowHideToolbar = _ST7API.St7HideWindowShowHideToolbar
St7HideWindowShowHideToolbar.argtypes = [c_long]
St7EnableWindowResize = _ST7API.St7EnableWindowResize
St7EnableWindowResize.argtypes = [c_long]
St7DisableWindowResize = _ST7API.St7DisableWindowResize
St7DisableWindowResize.argtypes = [c_long]
St7EnableWindowViewChanges = _ST7API.St7EnableWindowViewChanges
St7EnableWindowViewChanges.argtypes = [c_long]
St7DisableWindowViewChanges = _ST7API.St7DisableWindowViewChanges
St7DisableWindowViewChanges.argtypes = [c_long]
St7ClearModelWindow = _ST7API.St7ClearModelWindow
St7ClearModelWindow.argtypes = [c_long]
St7RedrawModel = _ST7API.St7RedrawModel
St7RedrawModel.argtypes = [c_long, c_bool]
St7RotateModel = _ST7API.St7RotateModel
St7RotateModel.argtypes = [c_long, c_double, c_double, c_double]
St7ShowEntity = _ST7API.St7ShowEntity
St7ShowEntity.argtypes = [c_long, c_long]
St7HideEntity = _ST7API.St7HideEntity
St7HideEntity.argtypes = [c_long, c_long]
St7GetEntityVisibility = _ST7API.St7GetEntityVisibility
St7GetEntityVisibility.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7ShowPointAttributes = _ST7API.St7ShowPointAttributes
St7ShowPointAttributes.argtypes = [c_long]
St7HidePointAttributes = _ST7API.St7HidePointAttributes
St7HidePointAttributes.argtypes = [c_long]
St7GetPointAttributesVisibility = _ST7API.St7GetPointAttributesVisibility
St7GetPointAttributesVisibility.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7ShowEntityAttributes = _ST7API.St7ShowEntityAttributes
St7ShowEntityAttributes.argtypes = [c_long]
St7HideEntityAttributes = _ST7API.St7HideEntityAttributes
St7HideEntityAttributes.argtypes = [c_long]
St7GetEntityAttributesVisibility = _ST7API.St7GetEntityAttributesVisibility
St7GetEntityAttributesVisibility.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7PositionModelWindow = _ST7API.St7PositionModelWindow
St7PositionModelWindow.argtypes = [c_long, c_long, c_long, c_long, c_long]
St7GetModelWindowPosition = _ST7API.St7GetModelWindowPosition
St7GetModelWindowPosition.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetDrawAreaSize = _ST7API.St7GetDrawAreaSize
St7GetDrawAreaSize.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetDrawAreaPosition = _ST7API.St7GetDrawAreaPosition
St7GetDrawAreaPosition.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7ShowProperty = _ST7API.St7ShowProperty
St7ShowProperty.argtypes = [c_long, c_long, c_long]
St7HideProperty = _ST7API.St7HideProperty
St7HideProperty.argtypes = [c_long, c_long, c_long]
St7GetPropertyVisibility = _ST7API.St7GetPropertyVisibility
St7GetPropertyVisibility.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7ShowGroup = _ST7API.St7ShowGroup
St7ShowGroup.argtypes = [c_long, c_long]
St7HideGroup = _ST7API.St7HideGroup
St7HideGroup.argtypes = [c_long, c_long]
St7GetGroupVisibility = _ST7API.St7GetGroupVisibility
St7GetGroupVisibility.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetAllEntitiesOn = _ST7API.St7SetAllEntitiesOn
St7SetAllEntitiesOn.argtypes = [c_long]
St7GetEntityNumVisibility = _ST7API.St7GetEntityNumVisibility
St7GetEntityNumVisibility.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7SetWindowResultCase = _ST7API.St7SetWindowResultCase
St7SetWindowResultCase.argtypes = [c_long, c_long]
St7SetWindowLoadCase = _ST7API.St7SetWindowLoadCase
St7SetWindowLoadCase.argtypes = [c_long, c_long]
St7SetWindowFreedomCase = _ST7API.St7SetWindowFreedomCase
St7SetWindowFreedomCase.argtypes = [c_long, c_long]
St7SetWindowUCSCase = _ST7API.St7SetWindowUCSCase
St7SetWindowUCSCase.argtypes = [c_long, c_long]
St7SetBeamResultDisplay = _ST7API.St7SetBeamResultDisplay
St7SetBeamResultDisplay.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetPlateResultDisplay = _ST7API.St7SetPlateResultDisplay
St7SetPlateResultDisplay.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetBrickResultDisplay = _ST7API.St7SetBrickResultDisplay
St7SetBrickResultDisplay.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetLinkResultDisplay = _ST7API.St7SetLinkResultDisplay
St7SetLinkResultDisplay.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetResultSettingsStyle = _ST7API.St7SetResultSettingsStyle
St7SetResultSettingsStyle.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetResultSettingsStyle = _ST7API.St7GetResultSettingsStyle
St7GetResultSettingsStyle.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetResultSettingsLimits = _ST7API.St7SetResultSettingsLimits
St7SetResultSettingsLimits.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetResultSettingsLimits = _ST7API.St7GetResultSettingsLimits
St7GetResultSettingsLimits.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetResultSettingsLimitsString = _ST7API.St7SetResultSettingsLimitsString
St7SetResultSettingsLimitsString.argtypes = [c_long, c_long, c_long, c_long, c_char_p]
St7GetResultSettingsLimitsString = _ST7API.St7GetResultSettingsLimitsString
St7GetResultSettingsLimitsString.argtypes = [c_long, c_long, c_long, c_long, c_char_p, c_long]
St7SetResultSettingsLegend = _ST7API.St7SetResultSettingsLegend
St7SetResultSettingsLegend.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetResultSettingsLegend = _ST7API.St7GetResultSettingsLegend
St7GetResultSettingsLegend.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetResultSettingsLegendFont = _ST7API.St7SetResultSettingsLegendFont
St7SetResultSettingsLegendFont.argtypes = [c_long, c_long, c_long, c_long, c_char_p, ctypes.POINTER(c_long)]
St7GetResultSettingsLegendFont = _ST7API.St7GetResultSettingsLegendFont
St7GetResultSettingsLegendFont.argtypes = [c_long, c_long, c_long, c_long, c_char_p, c_long, ctypes.POINTER(c_long)]
St7SetResultSettingsDiagram = _ST7API.St7SetResultSettingsDiagram
St7SetResultSettingsDiagram.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetResultSettingsDiagram = _ST7API.St7GetResultSettingsDiagram
St7GetResultSettingsDiagram.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetResultSettingsDiagramColours = _ST7API.St7SetResultSettingsDiagramColours
St7SetResultSettingsDiagramColours.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetResultSettingsDiagramColours = _ST7API.St7GetResultSettingsDiagramColours
St7GetResultSettingsDiagramColours.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetWindowColours = _ST7API.St7SetWindowColours
St7SetWindowColours.argtypes = [c_long, c_long, c_long, c_long]
St7GetWindowColours = _ST7API.St7GetWindowColours
St7GetWindowColours.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetWindowBackgroundMode = _ST7API.St7SetWindowBackgroundMode
St7SetWindowBackgroundMode.argtypes = [c_long, c_long, c_long]
St7GetWindowBackgroundMode = _ST7API.St7GetWindowBackgroundMode
St7GetWindowBackgroundMode.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetNumericOptions = _ST7API.St7SetNumericOptions
St7SetNumericOptions.argtypes = [c_long, c_long, c_long, c_long, c_long, c_double]
St7GetNumericOptions = _ST7API.St7GetNumericOptions
St7GetNumericOptions.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetNodeStyle = _ST7API.St7SetNodeStyle
St7SetNodeStyle.argtypes = [c_long, c_long]
St7SetNodeShowHideSelected = _ST7API.St7SetNodeShowHideSelected
St7SetNodeShowHideSelected.argtypes = [c_long, c_bool]
St7SetNodeSize = _ST7API.St7SetNodeSize
St7SetNodeSize.argtypes = [c_long, c_long]
St7SetFreeNodes = _ST7API.St7SetFreeNodes
St7SetFreeNodes.argtypes = [c_long, c_long]
St7SetNodeLabelStyle = _ST7API.St7SetNodeLabelStyle
St7SetNodeLabelStyle.argtypes = [c_long, c_long]
St7SetNodeColours = _ST7API.St7SetNodeColours
St7SetNodeColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7GetNodeStyle = _ST7API.St7GetNodeStyle
St7GetNodeStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetNodeShowHideSelected = _ST7API.St7GetNodeShowHideSelected
St7GetNodeShowHideSelected.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetNodeSize = _ST7API.St7GetNodeSize
St7GetNodeSize.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetFreeNodes = _ST7API.St7GetFreeNodes
St7GetFreeNodes.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetNodeLabelStyle = _ST7API.St7GetNodeLabelStyle
St7GetNodeLabelStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetNodeColours = _ST7API.St7GetNodeColours
St7GetNodeColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7SetBeamStyle = _ST7API.St7SetBeamStyle
St7SetBeamStyle.argtypes = [c_long, c_long]
St7SetBeamCableAsLine = _ST7API.St7SetBeamCableAsLine
St7SetBeamCableAsLine.argtypes = [c_long, c_bool]
St7SetBeamFill = _ST7API.St7SetBeamFill
St7SetBeamFill.argtypes = [c_long, c_long]
St7SetBeamOutline = _ST7API.St7SetBeamOutline
St7SetBeamOutline.argtypes = [c_long, c_long]
St7SetBeamLineThickness = _ST7API.St7SetBeamLineThickness
St7SetBeamLineThickness.argtypes = [c_long, c_long]
St7SetBeamLabelStyle = _ST7API.St7SetBeamLabelStyle
St7SetBeamLabelStyle.argtypes = [c_long, c_long]
St7SetBeamColours = _ST7API.St7SetBeamColours
St7SetBeamColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7SetBeamLighting = _ST7API.St7SetBeamLighting
St7SetBeamLighting.argtypes = [c_long, c_bool, c_bool]
St7SetBeamNRef = _ST7API.St7SetBeamNRef
St7SetBeamNRef.argtypes = [c_long, c_bool]
St7SetBeamOffsetNodes = _ST7API.St7SetBeamOffsetNodes
St7SetBeamOffsetNodes.argtypes = [c_long, c_bool]
St7SetBeamMoveToOffset = _ST7API.St7SetBeamMoveToOffset
St7SetBeamMoveToOffset.argtypes = [c_long, c_bool]
St7SetBeamDrawAxes = _ST7API.St7SetBeamDrawAxes
St7SetBeamDrawAxes.argtypes = [c_long, c_bool]
St7SetBeamSpringCoils = _ST7API.St7SetBeamSpringCoils
St7SetBeamSpringCoils.argtypes = [c_long, c_long]
St7SetBeamSpringAspect = _ST7API.St7SetBeamSpringAspect
St7SetBeamSpringAspect.argtypes = [c_long, c_long]
St7SetBeamRoundFacets = _ST7API.St7SetBeamRoundFacets
St7SetBeamRoundFacets.argtypes = [c_long, c_long]
St7SetBeamSlices = _ST7API.St7SetBeamSlices
St7SetBeamSlices.argtypes = [c_long, c_long]
St7SetBeamShrink = _ST7API.St7SetBeamShrink
St7SetBeamShrink.argtypes = [c_long, c_long]
St7GetBeamStyle = _ST7API.St7GetBeamStyle
St7GetBeamStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBeamCableAsLine = _ST7API.St7GetBeamCableAsLine
St7GetBeamCableAsLine.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetBeamFill = _ST7API.St7GetBeamFill
St7GetBeamFill.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBeamOutline = _ST7API.St7GetBeamOutline
St7GetBeamOutline.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBeamLineThickness = _ST7API.St7GetBeamLineThickness
St7GetBeamLineThickness.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBeamLabelStyle = _ST7API.St7GetBeamLabelStyle
St7GetBeamLabelStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBeamColours = _ST7API.St7GetBeamColours
St7GetBeamColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7GetBeamLighting = _ST7API.St7GetBeamLighting
St7GetBeamLighting.argtypes = [c_long, ctypes.POINTER(c_bool), ctypes.POINTER(c_bool)]
St7GetBeamNRef = _ST7API.St7GetBeamNRef
St7GetBeamNRef.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetBeamOffsetNodes = _ST7API.St7GetBeamOffsetNodes
St7GetBeamOffsetNodes.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetBeamMoveToOffset = _ST7API.St7GetBeamMoveToOffset
St7GetBeamMoveToOffset.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetBeamDrawAxes = _ST7API.St7GetBeamDrawAxes
St7GetBeamDrawAxes.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetBeamSpringCoils = _ST7API.St7GetBeamSpringCoils
St7GetBeamSpringCoils.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBeamSpringAspect = _ST7API.St7GetBeamSpringAspect
St7GetBeamSpringAspect.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBeamRoundFacets = _ST7API.St7GetBeamRoundFacets
St7GetBeamRoundFacets.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBeamSlices = _ST7API.St7GetBeamSlices
St7GetBeamSlices.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBeamShrink = _ST7API.St7GetBeamShrink
St7GetBeamShrink.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetPlateStyle = _ST7API.St7SetPlateStyle
St7SetPlateStyle.argtypes = [c_long, c_long]
St7SetPlateFill = _ST7API.St7SetPlateFill
St7SetPlateFill.argtypes = [c_long, c_long]
St7SetPlateOutline = _ST7API.St7SetPlateOutline
St7SetPlateOutline.argtypes = [c_long, c_long]
St7SetPlateLineThickness = _ST7API.St7SetPlateLineThickness
St7SetPlateLineThickness.argtypes = [c_long, c_long]
St7SetPlateLabelStyle = _ST7API.St7SetPlateLabelStyle
St7SetPlateLabelStyle.argtypes = [c_long, c_long]
St7SetPlateColours = _ST7API.St7SetPlateColours
St7SetPlateColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7SetPlateLighting = _ST7API.St7SetPlateLighting
St7SetPlateLighting.argtypes = [c_long, c_bool, c_bool]
St7SetPlateOffsetNodes = _ST7API.St7SetPlateOffsetNodes
St7SetPlateOffsetNodes.argtypes = [c_long, c_bool]
St7SetPlateMoveToOffset = _ST7API.St7SetPlateMoveToOffset
St7SetPlateMoveToOffset.argtypes = [c_long, c_bool]
St7SetPlateDrawAxes = _ST7API.St7SetPlateDrawAxes
St7SetPlateDrawAxes.argtypes = [c_long, c_bool]
St7SetPlateShrink = _ST7API.St7SetPlateShrink
St7SetPlateShrink.argtypes = [c_long, c_long]
St7SetPlateFaceNodes = _ST7API.St7SetPlateFaceNodes
St7SetPlateFaceNodes.argtypes = [c_long, c_bool]
St7SetPlateAxisLayer = _ST7API.St7SetPlateAxisLayer
St7SetPlateAxisLayer.argtypes = [c_long, c_long]
St7SetPlateOutlineMode = _ST7API.St7SetPlateOutlineMode
St7SetPlateOutlineMode.argtypes = [c_long, c_long]
St7SetPlateAverageNormals = _ST7API.St7SetPlateAverageNormals
St7SetPlateAverageNormals.argtypes = [c_long, c_bool]
St7SetPlateAverageNormalsAngle = _ST7API.St7SetPlateAverageNormalsAngle
St7SetPlateAverageNormalsAngle.argtypes = [c_long, c_long]
St7GetPlateStyle = _ST7API.St7GetPlateStyle
St7GetPlateStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPlateFill = _ST7API.St7GetPlateFill
St7GetPlateFill.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPlateOutline = _ST7API.St7GetPlateOutline
St7GetPlateOutline.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPlateLineThickness = _ST7API.St7GetPlateLineThickness
St7GetPlateLineThickness.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPlateLabelStyle = _ST7API.St7GetPlateLabelStyle
St7GetPlateLabelStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPlateColours = _ST7API.St7GetPlateColours
St7GetPlateColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7GetPlateLighting = _ST7API.St7GetPlateLighting
St7GetPlateLighting.argtypes = [c_long, ctypes.POINTER(c_bool), ctypes.POINTER(c_bool)]
St7GetPlateOffsetNodes = _ST7API.St7GetPlateOffsetNodes
St7GetPlateOffsetNodes.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetPlateMoveToOffset = _ST7API.St7GetPlateMoveToOffset
St7GetPlateMoveToOffset.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetPlateDrawAxes = _ST7API.St7GetPlateDrawAxes
St7GetPlateDrawAxes.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetPlateShrink = _ST7API.St7GetPlateShrink
St7GetPlateShrink.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPlateFaceNodes = _ST7API.St7GetPlateFaceNodes
St7GetPlateFaceNodes.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetPlateAxisLayer = _ST7API.St7GetPlateAxisLayer
St7GetPlateAxisLayer.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPlateOutlineMode = _ST7API.St7GetPlateOutlineMode
St7GetPlateOutlineMode.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPlateAverageNormals = _ST7API.St7GetPlateAverageNormals
St7GetPlateAverageNormals.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetPlateAverageNormalsAngle = _ST7API.St7GetPlateAverageNormalsAngle
St7GetPlateAverageNormalsAngle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetBrickFill = _ST7API.St7SetBrickFill
St7SetBrickFill.argtypes = [c_long, c_long]
St7SetBrickOutline = _ST7API.St7SetBrickOutline
St7SetBrickOutline.argtypes = [c_long, c_long]
St7SetBrickLineThickness = _ST7API.St7SetBrickLineThickness
St7SetBrickLineThickness.argtypes = [c_long, c_long]
St7SetBrickLabelStyle = _ST7API.St7SetBrickLabelStyle
St7SetBrickLabelStyle.argtypes = [c_long, c_long]
St7SetBrickColours = _ST7API.St7SetBrickColours
St7SetBrickColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7SetBrickLighting = _ST7API.St7SetBrickLighting
St7SetBrickLighting.argtypes = [c_long, c_bool, c_bool]
St7SetBrickDrawAxes = _ST7API.St7SetBrickDrawAxes
St7SetBrickDrawAxes.argtypes = [c_long, c_bool, c_bool, c_bool]
St7SetBrickShrink = _ST7API.St7SetBrickShrink
St7SetBrickShrink.argtypes = [c_long, c_long]
St7SetBrickOutlineMode = _ST7API.St7SetBrickOutlineMode
St7SetBrickOutlineMode.argtypes = [c_long, c_long]
St7SetBrickWireframeAll = _ST7API.St7SetBrickWireframeAll
St7SetBrickWireframeAll.argtypes = [c_long, c_bool]
St7GetBrickFill = _ST7API.St7GetBrickFill
St7GetBrickFill.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBrickOutline = _ST7API.St7GetBrickOutline
St7GetBrickOutline.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBrickLineThickness = _ST7API.St7GetBrickLineThickness
St7GetBrickLineThickness.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBrickLabelStyle = _ST7API.St7GetBrickLabelStyle
St7GetBrickLabelStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBrickColours = _ST7API.St7GetBrickColours
St7GetBrickColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7GetBrickLighting = _ST7API.St7GetBrickLighting
St7GetBrickLighting.argtypes = [c_long, ctypes.POINTER(c_bool), ctypes.POINTER(c_bool)]
St7GetBrickDrawAxes = _ST7API.St7GetBrickDrawAxes
St7GetBrickDrawAxes.argtypes = [c_long, ctypes.POINTER(c_bool), ctypes.POINTER(c_bool), ctypes.POINTER(c_bool)]
St7GetBrickShrink = _ST7API.St7GetBrickShrink
St7GetBrickShrink.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBrickOutlineMode = _ST7API.St7GetBrickOutlineMode
St7GetBrickOutlineMode.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetBrickWireframeAll = _ST7API.St7GetBrickWireframeAll
St7GetBrickWireframeAll.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetLinkOutline = _ST7API.St7SetLinkOutline
St7SetLinkOutline.argtypes = [c_long, c_long]
St7SetLinkLineThickness = _ST7API.St7SetLinkLineThickness
St7SetLinkLineThickness.argtypes = [c_long, c_long]
St7SetLinkLabelStyle = _ST7API.St7SetLinkLabelStyle
St7SetLinkLabelStyle.argtypes = [c_long, c_long]
St7SetLinkColours = _ST7API.St7SetLinkColours
St7SetLinkColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7SetLinkDashes = _ST7API.St7SetLinkDashes
St7SetLinkDashes.argtypes = [c_long, c_bool]
St7GetLinkOutline = _ST7API.St7GetLinkOutline
St7GetLinkOutline.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetLinkLineThickness = _ST7API.St7GetLinkLineThickness
St7GetLinkLineThickness.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetLinkLabelStyle = _ST7API.St7GetLinkLabelStyle
St7GetLinkLabelStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetLinkColours = _ST7API.St7GetLinkColours
St7GetLinkColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7GetLinkDashes = _ST7API.St7GetLinkDashes
St7GetLinkDashes.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetVertexStyle = _ST7API.St7SetVertexStyle
St7SetVertexStyle.argtypes = [c_long, c_long]
St7SetVertexShowHideSelected = _ST7API.St7SetVertexShowHideSelected
St7SetVertexShowHideSelected.argtypes = [c_long, c_bool]
St7SetVertexSize = _ST7API.St7SetVertexSize
St7SetVertexSize.argtypes = [c_long, c_long]
St7SetVertexLabelStyle = _ST7API.St7SetVertexLabelStyle
St7SetVertexLabelStyle.argtypes = [c_long, c_long]
St7SetVertexColours = _ST7API.St7SetVertexColours
St7SetVertexColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7SetFreeVertices = _ST7API.St7SetFreeVertices
St7SetFreeVertices.argtypes = [c_long, c_long]
St7GetVertexStyle = _ST7API.St7GetVertexStyle
St7GetVertexStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetVertexShowHideSelected = _ST7API.St7GetVertexShowHideSelected
St7GetVertexShowHideSelected.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetVertexSize = _ST7API.St7GetVertexSize
St7GetVertexSize.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetVertexLabelStyle = _ST7API.St7GetVertexLabelStyle
St7GetVertexLabelStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetVertexColours = _ST7API.St7GetVertexColours
St7GetVertexColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7GetFreeVertices = _ST7API.St7GetFreeVertices
St7GetFreeVertices.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetFaceFillStyle = _ST7API.St7SetFaceFillStyle
St7SetFaceFillStyle.argtypes = [c_long, c_long]
St7SetFaceFill = _ST7API.St7SetFaceFill
St7SetFaceFill.argtypes = [c_long, c_long]
St7SetFaceOutline = _ST7API.St7SetFaceOutline
St7SetFaceOutline.argtypes = [c_long, c_long]
St7SetFaceLabelStyle = _ST7API.St7SetFaceLabelStyle
St7SetFaceLabelStyle.argtypes = [c_long, c_long]
St7SetFaceColours = _ST7API.St7SetFaceColours
St7SetFaceColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7SetFaceLighting = _ST7API.St7SetFaceLighting
St7SetFaceLighting.argtypes = [c_long, c_bool, c_bool]
St7SetFaceLineThickness = _ST7API.St7SetFaceLineThickness
St7SetFaceLineThickness.argtypes = [c_long, c_long]
St7SetFaceWireThickness = _ST7API.St7SetFaceWireThickness
St7SetFaceWireThickness.argtypes = [c_long, c_long]
St7SetFaceWireDensity = _ST7API.St7SetFaceWireDensity
St7SetFaceWireDensity.argtypes = [c_long, c_long]
St7SetFaceNormalsSize = _ST7API.St7SetFaceNormalsSize
St7SetFaceNormalsSize.argtypes = [c_long, c_long]
St7SetFaceNIEdges = _ST7API.St7SetFaceNIEdges
St7SetFaceNIEdges.argtypes = [c_long, c_bool]
St7SetFaceControlPoints = _ST7API.St7SetFaceControlPoints
St7SetFaceControlPoints.argtypes = [c_long, c_bool]
St7SetFaceNormals = _ST7API.St7SetFaceNormals
St7SetFaceNormals.argtypes = [c_long, c_bool]
St7GetFaceFillStyle = _ST7API.St7GetFaceFillStyle
St7GetFaceFillStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetFaceFill = _ST7API.St7GetFaceFill
St7GetFaceFill.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetFaceOutline = _ST7API.St7GetFaceOutline
St7GetFaceOutline.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetFaceLabelStyle = _ST7API.St7GetFaceLabelStyle
St7GetFaceLabelStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetFaceColours = _ST7API.St7GetFaceColours
St7GetFaceColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7GetFaceLighting = _ST7API.St7GetFaceLighting
St7GetFaceLighting.argtypes = [c_long, ctypes.POINTER(c_bool), ctypes.POINTER(c_bool)]
St7GetFaceLineThickness = _ST7API.St7GetFaceLineThickness
St7GetFaceLineThickness.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetFaceWireThickness = _ST7API.St7GetFaceWireThickness
St7GetFaceWireThickness.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetFaceWireDensity = _ST7API.St7GetFaceWireDensity
St7GetFaceWireDensity.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetFaceNormalsSize = _ST7API.St7GetFaceNormalsSize
St7GetFaceNormalsSize.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetFaceNIEdges = _ST7API.St7GetFaceNIEdges
St7GetFaceNIEdges.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetFaceControlPoints = _ST7API.St7GetFaceControlPoints
St7GetFaceControlPoints.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetFaceNormals = _ST7API.St7GetFaceNormals
St7GetFaceNormals.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetPathFill = _ST7API.St7SetPathFill
St7SetPathFill.argtypes = [c_long, c_long]
St7SetPathOutline = _ST7API.St7SetPathOutline
St7SetPathOutline.argtypes = [c_long, c_long]
St7SetPathLabelStyle = _ST7API.St7SetPathLabelStyle
St7SetPathLabelStyle.argtypes = [c_long, c_long]
St7SetPathColours = _ST7API.St7SetPathColours
St7SetPathColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7SetPathLighting = _ST7API.St7SetPathLighting
St7SetPathLighting.argtypes = [c_long, c_bool, c_bool]
St7SetPathLineThickness = _ST7API.St7SetPathLineThickness
St7SetPathLineThickness.argtypes = [c_long, c_long]
St7SetPathDivisions = _ST7API.St7SetPathDivisions
St7SetPathDivisions.argtypes = [c_long, c_bool]
St7GetPathFill = _ST7API.St7GetPathFill
St7GetPathFill.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPathOutline = _ST7API.St7GetPathOutline
St7GetPathOutline.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPathLabelStyle = _ST7API.St7GetPathLabelStyle
St7GetPathLabelStyle.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPathColours = _ST7API.St7GetPathColours
St7GetPathColours.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7GetPathLighting = _ST7API.St7GetPathLighting
St7GetPathLighting.argtypes = [c_long, ctypes.POINTER(c_bool), ctypes.POINTER(c_bool)]
St7GetPathLineThickness = _ST7API.St7GetPathLineThickness
St7GetPathLineThickness.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetPathDivisions = _ST7API.St7GetPathDivisions
St7GetPathDivisions.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetAttributeDisplay = _ST7API.St7SetAttributeDisplay
St7SetAttributeDisplay.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetAttributeDisplay = _ST7API.St7GetAttributeDisplay
St7GetAttributeDisplay.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetEntityFont = _ST7API.St7SetEntityFont
St7SetEntityFont.argtypes = [c_long, c_long, c_char_p, ctypes.POINTER(c_long)]
St7GetEntityFont = _ST7API.St7GetEntityFont
St7GetEntityFont.argtypes = [c_long, c_long, c_char_p, c_long, ctypes.POINTER(c_long)]
St7SetEntityContourFile = _ST7API.St7SetEntityContourFile
St7SetEntityContourFile.argtypes = [c_long, c_long, c_long, c_char_p]
St7GetEntityContourFile = _ST7API.St7GetEntityContourFile
St7GetEntityContourFile.argtypes = [c_long, c_long, ctypes.POINTER(c_long), c_char_p, c_long]
St7SetEntityContourIndex = _ST7API.St7SetEntityContourIndex
St7SetEntityContourIndex.argtypes = [c_long, c_long, c_long]
St7GetEntityContourIndex = _ST7API.St7GetEntityContourIndex
St7GetEntityContourIndex.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetEntityContourSettingsStyle = _ST7API.St7SetEntityContourSettingsStyle
St7SetEntityContourSettingsStyle.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetEntityContourSettingsStyle = _ST7API.St7GetEntityContourSettingsStyle
St7GetEntityContourSettingsStyle.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetEntityContourSettingsLimits = _ST7API.St7SetEntityContourSettingsLimits
St7SetEntityContourSettingsLimits.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetEntityContourSettingsLimits = _ST7API.St7GetEntityContourSettingsLimits
St7GetEntityContourSettingsLimits.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetEntityContourSettingsLimitsString = _ST7API.St7SetEntityContourSettingsLimitsString
St7SetEntityContourSettingsLimitsString.argtypes = [c_long, c_long, c_char_p]
St7GetEntityContourSettingsLimitsString = _ST7API.St7GetEntityContourSettingsLimitsString
St7GetEntityContourSettingsLimitsString.argtypes = [c_long, c_long, c_char_p, c_long]
St7SetEntityContourSettingsLegend = _ST7API.St7SetEntityContourSettingsLegend
St7SetEntityContourSettingsLegend.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetEntityContourSettingsLegend = _ST7API.St7GetEntityContourSettingsLegend
St7GetEntityContourSettingsLegend.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetEntityContourSettingsLegendFont = _ST7API.St7SetEntityContourSettingsLegendFont
St7SetEntityContourSettingsLegendFont.argtypes = [c_long, c_long, c_char_p, ctypes.POINTER(c_long)]
St7GetEntityContourSettingsLegendFont = _ST7API.St7GetEntityContourSettingsLegendFont
St7GetEntityContourSettingsLegendFont.argtypes = [c_long, c_long, c_char_p, c_long, ctypes.POINTER(c_long)]
St7SetModelDefaults = _ST7API.St7SetModelDefaults
St7SetModelDefaults.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetDisplacementScale = _ST7API.St7SetDisplacementScale
St7SetDisplacementScale.argtypes = [c_long, c_double, c_long]
St7GetDisplacementScale = _ST7API.St7GetDisplacementScale
St7GetDisplacementScale.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_long)]
St7DeleteAllGraphs = _ST7API.St7DeleteAllGraphs
St7DeleteAllGraphs.argtypes = [c_long]
St7ImportST7 = _ST7API.St7ImportST7
St7ImportST7.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7ImportIGES = _ST7API.St7ImportIGES
St7ImportIGES.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), ctypes.POINTER(c_double), c_long]
St7ImportACIS = _ST7API.St7ImportACIS
St7ImportACIS.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), ctypes.POINTER(c_double), c_long]
St7ImportSTEP = _ST7API.St7ImportSTEP
St7ImportSTEP.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), ctypes.POINTER(c_double), c_long]
St7ImportRhino = _ST7API.St7ImportRhino
St7ImportRhino.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), ctypes.POINTER(c_double), c_long]
St7ImportDXF = _ST7API.St7ImportDXF
St7ImportDXF.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), ctypes.POINTER(c_double), c_long]
St7ImportSTL = _ST7API.St7ImportSTL
St7ImportSTL.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7ImportNASTRAN = _ST7API.St7ImportNASTRAN
St7ImportNASTRAN.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7ImportANSYS = _ST7API.St7ImportANSYS
St7ImportANSYS.argtypes = [c_long, c_char_p, c_char_p, ctypes.POINTER(c_long), c_long]
St7ImportSTAAD = _ST7API.St7ImportSTAAD
St7ImportSTAAD.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7ImportSAP2000 = _ST7API.St7ImportSAP2000
St7ImportSAP2000.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7ExportImage = _ST7API.St7ExportImage
St7ExportImage.argtypes = [c_long, c_char_p, c_long, c_long, c_long]
St7ExportImageToClipboard = _ST7API.St7ExportImageToClipboard
St7ExportImageToClipboard.argtypes = [c_long, c_long, c_long]
St7ExportST7 = _ST7API.St7ExportST7
St7ExportST7.argtypes = [c_long, c_char_p, c_long, c_long]
St7ExportIGES = _ST7API.St7ExportIGES
St7ExportIGES.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7ExportSTEP = _ST7API.St7ExportSTEP
St7ExportSTEP.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7ExportDXF = _ST7API.St7ExportDXF
St7ExportDXF.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7ExportSTL = _ST7API.St7ExportSTL
St7ExportSTL.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7ExportNASTRAN = _ST7API.St7ExportNASTRAN
St7ExportNASTRAN.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), ctypes.POINTER(c_double), c_long]
St7ExportANSYS = _ST7API.St7ExportANSYS
St7ExportANSYS.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7PlayAnimationFile = _ST7API.St7PlayAnimationFile
St7PlayAnimationFile.argtypes = [c_char_p, ctypes.POINTER(c_long)]
St7CreateAnimation = _ST7API.St7CreateAnimation
St7CreateAnimation.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7CreateAnimationEmbedded = _ST7API.St7CreateAnimationEmbedded
St7CreateAnimationEmbedded.argtypes = [c_long, HWND, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7CreateAnimationFile = _ST7API.St7CreateAnimationFile
St7CreateAnimationFile.argtypes = [c_long, ctypes.POINTER(c_long), c_char_p]
St7CloseAnimation = _ST7API.St7CloseAnimation
St7CloseAnimation.argtypes = [c_long]
St7SetAnimationCase = _ST7API.St7SetAnimationCase
St7SetAnimationCase.argtypes = [c_long, c_long, c_bool]
St7GetAnimationCase = _ST7API.St7GetAnimationCase
St7GetAnimationCase.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7GetTotal = _ST7API.St7GetTotal
St7GetTotal.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetTitle = _ST7API.St7SetTitle
St7SetTitle.argtypes = [c_long, c_long, c_char_p]
St7GetTitle = _ST7API.St7GetTitle
St7GetTitle.argtypes = [c_long, c_long, c_char_p, c_long]
St7AddComment = _ST7API.St7AddComment
St7AddComment.argtypes = [c_long, c_char_p]
St7GetNumComments = _ST7API.St7GetNumComments
St7GetNumComments.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetComment = _ST7API.St7SetComment
St7SetComment.argtypes = [c_long, c_long, c_char_p]
St7GetComment = _ST7API.St7GetComment
St7GetComment.argtypes = [c_long, c_long, c_char_p, c_long]
St7DeleteComment = _ST7API.St7DeleteComment
St7DeleteComment.argtypes = [c_long, c_long]
St7GetBeamAxisSystem = _ST7API.St7GetBeamAxisSystem
St7GetBeamAxisSystem.argtypes = [c_long, c_long, c_bool, ctypes.POINTER(c_double)]
St7GetPlateAxisSystem = _ST7API.St7GetPlateAxisSystem
St7GetPlateAxisSystem.argtypes = [c_long, c_long, c_bool, ctypes.POINTER(c_double)]
St7GetBrickFaceAxisSystem = _ST7API.St7GetBrickFaceAxisSystem
St7GetBrickFaceAxisSystem.argtypes = [c_long, c_long, c_long, c_bool, ctypes.POINTER(c_double)]
St7GetPlateNumPlies = _ST7API.St7GetPlateNumPlies
St7GetPlateNumPlies.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetNumBXSLoopsAndPlates = _ST7API.St7GetNumBXSLoopsAndPlates
St7GetNumBXSLoopsAndPlates.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetNumBXSLoopPoints = _ST7API.St7GetNumBXSLoopPoints
St7GetNumBXSLoopPoints.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetBXSLoop = _ST7API.St7GetBXSLoop
St7GetBXSLoop.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBXSLoopType = _ST7API.St7GetBXSLoopType
St7GetBXSLoopType.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GenerateBXS = _ST7API.St7GenerateBXS
St7GenerateBXS.argtypes = [c_long, c_char_p, ctypes.POINTER(c_double)]
St7NewLoadCase = _ST7API.St7NewLoadCase
St7NewLoadCase.argtypes = [c_long, c_char_p]
St7NewFreedomCase = _ST7API.St7NewFreedomCase
St7NewFreedomCase.argtypes = [c_long, c_char_p]
St7GetNumLoadCase = _ST7API.St7GetNumLoadCase
St7GetNumLoadCase.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetNumFreedomCase = _ST7API.St7GetNumFreedomCase
St7GetNumFreedomCase.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetLoadCaseName = _ST7API.St7SetLoadCaseName
St7SetLoadCaseName.argtypes = [c_long, c_long, c_char_p]
St7GetLoadCaseName = _ST7API.St7GetLoadCaseName
St7GetLoadCaseName.argtypes = [c_long, c_long, c_char_p, c_long]
St7SetFreedomCaseName = _ST7API.St7SetFreedomCaseName
St7SetFreedomCaseName.argtypes = [c_long, c_long, c_char_p]
St7GetFreedomCaseName = _ST7API.St7GetFreedomCaseName
St7GetFreedomCaseName.argtypes = [c_long, c_long, c_char_p, c_long]
St7SetLoadCaseDefaults = _ST7API.St7SetLoadCaseDefaults
St7SetLoadCaseDefaults.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetLoadCaseDefaults = _ST7API.St7GetLoadCaseDefaults
St7GetLoadCaseDefaults.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetFreedomCaseDefaults = _ST7API.St7SetFreedomCaseDefaults
St7SetFreedomCaseDefaults.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetFreedomCaseDefaults = _ST7API.St7GetFreedomCaseDefaults
St7GetFreedomCaseDefaults.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetLoadCaseType = _ST7API.St7SetLoadCaseType
St7SetLoadCaseType.argtypes = [c_long, c_long, c_long]
St7GetLoadCaseType = _ST7API.St7GetLoadCaseType
St7GetLoadCaseType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetLoadCaseGravityDir = _ST7API.St7SetLoadCaseGravityDir
St7SetLoadCaseGravityDir.argtypes = [c_long, c_long, c_long]
St7GetLoadCaseGravityDir = _ST7API.St7GetLoadCaseGravityDir
St7GetLoadCaseGravityDir.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetLoadCaseGravity = _ST7API.St7SetLoadCaseGravity
St7SetLoadCaseGravity.argtypes = [c_long, c_long, c_double]
St7GetLoadCaseGravity = _ST7API.St7GetLoadCaseGravity
St7GetLoadCaseGravity.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetSeismicCaseNSMassOption = _ST7API.St7SetSeismicCaseNSMassOption
St7SetSeismicCaseNSMassOption.argtypes = [c_long, c_long, c_bool]
St7GetSeismicCaseNSMassOption = _ST7API.St7GetSeismicCaseNSMassOption
St7GetSeismicCaseNSMassOption.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetFreedomCaseType = _ST7API.St7SetFreedomCaseType
St7SetFreedomCaseType.argtypes = [c_long, c_long, c_long]
St7GetFreedomCaseType = _ST7API.St7GetFreedomCaseType
St7GetFreedomCaseType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetLoadCaseMassOption = _ST7API.St7SetLoadCaseMassOption
St7SetLoadCaseMassOption.argtypes = [c_long, c_long, c_bool, c_bool]
St7GetLoadCaseMassOption = _ST7API.St7GetLoadCaseMassOption
St7GetLoadCaseMassOption.argtypes = [c_long, c_long, ctypes.POINTER(c_bool), ctypes.POINTER(c_bool)]
St7DeleteLoadCase = _ST7API.St7DeleteLoadCase
St7DeleteLoadCase.argtypes = [c_long, c_long]
St7DeleteFreedomCase = _ST7API.St7DeleteFreedomCase
St7DeleteFreedomCase.argtypes = [c_long, c_long]
St7GetNumSeismicCase = _ST7API.St7GetNumSeismicCase
St7GetNumSeismicCase.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetSeismicCaseDefaults = _ST7API.St7SetSeismicCaseDefaults
St7SetSeismicCaseDefaults.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetSeismicCaseDefaults = _ST7API.St7GetSeismicCaseDefaults
St7GetSeismicCaseDefaults.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetUCS = _ST7API.St7SetUCS
St7SetUCS.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetUCS = _ST7API.St7GetUCS
St7GetUCS.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7DeleteUCS = _ST7API.St7DeleteUCS
St7DeleteUCS.argtypes = [c_long, c_long]
St7SetUCSName = _ST7API.St7SetUCSName
St7SetUCSName.argtypes = [c_long, c_long, c_char_p]
St7GetUCSName = _ST7API.St7GetUCSName
St7GetUCSName.argtypes = [c_long, c_long, c_char_p, c_long]
St7GetUCSID = _ST7API.St7GetUCSID
St7GetUCSID.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetNumUCS = _ST7API.St7GetNumUCS
St7GetNumUCS.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetGroupIDName = _ST7API.St7SetGroupIDName
St7SetGroupIDName.argtypes = [c_long, c_long, c_char_p]
St7GetGroupIDName = _ST7API.St7GetGroupIDName
St7GetGroupIDName.argtypes = [c_long, c_long, c_char_p, c_long]
St7GetNumGroups = _ST7API.St7GetNumGroups
St7GetNumGroups.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetGroupByIndex = _ST7API.St7GetGroupByIndex
St7GetGroupByIndex.argtypes = [c_long, c_long, c_char_p, c_long, ctypes.POINTER(c_long)]
St7NewChildGroup = _ST7API.St7NewChildGroup
St7NewChildGroup.argtypes = [c_long, c_long, c_char_p, ctypes.POINTER(c_long)]
St7GetGroupParent = _ST7API.St7GetGroupParent
St7GetGroupParent.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGroupChild = _ST7API.St7GetGroupChild
St7GetGroupChild.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGroupSibling = _ST7API.St7GetGroupSibling
St7GetGroupSibling.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7DeleteGroup = _ST7API.St7DeleteGroup
St7DeleteGroup.argtypes = [c_long, c_long]
St7SetGroupColour = _ST7API.St7SetGroupColour
St7SetGroupColour.argtypes = [c_long, c_long, c_long]
St7GetGroupColour = _ST7API.St7GetGroupColour
St7GetGroupColour.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetDefaultGroupID = _ST7API.St7SetDefaultGroupID
St7SetDefaultGroupID.argtypes = [c_long, c_long]
St7GetDefaultGroupID = _ST7API.St7GetDefaultGroupID
St7GetDefaultGroupID.argtypes = [c_long, ctypes.POINTER(c_long)]
St7AddStage = _ST7API.St7AddStage
St7AddStage.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long)]
St7InsertStage = _ST7API.St7InsertStage
St7InsertStage.argtypes = [c_long, c_long, c_char_p, ctypes.POINTER(c_long)]
St7DeleteStage = _ST7API.St7DeleteStage
St7DeleteStage.argtypes = [c_long, c_long]
St7GetNumStages = _ST7API.St7GetNumStages
St7GetNumStages.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetStageName = _ST7API.St7SetStageName
St7SetStageName.argtypes = [c_long, c_long, c_char_p]
St7GetStageName = _ST7API.St7GetStageName
St7GetStageName.argtypes = [c_long, c_long, c_char_p, c_long]
St7SetStageData = _ST7API.St7SetStageData
St7SetStageData.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetStageData = _ST7API.St7GetStageData
St7GetStageData.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetStageFluidLevel = _ST7API.St7SetStageFluidLevel
St7SetStageFluidLevel.argtypes = [c_long, c_long, c_double]
St7GetStageFluidLevel = _ST7API.St7GetStageFluidLevel
St7GetStageFluidLevel.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7EnableStageGroup = _ST7API.St7EnableStageGroup
St7EnableStageGroup.argtypes = [c_long, c_long, c_long]
St7DisableStageGroup = _ST7API.St7DisableStageGroup
St7DisableStageGroup.argtypes = [c_long, c_long, c_long]
St7GetStageGroupState = _ST7API.St7GetStageGroupState
St7GetStageGroupState.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7NewEntitySet = _ST7API.St7NewEntitySet
St7NewEntitySet.argtypes = [c_long, c_char_p]
St7DeleteEntitySet = _ST7API.St7DeleteEntitySet
St7DeleteEntitySet.argtypes = [c_long, c_long]
St7SetEntitySetName = _ST7API.St7SetEntitySetName
St7SetEntitySetName.argtypes = [c_long, c_long, c_char_p]
St7GetEntitySetName = _ST7API.St7GetEntitySetName
St7GetEntitySetName.argtypes = [c_long, c_long, c_char_p, c_long]
St7GetEntitySetEntityState = _ST7API.St7GetEntitySetEntityState
St7GetEntitySetEntityState.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7ShowEntitySet = _ST7API.St7ShowEntitySet
St7ShowEntitySet.argtypes = [c_long, c_long]
St7HideEntitySet = _ST7API.St7HideEntitySet
St7HideEntitySet.argtypes = [c_long, c_long]
St7GetEntitySetVisibility = _ST7API.St7GetEntitySetVisibility
St7GetEntitySetVisibility.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7GetNumEntitySets = _ST7API.St7GetNumEntitySets
St7GetNumEntitySets.argtypes = [c_long, ctypes.POINTER(c_long)]
St7AddSelectedToEntitySet = _ST7API.St7AddSelectedToEntitySet
St7AddSelectedToEntitySet.argtypes = [c_long, c_long, c_long]
St7RemoveSelectedFromEntitySet = _ST7API.St7RemoveSelectedFromEntitySet
St7RemoveSelectedFromEntitySet.argtypes = [c_long, c_long, c_long]
St7SetUnits = _ST7API.St7SetUnits
St7SetUnits.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetUnits = _ST7API.St7GetUnits
St7GetUnits.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetRCUnits = _ST7API.St7SetRCUnits
St7SetRCUnits.argtypes = [c_long, c_long, c_long]
St7GetRCUnits = _ST7API.St7GetRCUnits
St7GetRCUnits.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7ConvertUnits = _ST7API.St7ConvertUnits
St7ConvertUnits.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetNodeXYZ = _ST7API.St7SetNodeXYZ
St7SetNodeXYZ.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeXYZ = _ST7API.St7GetNodeXYZ
St7GetNodeXYZ.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeUCS = _ST7API.St7SetNodeUCS
St7SetNodeUCS.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeUCS = _ST7API.St7GetNodeUCS
St7GetNodeUCS.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetElementConnection = _ST7API.St7SetElementConnection
St7SetElementConnection.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetElementConnection = _ST7API.St7GetElementConnection
St7GetElementConnection.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetElementData = _ST7API.St7GetElementData
St7GetElementData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetElementDataGNL = _ST7API.St7GetElementDataGNL
St7GetElementDataGNL.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetElementDataDeformed = _ST7API.St7GetElementDataDeformed
St7GetElementDataDeformed.argtypes = [c_long, c_long, c_long, c_double, ctypes.POINTER(c_double)]
St7GetElementCentroid = _ST7API.St7GetElementCentroid
St7GetElementCentroid.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetElementCentroidAtBirth = _ST7API.St7GetElementCentroidAtBirth
St7GetElementCentroidAtBirth.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetMasterSlaveLink = _ST7API.St7SetMasterSlaveLink
St7SetMasterSlaveLink.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetMasterSlaveLink = _ST7API.St7GetMasterSlaveLink
St7GetMasterSlaveLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetSectorSymmetryLink = _ST7API.St7SetSectorSymmetryLink
St7SetSectorSymmetryLink.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetSectorSymmetryLink = _ST7API.St7GetSectorSymmetryLink
St7GetSectorSymmetryLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetCouplingLink = _ST7API.St7SetCouplingLink
St7SetCouplingLink.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetCouplingLink = _ST7API.St7GetCouplingLink
St7GetCouplingLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetPinnedLink = _ST7API.St7SetPinnedLink
St7SetPinnedLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetPinnedLink = _ST7API.St7GetPinnedLink
St7GetPinnedLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetRigidLink = _ST7API.St7SetRigidLink
St7SetRigidLink.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetRigidLink = _ST7API.St7GetRigidLink
St7GetRigidLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetShrinkLink = _ST7API.St7SetShrinkLink
St7SetShrinkLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetShrinkLink = _ST7API.St7GetShrinkLink
St7GetShrinkLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetTwoPointLink = _ST7API.St7SetTwoPointLink
St7SetTwoPointLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetTwoPointLink = _ST7API.St7GetTwoPointLink
St7GetTwoPointLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetAttachmentLink = _ST7API.St7SetAttachmentLink
St7SetAttachmentLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetAttachmentLink = _ST7API.St7GetAttachmentLink
St7GetAttachmentLink.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetInterpolatedMultiPointLink = _ST7API.St7SetInterpolatedMultiPointLink
St7SetInterpolatedMultiPointLink.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetInterpolatedMultiPointLink = _ST7API.St7GetInterpolatedMultiPointLink
St7GetInterpolatedMultiPointLink.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetMasterSlaveMultiPointLink = _ST7API.St7SetMasterSlaveMultiPointLink
St7SetMasterSlaveMultiPointLink.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetMasterSlaveMultiPointLink = _ST7API.St7GetMasterSlaveMultiPointLink
St7GetMasterSlaveMultiPointLink.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetPinnedMultiPointLink = _ST7API.St7SetPinnedMultiPointLink
St7SetPinnedMultiPointLink.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetPinnedMultiPointLink = _ST7API.St7GetPinnedMultiPointLink
St7GetPinnedMultiPointLink.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetRigidMultiPointLink = _ST7API.St7SetRigidMultiPointLink
St7SetRigidMultiPointLink.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetRigidMultiPointLink = _ST7API.St7GetRigidMultiPointLink
St7GetRigidMultiPointLink.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetUserDefinedMultiPointLink = _ST7API.St7SetUserDefinedMultiPointLink
St7SetUserDefinedMultiPointLink.argtypes = [c_long, c_long, c_long, c_long, c_double, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetUserDefinedMultiPointLink = _ST7API.St7GetUserDefinedMultiPointLink
St7GetUserDefinedMultiPointLink.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetReactionMultiPointLink = _ST7API.St7SetReactionMultiPointLink
St7SetReactionMultiPointLink.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetReactionMultiPointLink = _ST7API.St7GetReactionMultiPointLink
St7GetReactionMultiPointLink.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetReactionMultiPointLinkAttributes = _ST7API.St7SetReactionMultiPointLinkAttributes
St7SetReactionMultiPointLinkAttributes.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetReactionMultiPointLinkAttributes = _ST7API.St7GetReactionMultiPointLinkAttributes
St7GetReactionMultiPointLinkAttributes.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetInterpolatedMultiPointLinkAttributes = _ST7API.St7SetInterpolatedMultiPointLinkAttributes
St7SetInterpolatedMultiPointLinkAttributes.argtypes = [c_long, c_long, c_long]
St7GetInterpolatedMultiPointLinkAttributes = _ST7API.St7GetInterpolatedMultiPointLinkAttributes
St7GetInterpolatedMultiPointLinkAttributes.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetMasterSlaveMultiPointLinkAttributes = _ST7API.St7SetMasterSlaveMultiPointLinkAttributes
St7SetMasterSlaveMultiPointLinkAttributes.argtypes = [c_long, c_long, c_long, c_long]
St7GetMasterSlaveMultiPointLinkAttributes = _ST7API.St7GetMasterSlaveMultiPointLinkAttributes
St7GetMasterSlaveMultiPointLinkAttributes.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetRigidMultiPointLinkAttributes = _ST7API.St7SetRigidMultiPointLinkAttributes
St7SetRigidMultiPointLinkAttributes.argtypes = [c_long, c_long, c_long, c_long]
St7GetRigidMultiPointLinkAttributes = _ST7API.St7GetRigidMultiPointLinkAttributes
St7GetRigidMultiPointLinkAttributes.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetNumMultiPointLinkNodes = _ST7API.St7GetNumMultiPointLinkNodes
St7GetNumMultiPointLinkNodes.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetLinkType = _ST7API.St7GetLinkType
St7GetLinkType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetVertexXYZ = _ST7API.St7GetVertexXYZ
St7GetVertexXYZ.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryFaceOuterLoops = _ST7API.St7GetGeometryFaceOuterLoops
St7GetGeometryFaceOuterLoops.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetNumGeometryFaceCavityLoops = _ST7API.St7GetNumGeometryFaceCavityLoops
St7GetNumGeometryFaceCavityLoops.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryFaceCavityLoops = _ST7API.St7GetGeometryFaceCavityLoops
St7GetGeometryFaceCavityLoops.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetNumGeometryFaceEdges = _ST7API.St7GetNumGeometryFaceEdges
St7GetNumGeometryFaceEdges.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryFaceEdges = _ST7API.St7GetGeometryFaceEdges
St7GetGeometryFaceEdges.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetNumGeometryLoopEdges = _ST7API.St7GetNumGeometryLoopEdges
St7GetNumGeometryLoopEdges.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryLoopEdges = _ST7API.St7GetGeometryLoopEdges
St7GetGeometryLoopEdges.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryEdgeLength = _ST7API.St7GetGeometryEdgeLength
St7GetGeometryEdgeLength.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetNumGeometryFaceCoedges = _ST7API.St7GetNumGeometryFaceCoedges
St7GetNumGeometryFaceCoedges.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryFaceCoedges = _ST7API.St7GetGeometryFaceCoedges
St7GetGeometryFaceCoedges.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetNumGeometryLoopCoedges = _ST7API.St7GetNumGeometryLoopCoedges
St7GetNumGeometryLoopCoedges.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryLoopCoedges = _ST7API.St7GetGeometryLoopCoedges
St7GetGeometryLoopCoedges.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryCoedgeEdge = _ST7API.St7GetGeometryCoedgeEdge
St7GetGeometryCoedgeEdge.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetNumGeometryFaceVertices = _ST7API.St7GetNumGeometryFaceVertices
St7GetNumGeometryFaceVertices.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryFaceVertices = _ST7API.St7GetGeometryFaceVertices
St7GetGeometryFaceVertices.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryEdgeVertices = _ST7API.St7GetGeometryEdgeVertices
St7GetGeometryEdgeVertices.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryFaceSurface = _ST7API.St7GetGeometryFaceSurface
St7GetGeometryFaceSurface.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometrySurfaceType = _ST7API.St7GetGeometrySurfaceType
St7GetGeometrySurfaceType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7InvalidateGeometryFace = _ST7API.St7InvalidateGeometryFace
St7InvalidateGeometryFace.argtypes = [c_long, c_long]
St7InvalidateGeometryFaceCavityLoopID = _ST7API.St7InvalidateGeometryFaceCavityLoopID
St7InvalidateGeometryFaceCavityLoopID.argtypes = [c_long, c_long, c_long]
St7InvalidateGeometryFaceCavityLoopIndex = _ST7API.St7InvalidateGeometryFaceCavityLoopIndex
St7InvalidateGeometryFaceCavityLoopIndex.argtypes = [c_long, c_long, c_long]
St7DeleteInvalidGeometry = _ST7API.St7DeleteInvalidGeometry
St7DeleteInvalidGeometry.argtypes = [c_long]
St7SetCleanGeometryOptions = _ST7API.St7SetCleanGeometryOptions
St7SetCleanGeometryOptions.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetCleanGeometryOptions = _ST7API.St7GetCleanGeometryOptions
St7GetCleanGeometryOptions.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7CleanGeometry = _ST7API.St7CleanGeometry
St7CleanGeometry.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7GetGeometrySize = _ST7API.St7GetGeometrySize
St7GetGeometrySize.argtypes = [c_long, ctypes.POINTER(c_double)]
St7SetLoadPath = _ST7API.St7SetLoadPath
St7SetLoadPath.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetLoadPath = _ST7API.St7GetLoadPath
St7GetLoadPath.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7DeleteLoadPath = _ST7API.St7DeleteLoadPath
St7DeleteLoadPath.argtypes = [c_long, c_long]
St7SetNodeID = _ST7API.St7SetNodeID
St7SetNodeID.argtypes = [c_long, c_long, c_long]
St7SetNodeRestraint6 = _ST7API.St7SetNodeRestraint6
St7SetNodeRestraint6.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetNodeForce3 = _ST7API.St7SetNodeForce3
St7SetNodeForce3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeMoment3 = _ST7API.St7SetNodeMoment3
St7SetNodeMoment3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeTemperature1 = _ST7API.St7SetNodeTemperature1
St7SetNodeTemperature1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeTemperatureType1 = _ST7API.St7SetNodeTemperatureType1
St7SetNodeTemperatureType1.argtypes = [c_long, c_long, c_long, c_long]
St7SetNodeTemperatureTable = _ST7API.St7SetNodeTemperatureTable
St7SetNodeTemperatureTable.argtypes = [c_long, c_long, c_long, c_long]
St7SetNodeKTranslation3F = _ST7API.St7SetNodeKTranslation3F
St7SetNodeKTranslation3F.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeKRotation3F = _ST7API.St7SetNodeKRotation3F
St7SetNodeKRotation3F.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeTMass1 = _ST7API.St7SetNodeTMass1
St7SetNodeTMass1.argtypes = [c_long, c_long, c_double]
St7SetNodeTMass3 = _ST7API.St7SetNodeTMass3
St7SetNodeTMass3.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeRMass3 = _ST7API.St7SetNodeRMass3
St7SetNodeRMass3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeNSMass5ID = _ST7API.St7SetNodeNSMass5ID
St7SetNodeNSMass5ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeKDamping3F = _ST7API.St7SetNodeKDamping3F
St7SetNodeKDamping3F.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeHeatSource1 = _ST7API.St7SetNodeHeatSource1
St7SetNodeHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeHeatSourceTables = _ST7API.St7SetNodeHeatSourceTables
St7SetNodeHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetNodeInitialVelocity3 = _ST7API.St7SetNodeInitialVelocity3
St7SetNodeInitialVelocity3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeAcceleration3 = _ST7API.St7SetNodeAcceleration3
St7SetNodeAcceleration3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNodeResponse = _ST7API.St7SetNodeResponse
St7SetNodeResponse.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetNodeID = _ST7API.St7GetNodeID
St7GetNodeID.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetNodeRestraint6 = _ST7API.St7GetNodeRestraint6
St7GetNodeRestraint6.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetNodeForce3 = _ST7API.St7GetNodeForce3
St7GetNodeForce3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeMoment3 = _ST7API.St7GetNodeMoment3
St7GetNodeMoment3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeTemperature1 = _ST7API.St7GetNodeTemperature1
St7GetNodeTemperature1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeTemperatureType1 = _ST7API.St7GetNodeTemperatureType1
St7GetNodeTemperatureType1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetNodeTemperatureTable = _ST7API.St7GetNodeTemperatureTable
St7GetNodeTemperatureTable.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetNodeKTranslation3F = _ST7API.St7GetNodeKTranslation3F
St7GetNodeKTranslation3F.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetNodeKRotation3F = _ST7API.St7GetNodeKRotation3F
St7GetNodeKRotation3F.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetNodeTMass3 = _ST7API.St7GetNodeTMass3
St7GetNodeTMass3.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeRMass3 = _ST7API.St7GetNodeRMass3
St7GetNodeRMass3.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetNodeNSMass5ID = _ST7API.St7GetNodeNSMass5ID
St7GetNodeNSMass5ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeKDamping3F = _ST7API.St7GetNodeKDamping3F
St7GetNodeKDamping3F.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetNodeHeatSource1 = _ST7API.St7GetNodeHeatSource1
St7GetNodeHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeHeatSourceTables = _ST7API.St7GetNodeHeatSourceTables
St7GetNodeHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetNodeInitialVelocity3 = _ST7API.St7GetNodeInitialVelocity3
St7GetNodeInitialVelocity3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeAcceleration3 = _ST7API.St7GetNodeAcceleration3
St7GetNodeAcceleration3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeResponse = _ST7API.St7GetNodeResponse
St7GetNodeResponse.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetBeamID = _ST7API.St7SetBeamID
St7SetBeamID.argtypes = [c_long, c_long, c_long]
St7SetBeamReferenceAngle1 = _ST7API.St7SetBeamReferenceAngle1
St7SetBeamReferenceAngle1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamConnectionUCS = _ST7API.St7SetBeamConnectionUCS
St7SetBeamConnectionUCS.argtypes = [c_long, c_long, c_long, c_long]
St7SetBeamTaper2 = _ST7API.St7SetBeamTaper2
St7SetBeamTaper2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamOffset2 = _ST7API.St7SetBeamOffset2
St7SetBeamOffset2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamSupport2 = _ST7API.St7SetBeamSupport2
St7SetBeamSupport2.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamSectionFactor7 = _ST7API.St7SetBeamSectionFactor7
St7SetBeamSectionFactor7.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamTRelease3 = _ST7API.St7SetBeamTRelease3
St7SetBeamTRelease3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBeamRRelease3 = _ST7API.St7SetBeamRRelease3
St7SetBeamRRelease3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBeamCableFreeLength1 = _ST7API.St7SetBeamCableFreeLength1
St7SetBeamCableFreeLength1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamRadius1 = _ST7API.St7SetBeamRadius1
St7SetBeamRadius1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPipePressure2AF = _ST7API.St7SetPipePressure2AF
St7SetPipePressure2AF.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPipeTemperature2OT = _ST7API.St7SetPipeTemperature2OT
St7SetPipeTemperature2OT.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamStringGroup1 = _ST7API.St7SetBeamStringGroup1
St7SetBeamStringGroup1.argtypes = [c_long, c_long, c_long]
St7SetBeamPreLoad1 = _ST7API.St7SetBeamPreLoad1
St7SetBeamPreLoad1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamTempGradient2 = _ST7API.St7SetBeamTempGradient2
St7SetBeamTempGradient2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamPreCurvature2 = _ST7API.St7SetBeamPreCurvature2
St7SetBeamPreCurvature2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamPointForcePrincipal4ID = _ST7API.St7SetBeamPointForcePrincipal4ID
St7SetBeamPointForcePrincipal4ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamPointForceGlobal4ID = _ST7API.St7SetBeamPointForceGlobal4ID
St7SetBeamPointForceGlobal4ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamPointMomentPrincipal4ID = _ST7API.St7SetBeamPointMomentPrincipal4ID
St7SetBeamPointMomentPrincipal4ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamPointMomentGlobal4ID = _ST7API.St7SetBeamPointMomentGlobal4ID
St7SetBeamPointMomentGlobal4ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamDistributedForcePrincipal6ID = _ST7API.St7SetBeamDistributedForcePrincipal6ID
St7SetBeamDistributedForcePrincipal6ID.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamDistributedForceGlobal6ID = _ST7API.St7SetBeamDistributedForceGlobal6ID
St7SetBeamDistributedForceGlobal6ID.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamDistributedMomentPrincipal6ID = _ST7API.St7SetBeamDistributedMomentPrincipal6ID
St7SetBeamDistributedMomentPrincipal6ID.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamNSMass10ID = _ST7API.St7SetBeamNSMass10ID
St7SetBeamNSMass10ID.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamConvection2 = _ST7API.St7SetBeamConvection2
St7SetBeamConvection2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamConvectionTables = _ST7API.St7SetBeamConvectionTables
St7SetBeamConvectionTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetBeamRadiation2 = _ST7API.St7SetBeamRadiation2
St7SetBeamRadiation2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamRadiationTables = _ST7API.St7SetBeamRadiationTables
St7SetBeamRadiationTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetBeamFlux1 = _ST7API.St7SetBeamFlux1
St7SetBeamFlux1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamFluxTables = _ST7API.St7SetBeamFluxTables
St7SetBeamFluxTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetBeamHeatSource1 = _ST7API.St7SetBeamHeatSource1
St7SetBeamHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamHeatSourceTables = _ST7API.St7SetBeamHeatSourceTables
St7SetBeamHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetBeamResponse = _ST7API.St7SetBeamResponse
St7SetBeamResponse.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetBeamCreepLoadingAge1 = _ST7API.St7SetBeamCreepLoadingAge1
St7SetBeamCreepLoadingAge1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamEndAttachment1 = _ST7API.St7SetBeamEndAttachment1
St7SetBeamEndAttachment1.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamSideAttachment1 = _ST7API.St7SetBeamSideAttachment1
St7SetBeamSideAttachment1.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamID = _ST7API.St7GetBeamID
St7GetBeamID.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetBeamReferenceAngle1 = _ST7API.St7GetBeamReferenceAngle1
St7GetBeamReferenceAngle1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamConnectionUCS = _ST7API.St7GetBeamConnectionUCS
St7GetBeamConnectionUCS.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetBeamTaper2 = _ST7API.St7GetBeamTaper2
St7GetBeamTaper2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamOffset2 = _ST7API.St7GetBeamOffset2
St7GetBeamOffset2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamSupport2 = _ST7API.St7GetBeamSupport2
St7GetBeamSupport2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamSectionFactor7 = _ST7API.St7GetBeamSectionFactor7
St7GetBeamSectionFactor7.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamTRelease3 = _ST7API.St7GetBeamTRelease3
St7GetBeamTRelease3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamRRelease3 = _ST7API.St7GetBeamRRelease3
St7GetBeamRRelease3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamCableFreeLength1 = _ST7API.St7GetBeamCableFreeLength1
St7GetBeamCableFreeLength1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamRadius1 = _ST7API.St7GetBeamRadius1
St7GetBeamRadius1.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPipePressure2AF = _ST7API.St7GetPipePressure2AF
St7GetPipePressure2AF.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPipeTemperature2OT = _ST7API.St7GetPipeTemperature2OT
St7GetPipeTemperature2OT.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamStringGroup1 = _ST7API.St7GetBeamStringGroup1
St7GetBeamStringGroup1.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetBeamPreLoad1 = _ST7API.St7GetBeamPreLoad1
St7GetBeamPreLoad1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamTempGradient2 = _ST7API.St7GetBeamTempGradient2
St7GetBeamTempGradient2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamPreCurvature2 = _ST7API.St7GetBeamPreCurvature2
St7GetBeamPreCurvature2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamPointForcePrincipal4ID = _ST7API.St7GetBeamPointForcePrincipal4ID
St7GetBeamPointForcePrincipal4ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamPointForceGlobal4ID = _ST7API.St7GetBeamPointForceGlobal4ID
St7GetBeamPointForceGlobal4ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamPointMomentPrincipal4ID = _ST7API.St7GetBeamPointMomentPrincipal4ID
St7GetBeamPointMomentPrincipal4ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamPointMomentGlobal4ID = _ST7API.St7GetBeamPointMomentGlobal4ID
St7GetBeamPointMomentGlobal4ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamDistributedForcePrincipal6ID = _ST7API.St7GetBeamDistributedForcePrincipal6ID
St7GetBeamDistributedForcePrincipal6ID.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamDistributedForceGlobal6ID = _ST7API.St7GetBeamDistributedForceGlobal6ID
St7GetBeamDistributedForceGlobal6ID.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamDistributedMomentPrincipal6ID = _ST7API.St7GetBeamDistributedMomentPrincipal6ID
St7GetBeamDistributedMomentPrincipal6ID.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamNSMass10ID = _ST7API.St7GetBeamNSMass10ID
St7GetBeamNSMass10ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamConvection2 = _ST7API.St7GetBeamConvection2
St7GetBeamConvection2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamConvectionTables = _ST7API.St7GetBeamConvectionTables
St7GetBeamConvectionTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetBeamRadiation2 = _ST7API.St7GetBeamRadiation2
St7GetBeamRadiation2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamRadiationTables = _ST7API.St7GetBeamRadiationTables
St7GetBeamRadiationTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetBeamFlux1 = _ST7API.St7GetBeamFlux1
St7GetBeamFlux1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamFluxTables = _ST7API.St7GetBeamFluxTables
St7GetBeamFluxTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetBeamHeatSource1 = _ST7API.St7GetBeamHeatSource1
St7GetBeamHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamHeatSourceTables = _ST7API.St7GetBeamHeatSourceTables
St7GetBeamHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetBeamResponse = _ST7API.St7GetBeamResponse
St7GetBeamResponse.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetBeamCreepLoadingAge1 = _ST7API.St7GetBeamCreepLoadingAge1
St7GetBeamCreepLoadingAge1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamEndAttachment1 = _ST7API.St7GetBeamEndAttachment1
St7GetBeamEndAttachment1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamSideAttachment1 = _ST7API.St7GetBeamSideAttachment1
St7GetBeamSideAttachment1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateID = _ST7API.St7SetPlateID
St7SetPlateID.argtypes = [c_long, c_long, c_long]
St7SetPlateXAngle1 = _ST7API.St7SetPlateXAngle1
St7SetPlateXAngle1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateThickness2 = _ST7API.St7SetPlateThickness2
St7SetPlateThickness2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateOffset1 = _ST7API.St7SetPlateOffset1
St7SetPlateOffset1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateEdgeSupport4 = _ST7API.St7SetPlateEdgeSupport4
St7SetPlateEdgeSupport4.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateFaceSupport4 = _ST7API.St7SetPlateFaceSupport4
St7SetPlateFaceSupport4.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateEdgeRelease1 = _ST7API.St7SetPlateEdgeRelease1
St7SetPlateEdgeRelease1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetPlatePreLoad3 = _ST7API.St7SetPlatePreLoad3
St7SetPlatePreLoad3.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlatePreCurvature2 = _ST7API.St7SetPlatePreCurvature2
St7SetPlatePreCurvature2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateTempGradient1 = _ST7API.St7SetPlateTempGradient1
St7SetPlateTempGradient1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlatePointForce6 = _ST7API.St7SetPlatePointForce6
St7SetPlatePointForce6.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlatePointMoment6 = _ST7API.St7SetPlatePointMoment6
St7SetPlatePointMoment6.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateEdgePressure1 = _ST7API.St7SetPlateEdgePressure1
St7SetPlateEdgePressure1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateEdgePressure3 = _ST7API.St7SetPlateEdgePressure3
St7SetPlateEdgePressure3.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateEdgeShear1 = _ST7API.St7SetPlateEdgeShear1
St7SetPlateEdgeShear1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateEdgeTransverseShear1 = _ST7API.St7SetPlateEdgeTransverseShear1
St7SetPlateEdgeTransverseShear1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateNormalPressure2 = _ST7API.St7SetPlateNormalPressure2
St7SetPlateNormalPressure2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateGlobalPressure3S = _ST7API.St7SetPlateGlobalPressure3S
St7SetPlateGlobalPressure3S.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateShear2 = _ST7API.St7SetPlateShear2
St7SetPlateShear2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateNSMass5ID = _ST7API.St7SetPlateNSMass5ID
St7SetPlateNSMass5ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateEdgeConvection2 = _ST7API.St7SetPlateEdgeConvection2
St7SetPlateEdgeConvection2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateEdgeConvectionTables = _ST7API.St7SetPlateEdgeConvectionTables
St7SetPlateEdgeConvectionTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetPlateEdgeRadiation2 = _ST7API.St7SetPlateEdgeRadiation2
St7SetPlateEdgeRadiation2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateEdgeRadiationTables = _ST7API.St7SetPlateEdgeRadiationTables
St7SetPlateEdgeRadiationTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetPlateFlux1 = _ST7API.St7SetPlateFlux1
St7SetPlateFlux1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateFluxTables = _ST7API.St7SetPlateFluxTables
St7SetPlateFluxTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetPlateFaceConvection2 = _ST7API.St7SetPlateFaceConvection2
St7SetPlateFaceConvection2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateFaceConvectionTables = _ST7API.St7SetPlateFaceConvectionTables
St7SetPlateFaceConvectionTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetPlateFaceRadiation2 = _ST7API.St7SetPlateFaceRadiation2
St7SetPlateFaceRadiation2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateFaceRadiationTables = _ST7API.St7SetPlateFaceRadiationTables
St7SetPlateFaceRadiationTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetPlateHeatSource1 = _ST7API.St7SetPlateHeatSource1
St7SetPlateHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateHeatSourceTables = _ST7API.St7SetPlateHeatSourceTables
St7SetPlateHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetPlateSoilStress2 = _ST7API.St7SetPlateSoilStress2
St7SetPlateSoilStress2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateSoilRatio2 = _ST7API.St7SetPlateSoilRatio2
St7SetPlateSoilRatio2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateResponse = _ST7API.St7SetPlateResponse
St7SetPlateResponse.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetPlateLoadPatch4 = _ST7API.St7SetPlateLoadPatch4
St7SetPlateLoadPatch4.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateReinforcement2 = _ST7API.St7SetPlateReinforcement2
St7SetPlateReinforcement2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateCreepLoadingAge1 = _ST7API.St7SetPlateCreepLoadingAge1
St7SetPlateCreepLoadingAge1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateEdgeAttachment1 = _ST7API.St7SetPlateEdgeAttachment1
St7SetPlateEdgeAttachment1.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateFaceAttachment1 = _ST7API.St7SetPlateFaceAttachment1
St7SetPlateFaceAttachment1.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateID = _ST7API.St7GetPlateID
St7GetPlateID.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetPlateXAngle1 = _ST7API.St7GetPlateXAngle1
St7GetPlateXAngle1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateThickness2 = _ST7API.St7GetPlateThickness2
St7GetPlateThickness2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateOffset1 = _ST7API.St7GetPlateOffset1
St7GetPlateOffset1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateEdgeSupport4 = _ST7API.St7GetPlateEdgeSupport4
St7GetPlateEdgeSupport4.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlateFaceSupport4 = _ST7API.St7GetPlateFaceSupport4
St7GetPlateFaceSupport4.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlateEdgeRelease1 = _ST7API.St7GetPlateEdgeRelease1
St7GetPlateEdgeRelease1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetPlatePreLoad3 = _ST7API.St7GetPlatePreLoad3
St7GetPlatePreLoad3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlatePreCurvature2 = _ST7API.St7GetPlatePreCurvature2
St7GetPlatePreCurvature2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateTempGradient1 = _ST7API.St7GetPlateTempGradient1
St7GetPlateTempGradient1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlatePointForce6 = _ST7API.St7GetPlatePointForce6
St7GetPlatePointForce6.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlatePointMoment6 = _ST7API.St7GetPlatePointMoment6
St7GetPlatePointMoment6.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateEdgePressure1 = _ST7API.St7GetPlateEdgePressure1
St7GetPlateEdgePressure1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateEdgePressure3 = _ST7API.St7GetPlateEdgePressure3
St7GetPlateEdgePressure3.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateEdgeShear1 = _ST7API.St7GetPlateEdgeShear1
St7GetPlateEdgeShear1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateEdgeTransverseShear1 = _ST7API.St7GetPlateEdgeTransverseShear1
St7GetPlateEdgeTransverseShear1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateNormalPressure2 = _ST7API.St7GetPlateNormalPressure2
St7GetPlateNormalPressure2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateGlobalPressure3S = _ST7API.St7GetPlateGlobalPressure3S
St7GetPlateGlobalPressure3S.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlateShear2 = _ST7API.St7GetPlateShear2
St7GetPlateShear2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateNSMass5ID = _ST7API.St7GetPlateNSMass5ID
St7GetPlateNSMass5ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateEdgeConvection2 = _ST7API.St7GetPlateEdgeConvection2
St7GetPlateEdgeConvection2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateEdgeConvectionTables = _ST7API.St7GetPlateEdgeConvectionTables
St7GetPlateEdgeConvectionTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetPlateEdgeRadiation2 = _ST7API.St7GetPlateEdgeRadiation2
St7GetPlateEdgeRadiation2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateEdgeRadiationTables = _ST7API.St7GetPlateEdgeRadiationTables
St7GetPlateEdgeRadiationTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetPlateFlux1 = _ST7API.St7GetPlateFlux1
St7GetPlateFlux1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateFluxTables = _ST7API.St7GetPlateFluxTables
St7GetPlateFluxTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetPlateFaceConvection2 = _ST7API.St7GetPlateFaceConvection2
St7GetPlateFaceConvection2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateFaceConvectionTables = _ST7API.St7GetPlateFaceConvectionTables
St7GetPlateFaceConvectionTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetPlateFaceRadiation2 = _ST7API.St7GetPlateFaceRadiation2
St7GetPlateFaceRadiation2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateFaceRadiationTables = _ST7API.St7GetPlateFaceRadiationTables
St7GetPlateFaceRadiationTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetPlateHeatSource1 = _ST7API.St7GetPlateHeatSource1
St7GetPlateHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateHeatSourceTables = _ST7API.St7GetPlateHeatSourceTables
St7GetPlateHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetPlateSoilStress2 = _ST7API.St7GetPlateSoilStress2
St7GetPlateSoilStress2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateSoilRatio2 = _ST7API.St7GetPlateSoilRatio2
St7GetPlateSoilRatio2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateResponse = _ST7API.St7GetPlateResponse
St7GetPlateResponse.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetPlateLoadPatch4 = _ST7API.St7GetPlateLoadPatch4
St7GetPlateLoadPatch4.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlateReinforcement2 = _ST7API.St7GetPlateReinforcement2
St7GetPlateReinforcement2.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlateCreepLoadingAge1 = _ST7API.St7GetPlateCreepLoadingAge1
St7GetPlateCreepLoadingAge1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateEdgeAttachment1 = _ST7API.St7GetPlateEdgeAttachment1
St7GetPlateEdgeAttachment1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlateFaceAttachment1 = _ST7API.St7GetPlateFaceAttachment1
St7GetPlateFaceAttachment1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBrickID = _ST7API.St7SetBrickID
St7SetBrickID.argtypes = [c_long, c_long, c_long]
St7SetBrickLocalAxes1 = _ST7API.St7SetBrickLocalAxes1
St7SetBrickLocalAxes1.argtypes = [c_long, c_long, c_long]
St7SetBrickSupport4 = _ST7API.St7SetBrickSupport4
St7SetBrickSupport4.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBrickPreLoad3 = _ST7API.St7SetBrickPreLoad3
St7SetBrickPreLoad3.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickPointForce6 = _ST7API.St7SetBrickPointForce6
St7SetBrickPointForce6.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickNormalPressure1 = _ST7API.St7SetBrickNormalPressure1
St7SetBrickNormalPressure1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickGlobalPressure3 = _ST7API.St7SetBrickGlobalPressure3
St7SetBrickGlobalPressure3.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickShear2 = _ST7API.St7SetBrickShear2
St7SetBrickShear2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickNSMass5ID = _ST7API.St7SetBrickNSMass5ID
St7SetBrickNSMass5ID.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickConvection2 = _ST7API.St7SetBrickConvection2
St7SetBrickConvection2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickConvectionTables = _ST7API.St7SetBrickConvectionTables
St7SetBrickConvectionTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetBrickRadiation2 = _ST7API.St7SetBrickRadiation2
St7SetBrickRadiation2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickRadiationTables = _ST7API.St7SetBrickRadiationTables
St7SetBrickRadiationTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetBrickFlux1 = _ST7API.St7SetBrickFlux1
St7SetBrickFlux1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickFluxTables = _ST7API.St7SetBrickFluxTables
St7SetBrickFluxTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetBrickHeatSource1 = _ST7API.St7SetBrickHeatSource1
St7SetBrickHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickHeatSourceTables = _ST7API.St7SetBrickHeatSourceTables
St7SetBrickHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetBrickSoilStress2 = _ST7API.St7SetBrickSoilStress2
St7SetBrickSoilStress2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickSoilRatio2 = _ST7API.St7SetBrickSoilRatio2
St7SetBrickSoilRatio2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickResponse = _ST7API.St7SetBrickResponse
St7SetBrickResponse.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetBrickCreepLoadingAge1 = _ST7API.St7SetBrickCreepLoadingAge1
St7SetBrickCreepLoadingAge1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickFaceAttachment1 = _ST7API.St7SetBrickFaceAttachment1
St7SetBrickFaceAttachment1.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickID = _ST7API.St7GetBrickID
St7GetBrickID.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetBrickLocalAxes1 = _ST7API.St7GetBrickLocalAxes1
St7GetBrickLocalAxes1.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetBrickSupport4 = _ST7API.St7GetBrickSupport4
St7GetBrickSupport4.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBrickPreLoad3 = _ST7API.St7GetBrickPreLoad3
St7GetBrickPreLoad3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBrickPointForce6 = _ST7API.St7GetBrickPointForce6
St7GetBrickPointForce6.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickNormalPressure1 = _ST7API.St7GetBrickNormalPressure1
St7GetBrickNormalPressure1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickGlobalPressure3 = _ST7API.St7GetBrickGlobalPressure3
St7GetBrickGlobalPressure3.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBrickShear2 = _ST7API.St7GetBrickShear2
St7GetBrickShear2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickNSMass5ID = _ST7API.St7GetBrickNSMass5ID
St7GetBrickNSMass5ID.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickConvection2 = _ST7API.St7GetBrickConvection2
St7GetBrickConvection2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickConvectionTables = _ST7API.St7GetBrickConvectionTables
St7GetBrickConvectionTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetBrickRadiation2 = _ST7API.St7GetBrickRadiation2
St7GetBrickRadiation2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickRadiationTables = _ST7API.St7GetBrickRadiationTables
St7GetBrickRadiationTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetBrickFlux1 = _ST7API.St7GetBrickFlux1
St7GetBrickFlux1.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickFluxTables = _ST7API.St7GetBrickFluxTables
St7GetBrickFluxTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetBrickHeatSource1 = _ST7API.St7GetBrickHeatSource1
St7GetBrickHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickHeatSourceTables = _ST7API.St7GetBrickHeatSourceTables
St7GetBrickHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetBrickSoilStress2 = _ST7API.St7GetBrickSoilStress2
St7GetBrickSoilStress2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickSoilRatio2 = _ST7API.St7GetBrickSoilRatio2
St7GetBrickSoilRatio2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickResponse = _ST7API.St7GetBrickResponse
St7GetBrickResponse.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetBrickCreepLoadingAge1 = _ST7API.St7GetBrickCreepLoadingAge1
St7GetBrickCreepLoadingAge1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickFaceAttachment1 = _ST7API.St7GetBrickFaceAttachment1
St7GetBrickFaceAttachment1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetLinkID = _ST7API.St7SetLinkID
St7SetLinkID.argtypes = [c_long, c_long, c_long]
St7GetLinkID = _ST7API.St7GetLinkID
St7GetLinkID.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetVertexType = _ST7API.St7SetVertexType
St7SetVertexType.argtypes = [c_long, c_long, c_long]
St7SetVertexID = _ST7API.St7SetVertexID
St7SetVertexID.argtypes = [c_long, c_long, c_long]
St7SetVertexMeshSize1 = _ST7API.St7SetVertexMeshSize1
St7SetVertexMeshSize1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetVertexRestraint6 = _ST7API.St7SetVertexRestraint6
St7SetVertexRestraint6.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetVertexForce3 = _ST7API.St7SetVertexForce3
St7SetVertexForce3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetVertexMoment3 = _ST7API.St7SetVertexMoment3
St7SetVertexMoment3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetVertexTemperature1 = _ST7API.St7SetVertexTemperature1
St7SetVertexTemperature1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetVertexTemperatureType1 = _ST7API.St7SetVertexTemperatureType1
St7SetVertexTemperatureType1.argtypes = [c_long, c_long, c_long, c_long]
St7SetVertexTemperatureTable = _ST7API.St7SetVertexTemperatureTable
St7SetVertexTemperatureTable.argtypes = [c_long, c_long, c_long, c_long]
St7SetVertexKTranslation3F = _ST7API.St7SetVertexKTranslation3F
St7SetVertexKTranslation3F.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetVertexKRotation3F = _ST7API.St7SetVertexKRotation3F
St7SetVertexKRotation3F.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetVertexTMass1 = _ST7API.St7SetVertexTMass1
St7SetVertexTMass1.argtypes = [c_long, c_long, c_double]
St7SetVertexTMass3 = _ST7API.St7SetVertexTMass3
St7SetVertexTMass3.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetVertexRMass3 = _ST7API.St7SetVertexRMass3
St7SetVertexRMass3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetVertexNSMass5ID = _ST7API.St7SetVertexNSMass5ID
St7SetVertexNSMass5ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetVertexKDamping3F = _ST7API.St7SetVertexKDamping3F
St7SetVertexKDamping3F.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetVertexHeatSource1 = _ST7API.St7SetVertexHeatSource1
St7SetVertexHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetVertexHeatSourceTables = _ST7API.St7SetVertexHeatSourceTables
St7SetVertexHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetVertexType = _ST7API.St7GetVertexType
St7GetVertexType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetVertexID = _ST7API.St7GetVertexID
St7GetVertexID.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetVertexMeshSize1 = _ST7API.St7GetVertexMeshSize1
St7GetVertexMeshSize1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetVertexRestraint6 = _ST7API.St7GetVertexRestraint6
St7GetVertexRestraint6.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetVertexForce3 = _ST7API.St7GetVertexForce3
St7GetVertexForce3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetVertexMoment3 = _ST7API.St7GetVertexMoment3
St7GetVertexMoment3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetVertexTemperature1 = _ST7API.St7GetVertexTemperature1
St7GetVertexTemperature1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetVertexTemperatureType1 = _ST7API.St7GetVertexTemperatureType1
St7GetVertexTemperatureType1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetVertexTemperatureTable = _ST7API.St7GetVertexTemperatureTable
St7GetVertexTemperatureTable.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetVertexKTranslation3F = _ST7API.St7GetVertexKTranslation3F
St7GetVertexKTranslation3F.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetVertexKRotation3F = _ST7API.St7GetVertexKRotation3F
St7GetVertexKRotation3F.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetVertexTMass3 = _ST7API.St7GetVertexTMass3
St7GetVertexTMass3.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetVertexRMass3 = _ST7API.St7GetVertexRMass3
St7GetVertexRMass3.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetVertexNSMass5ID = _ST7API.St7GetVertexNSMass5ID
St7GetVertexNSMass5ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetVertexKDamping3F = _ST7API.St7GetVertexKDamping3F
St7GetVertexKDamping3F.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetVertexHeatSource1 = _ST7API.St7GetVertexHeatSource1
St7GetVertexHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetVertexHeatSourceTables = _ST7API.St7GetVertexHeatSourceTables
St7GetVertexHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetGeometryEdgeType = _ST7API.St7SetGeometryEdgeType
St7SetGeometryEdgeType.argtypes = [c_long, c_long, c_long]
St7SetGeometryEdgeMinDivisions = _ST7API.St7SetGeometryEdgeMinDivisions
St7SetGeometryEdgeMinDivisions.argtypes = [c_long, c_long, c_long]
St7SetGeometryEdgeBeamProperty = _ST7API.St7SetGeometryEdgeBeamProperty
St7SetGeometryEdgeBeamProperty.argtypes = [c_long, c_long, c_long]
St7SetGeometryEdgeCluster = _ST7API.St7SetGeometryEdgeCluster
St7SetGeometryEdgeCluster.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryEdgeType = _ST7API.St7GetGeometryEdgeType
St7GetGeometryEdgeType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryEdgeMinDivisions = _ST7API.St7GetGeometryEdgeMinDivisions
St7GetGeometryEdgeMinDivisions.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryEdgeBeamProperty = _ST7API.St7GetGeometryEdgeBeamProperty
St7GetGeometryEdgeBeamProperty.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryEdgeCluster = _ST7API.St7GetGeometryEdgeCluster
St7GetGeometryEdgeCluster.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetGeometryCoedgeRelease1 = _ST7API.St7SetGeometryCoedgeRelease1
St7SetGeometryCoedgeRelease1.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetGeometryCoedgeSupport4 = _ST7API.St7SetGeometryCoedgeSupport4
St7SetGeometryCoedgeSupport4.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetGeometryCoedgePressure1 = _ST7API.St7SetGeometryCoedgePressure1
St7SetGeometryCoedgePressure1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryCoedgePressure3 = _ST7API.St7SetGeometryCoedgePressure3
St7SetGeometryCoedgePressure3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryCoedgeShear1 = _ST7API.St7SetGeometryCoedgeShear1
St7SetGeometryCoedgeShear1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryCoedgeTransverseShear1 = _ST7API.St7SetGeometryCoedgeTransverseShear1
St7SetGeometryCoedgeTransverseShear1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryCoedgeConvection2 = _ST7API.St7SetGeometryCoedgeConvection2
St7SetGeometryCoedgeConvection2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryCoedgeConvectionTables = _ST7API.St7SetGeometryCoedgeConvectionTables
St7SetGeometryCoedgeConvectionTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetGeometryCoedgeRadiation2 = _ST7API.St7SetGeometryCoedgeRadiation2
St7SetGeometryCoedgeRadiation2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryCoedgeRadiationTables = _ST7API.St7SetGeometryCoedgeRadiationTables
St7SetGeometryCoedgeRadiationTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetGeometryCoedgeFlux1 = _ST7API.St7SetGeometryCoedgeFlux1
St7SetGeometryCoedgeFlux1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryCoedgeFluxTables = _ST7API.St7SetGeometryCoedgeFluxTables
St7SetGeometryCoedgeFluxTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetGeometryCoedgeAttachment1 = _ST7API.St7SetGeometryCoedgeAttachment1
St7SetGeometryCoedgeAttachment1.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryCoedgeRelease1 = _ST7API.St7GetGeometryCoedgeRelease1
St7GetGeometryCoedgeRelease1.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryCoedgeSupport4 = _ST7API.St7GetGeometryCoedgeSupport4
St7GetGeometryCoedgeSupport4.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetGeometryCoedgePressure1 = _ST7API.St7GetGeometryCoedgePressure1
St7GetGeometryCoedgePressure1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryCoedgePressure3 = _ST7API.St7GetGeometryCoedgePressure3
St7GetGeometryCoedgePressure3.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryCoedgeShear1 = _ST7API.St7GetGeometryCoedgeShear1
St7GetGeometryCoedgeShear1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryCoedgeTransverseShear1 = _ST7API.St7GetGeometryCoedgeTransverseShear1
St7GetGeometryCoedgeTransverseShear1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryCoedgeConvection2 = _ST7API.St7GetGeometryCoedgeConvection2
St7GetGeometryCoedgeConvection2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryCoedgeConvectionTables = _ST7API.St7GetGeometryCoedgeConvectionTables
St7GetGeometryCoedgeConvectionTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryCoedgeRadiation2 = _ST7API.St7GetGeometryCoedgeRadiation2
St7GetGeometryCoedgeRadiation2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryCoedgeRadiationTables = _ST7API.St7GetGeometryCoedgeRadiationTables
St7GetGeometryCoedgeRadiationTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryCoedgeFlux1 = _ST7API.St7GetGeometryCoedgeFlux1
St7GetGeometryCoedgeFlux1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryCoedgeFluxTables = _ST7API.St7GetGeometryCoedgeFluxTables
St7GetGeometryCoedgeFluxTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryCoedgeAttachment1 = _ST7API.St7GetGeometryCoedgeAttachment1
St7GetGeometryCoedgeAttachment1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetGeometryFaceProperty = _ST7API.St7SetGeometryFaceProperty
St7SetGeometryFaceProperty.argtypes = [c_long, c_long, c_long]
St7SetGeometryFaceID = _ST7API.St7SetGeometryFaceID
St7SetGeometryFaceID.argtypes = [c_long, c_long, c_long]
St7SetGeometryFaceThickness2 = _ST7API.St7SetGeometryFaceThickness2
St7SetGeometryFaceThickness2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryFaceOffset1 = _ST7API.St7SetGeometryFaceOffset1
St7SetGeometryFaceOffset1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryFaceSupport4 = _ST7API.St7SetGeometryFaceSupport4
St7SetGeometryFaceSupport4.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetGeometryFaceTempGradient1 = _ST7API.St7SetGeometryFaceTempGradient1
St7SetGeometryFaceTempGradient1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryFaceNormalPressure2 = _ST7API.St7SetGeometryFaceNormalPressure2
St7SetGeometryFaceNormalPressure2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryFaceGlobalPressure3S = _ST7API.St7SetGeometryFaceGlobalPressure3S
St7SetGeometryFaceGlobalPressure3S.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryFaceNSMass5ID = _ST7API.St7SetGeometryFaceNSMass5ID
St7SetGeometryFaceNSMass5ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryFaceConvection2 = _ST7API.St7SetGeometryFaceConvection2
St7SetGeometryFaceConvection2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryFaceConvectionTables = _ST7API.St7SetGeometryFaceConvectionTables
St7SetGeometryFaceConvectionTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetGeometryFaceRadiation2 = _ST7API.St7SetGeometryFaceRadiation2
St7SetGeometryFaceRadiation2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryFaceRadiationTables = _ST7API.St7SetGeometryFaceRadiationTables
St7SetGeometryFaceRadiationTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetGeometryFaceHeatSource1 = _ST7API.St7SetGeometryFaceHeatSource1
St7SetGeometryFaceHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetGeometryFaceHeatSourceTables = _ST7API.St7SetGeometryFaceHeatSourceTables
St7SetGeometryFaceHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetGeometryFaceAttachment1 = _ST7API.St7SetGeometryFaceAttachment1
St7SetGeometryFaceAttachment1.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryFaceProperty = _ST7API.St7GetGeometryFaceProperty
St7GetGeometryFaceProperty.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryFaceID = _ST7API.St7GetGeometryFaceID
St7GetGeometryFaceID.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryFaceThickness2 = _ST7API.St7GetGeometryFaceThickness2
St7GetGeometryFaceThickness2.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryFaceOffset1 = _ST7API.St7GetGeometryFaceOffset1
St7GetGeometryFaceOffset1.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryFaceSupport4 = _ST7API.St7GetGeometryFaceSupport4
St7GetGeometryFaceSupport4.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetGeometryFaceTempGradient1 = _ST7API.St7GetGeometryFaceTempGradient1
St7GetGeometryFaceTempGradient1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryFaceNormalPressure2 = _ST7API.St7GetGeometryFaceNormalPressure2
St7GetGeometryFaceNormalPressure2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryFaceGlobalPressure3S = _ST7API.St7GetGeometryFaceGlobalPressure3S
St7GetGeometryFaceGlobalPressure3S.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetGeometryFaceNSMass5ID = _ST7API.St7GetGeometryFaceNSMass5ID
St7GetGeometryFaceNSMass5ID.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryFaceConvection2 = _ST7API.St7GetGeometryFaceConvection2
St7GetGeometryFaceConvection2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryFaceConvectionTables = _ST7API.St7GetGeometryFaceConvectionTables
St7GetGeometryFaceConvectionTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryFaceRadiation2 = _ST7API.St7GetGeometryFaceRadiation2
St7GetGeometryFaceRadiation2.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryFaceRadiationTables = _ST7API.St7GetGeometryFaceRadiationTables
St7GetGeometryFaceRadiationTables.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryFaceHeatSource1 = _ST7API.St7GetGeometryFaceHeatSource1
St7GetGeometryFaceHeatSource1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetGeometryFaceHeatSourceTables = _ST7API.St7GetGeometryFaceHeatSourceTables
St7GetGeometryFaceHeatSourceTables.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetGeometryFaceAttachment1 = _ST7API.St7GetGeometryFaceAttachment1
St7GetGeometryFaceAttachment1.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetElementProperty = _ST7API.St7SetElementProperty
St7SetElementProperty.argtypes = [c_long, c_long, c_long, c_long]
St7GetElementProperty = _ST7API.St7GetElementProperty
St7GetElementProperty.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetElementPropertySequence = _ST7API.St7GetElementPropertySequence
St7GetElementPropertySequence.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetElementPropertySwitch = _ST7API.St7SetElementPropertySwitch
St7SetElementPropertySwitch.argtypes = [c_long, c_long, c_long, c_long, c_long]
St7DeleteAttribute = _ST7API.St7DeleteAttribute
St7DeleteAttribute.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetEntityGroup = _ST7API.St7SetEntityGroup
St7SetEntityGroup.argtypes = [c_long, c_long, c_long, c_long]
St7GetEntityGroup = _ST7API.St7GetEntityGroup
St7GetEntityGroup.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetEntityAttributeSequenceCount = _ST7API.St7GetEntityAttributeSequenceCount
St7GetEntityAttributeSequenceCount.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetEntityAttributeSequence = _ST7API.St7GetEntityAttributeSequence
St7GetEntityAttributeSequence.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetMarker = _ST7API.St7SetMarker
St7SetMarker.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetMarker = _ST7API.St7GetMarker
St7GetMarker.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7DeleteMarker = _ST7API.St7DeleteMarker
St7DeleteMarker.argtypes = [c_long, c_long, c_long, c_long]
St7ShowMarker = _ST7API.St7ShowMarker
St7ShowMarker.argtypes = [c_long, c_long, c_long, c_long]
St7HideMarker = _ST7API.St7HideMarker
St7HideMarker.argtypes = [c_long, c_long, c_long, c_long]
St7GetTotalProperties = _ST7API.St7GetTotalProperties
St7GetTotalProperties.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetTotalCreepDefinitions = _ST7API.St7GetTotalCreepDefinitions
St7GetTotalCreepDefinitions.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetTotalLaminateStacks = _ST7API.St7GetTotalLaminateStacks
St7GetTotalLaminateStacks.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetTotalLoadPathTemplates = _ST7API.St7GetTotalLoadPathTemplates
St7GetTotalLoadPathTemplates.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetTotalReinforcementLayouts = _ST7API.St7GetTotalReinforcementLayouts
St7GetTotalReinforcementLayouts.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetPropertyNumByIndex = _ST7API.St7GetPropertyNumByIndex
St7GetPropertyNumByIndex.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7GetCreepDefinitionNumByIndex = _ST7API.St7GetCreepDefinitionNumByIndex
St7GetCreepDefinitionNumByIndex.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetLaminateStackNumByIndex = _ST7API.St7GetLaminateStackNumByIndex
St7GetLaminateStackNumByIndex.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetLoadPathTemplateNumByIndex = _ST7API.St7GetLoadPathTemplateNumByIndex
St7GetLoadPathTemplateNumByIndex.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetReinforcementLayoutNumByIndex = _ST7API.St7GetReinforcementLayoutNumByIndex
St7GetReinforcementLayoutNumByIndex.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetPropertyName = _ST7API.St7SetPropertyName
St7SetPropertyName.argtypes = [c_long, c_long, c_long, c_char_p]
St7GetPropertyName = _ST7API.St7GetPropertyName
St7GetPropertyName.argtypes = [c_long, c_long, c_long, c_char_p, c_long]
St7SetPropertyColour = _ST7API.St7SetPropertyColour
St7SetPropertyColour.argtypes = [c_long, c_long, c_long, c_long]
St7GetPropertyColour = _ST7API.St7GetPropertyColour
St7GetPropertyColour.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetPropertyTable = _ST7API.St7SetPropertyTable
St7SetPropertyTable.argtypes = [c_long, c_long, c_long, c_long]
St7GetPropertyTable = _ST7API.St7GetPropertyTable
St7GetPropertyTable.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetPropertyCreepID = _ST7API.St7SetPropertyCreepID
St7SetPropertyCreepID.argtypes = [c_long, c_long, c_long, c_long]
St7GetPropertyCreepID = _ST7API.St7GetPropertyCreepID
St7GetPropertyCreepID.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetPropertyRayleighFactors = _ST7API.St7SetPropertyRayleighFactors
St7SetPropertyRayleighFactors.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPropertyRayleighFactors = _ST7API.St7GetPropertyRayleighFactors
St7GetPropertyRayleighFactors.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetMaterialName = _ST7API.St7SetMaterialName
St7SetMaterialName.argtypes = [c_long, c_long, c_long, c_char_p]
St7GetMaterialName = _ST7API.St7GetMaterialName
St7GetMaterialName.argtypes = [c_long, c_long, c_long, c_char_p, c_long]
St7SetTimeDependentModType = _ST7API.St7SetTimeDependentModType
St7SetTimeDependentModType.argtypes = [c_long, c_long, c_long, c_long]
St7GetTimeDependentModType = _ST7API.St7GetTimeDependentModType
St7GetTimeDependentModType.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetHardeningType = _ST7API.St7SetHardeningType
St7SetHardeningType.argtypes = [c_long, c_long, c_long, c_long]
St7GetHardeningType = _ST7API.St7GetHardeningType
St7GetHardeningType.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetAlphaTempType = _ST7API.St7SetAlphaTempType
St7SetAlphaTempType.argtypes = [c_long, c_long, c_long, c_long]
St7GetAlphaTempType = _ST7API.St7GetAlphaTempType
St7GetAlphaTempType.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7NewBeamProperty = _ST7API.St7NewBeamProperty
St7NewBeamProperty.argtypes = [c_long, c_long, c_long, c_char_p]
St7SetBeamSectionName = _ST7API.St7SetBeamSectionName
St7SetBeamSectionName.argtypes = [c_long, c_long, c_char_p]
St7GetBeamSectionName = _ST7API.St7GetBeamSectionName
St7GetBeamSectionName.argtypes = [c_long, c_long, c_char_p, c_long]
St7SetBeamPropertyType = _ST7API.St7SetBeamPropertyType
St7SetBeamPropertyType.argtypes = [c_long, c_long, c_long]
St7GetBeamPropertyType = _ST7API.St7GetBeamPropertyType
St7GetBeamPropertyType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetBeamMirrorOption = _ST7API.St7SetBeamMirrorOption
St7SetBeamMirrorOption.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamMirrorOption = _ST7API.St7GetBeamMirrorOption
St7GetBeamMirrorOption.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBeamNonlinearType = _ST7API.St7SetBeamNonlinearType
St7SetBeamNonlinearType.argtypes = [c_long, c_long, c_long, c_long]
St7GetBeamNonlinearType = _ST7API.St7GetBeamNonlinearType
St7GetBeamNonlinearType.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetBeamSectionPropertyData = _ST7API.St7SetBeamSectionPropertyData
St7SetBeamSectionPropertyData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamSectionPropertyData = _ST7API.St7GetBeamSectionPropertyData
St7GetBeamSectionPropertyData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBeamSectionGeometry = _ST7API.St7SetBeamSectionGeometry
St7SetBeamSectionGeometry.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamSectionGeometry = _ST7API.St7GetBeamSectionGeometry
St7GetBeamSectionGeometry.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBeamSectionNominalDiscretisation = _ST7API.St7SetBeamSectionNominalDiscretisation
St7SetBeamSectionNominalDiscretisation.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetBeamSectionNominalDiscretisation = _ST7API.St7GetBeamSectionNominalDiscretisation
St7GetBeamSectionNominalDiscretisation.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetBeamSectionCircularDiscretisation = _ST7API.St7SetBeamSectionCircularDiscretisation
St7SetBeamSectionCircularDiscretisation.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetBeamSectionCircularDiscretisation = _ST7API.St7GetBeamSectionCircularDiscretisation
St7GetBeamSectionCircularDiscretisation.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetBeamPropertyData = _ST7API.St7GetBeamPropertyData
St7GetBeamPropertyData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7CalculateBeamSectionProperties = _ST7API.St7CalculateBeamSectionProperties
St7CalculateBeamSectionProperties.argtypes = [c_long, c_long, c_bool]
St7AssignBXS = _ST7API.St7AssignBXS
St7AssignBXS.argtypes = [c_long, c_long, c_char_p]
St7SaveBeamSectionMesh = _ST7API.St7SaveBeamSectionMesh
St7SaveBeamSectionMesh.argtypes = [c_long, c_long, c_char_p]
St7SetSpringDamperData = _ST7API.St7SetSpringDamperData
St7SetSpringDamperData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetSpringDamperData = _ST7API.St7GetSpringDamperData
St7GetSpringDamperData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetTrussData = _ST7API.St7SetTrussData
St7SetTrussData.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetTrussData = _ST7API.St7GetTrussData
St7GetTrussData.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetCableData = _ST7API.St7SetCableData
St7SetCableData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetCableData = _ST7API.St7GetCableData
St7GetCableData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetCutoffBarData = _ST7API.St7SetCutoffBarData
St7SetCutoffBarData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetCutoffBarData = _ST7API.St7GetCutoffBarData
St7GetCutoffBarData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPointContactData = _ST7API.St7SetPointContactData
St7SetPointContactData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPointContactData = _ST7API.St7GetPointContactData
St7GetPointContactData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPipeData = _ST7API.St7SetPipeData
St7SetPipeData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPipeData = _ST7API.St7GetPipeData
St7GetPipeData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetConnectionData = _ST7API.St7SetConnectionData
St7SetConnectionData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetConnectionData = _ST7API.St7GetConnectionData
St7GetConnectionData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetUserBeamData = _ST7API.St7SetUserBeamData
St7SetUserBeamData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetUserBeamData = _ST7API.St7GetUserBeamData
St7GetUserBeamData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7CheckBeamSectionQuality = _ST7API.St7CheckBeamSectionQuality
St7CheckBeamSectionQuality.argtypes = [c_long, c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7SetSpringDamperThermalData = _ST7API.St7SetSpringDamperThermalData
St7SetSpringDamperThermalData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetSpringDamperThermalData = _ST7API.St7GetSpringDamperThermalData
St7GetSpringDamperThermalData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPointContactThermalData = _ST7API.St7SetPointContactThermalData
St7SetPointContactThermalData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPointContactThermalData = _ST7API.St7GetPointContactThermalData
St7GetPointContactThermalData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetUserBeamThermalData = _ST7API.St7SetUserBeamThermalData
St7SetUserBeamThermalData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetUserBeamThermalData = _ST7API.St7GetUserBeamThermalData
St7GetUserBeamThermalData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetConnectionThermalData = _ST7API.St7SetConnectionThermalData
St7SetConnectionThermalData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetConnectionThermalData = _ST7API.St7GetConnectionThermalData
St7GetConnectionThermalData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamMaterialData = _ST7API.St7SetBeamMaterialData
St7SetBeamMaterialData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamMaterialData = _ST7API.St7GetBeamMaterialData
St7GetBeamMaterialData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBeamUsePoisson = _ST7API.St7SetBeamUsePoisson
St7SetBeamUsePoisson.argtypes = [c_long, c_long]
St7SetBeamUseShearMod = _ST7API.St7SetBeamUseShearMod
St7SetBeamUseShearMod.argtypes = [c_long, c_long]
St7SetBeamUseMomCurv = _ST7API.St7SetBeamUseMomCurv
St7SetBeamUseMomCurv.argtypes = [c_long, c_long, c_bool]
St7GetBeamUseMomCurv = _ST7API.St7GetBeamUseMomCurv
St7GetBeamUseMomCurv.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7NewPlateProperty = _ST7API.St7NewPlateProperty
St7NewPlateProperty.argtypes = [c_long, c_long, c_long, c_long, c_char_p]
St7SetPlatePropertyType = _ST7API.St7SetPlatePropertyType
St7SetPlatePropertyType.argtypes = [c_long, c_long, c_long, c_long]
St7GetPlatePropertyType = _ST7API.St7GetPlatePropertyType
St7GetPlatePropertyType.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetPlateNonlinearType = _ST7API.St7SetPlateNonlinearType
St7SetPlateNonlinearType.argtypes = [c_long, c_long, c_long, c_long]
St7GetPlateNonlinearType = _ST7API.St7GetPlateNonlinearType
St7GetPlateNonlinearType.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetPlateThickness = _ST7API.St7SetPlateThickness
St7SetPlateThickness.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateThickness = _ST7API.St7GetPlateThickness
St7GetPlateThickness.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateLayers = _ST7API.St7SetPlateLayers
St7SetPlateLayers.argtypes = [c_long, c_long, c_long]
St7GetPlateLayers = _ST7API.St7GetPlateLayers
St7GetPlateLayers.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetPlatePatchTol = _ST7API.St7SetPlatePatchTol
St7SetPlatePatchTol.argtypes = [c_long, c_long, c_double]
St7GetPlatePatchTol = _ST7API.St7GetPlatePatchTol
St7GetPlatePatchTol.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateIsotropicMaterial = _ST7API.St7SetPlateIsotropicMaterial
St7SetPlateIsotropicMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateIsotropicMaterial = _ST7API.St7GetPlateIsotropicMaterial
St7GetPlateIsotropicMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateOrthotropicMaterial = _ST7API.St7SetPlateOrthotropicMaterial
St7SetPlateOrthotropicMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateOrthotropicMaterial = _ST7API.St7GetPlateOrthotropicMaterial
St7GetPlateOrthotropicMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateRubberMaterial = _ST7API.St7SetPlateRubberMaterial
St7SetPlateRubberMaterial.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateRubberMaterial = _ST7API.St7GetPlateRubberMaterial
St7GetPlateRubberMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateAnisotropicMaterial = _ST7API.St7SetPlateAnisotropicMaterial
St7SetPlateAnisotropicMaterial.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateAnisotropicMaterial = _ST7API.St7GetPlateAnisotropicMaterial
St7GetPlateAnisotropicMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateLaminateMaterial = _ST7API.St7SetPlateLaminateMaterial
St7SetPlateLaminateMaterial.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateLaminateMaterial = _ST7API.St7GetPlateLaminateMaterial
St7GetPlateLaminateMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateUserDefinedMaterial = _ST7API.St7SetPlateUserDefinedMaterial
St7SetPlateUserDefinedMaterial.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateUserDefinedMaterial = _ST7API.St7GetPlateUserDefinedMaterial
St7GetPlateUserDefinedMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateMCDPMaterial = _ST7API.St7SetPlateMCDPMaterial
St7SetPlateMCDPMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateMCDPMaterial = _ST7API.St7GetPlateMCDPMaterial
St7GetPlateMCDPMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateSoilDCMaterial = _ST7API.St7SetPlateSoilDCMaterial
St7SetPlateSoilDCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlateSoilDCMaterial = _ST7API.St7GetPlateSoilDCMaterial
St7GetPlateSoilDCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateSoilCCMaterial = _ST7API.St7SetPlateSoilCCMaterial
St7SetPlateSoilCCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlateSoilCCMaterial = _ST7API.St7GetPlateSoilCCMaterial
St7GetPlateSoilCCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateSoilMCMaterial = _ST7API.St7SetPlateSoilMCMaterial
St7SetPlateSoilMCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlateSoilMCMaterial = _ST7API.St7GetPlateSoilMCMaterial
St7GetPlateSoilMCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateSoilDPMaterial = _ST7API.St7SetPlateSoilDPMaterial
St7SetPlateSoilDPMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlateSoilDPMaterial = _ST7API.St7GetPlateSoilDPMaterial
St7GetPlateSoilDPMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateSoilLSMaterial = _ST7API.St7SetPlateSoilLSMaterial
St7SetPlateSoilLSMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlateSoilLSMaterial = _ST7API.St7GetPlateSoilLSMaterial
St7GetPlateSoilLSMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateFluidMaterial = _ST7API.St7SetPlateFluidMaterial
St7SetPlateFluidMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetPlateFluidMaterial = _ST7API.St7GetPlateFluidMaterial
St7GetPlateFluidMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetPlateUseReducedInt = _ST7API.St7SetPlateUseReducedInt
St7SetPlateUseReducedInt.argtypes = [c_long, c_long, c_bool]
St7GetPlateUseReducedInt = _ST7API.St7GetPlateUseReducedInt
St7GetPlateUseReducedInt.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7NewBrickProperty = _ST7API.St7NewBrickProperty
St7NewBrickProperty.argtypes = [c_long, c_long, c_long, c_char_p]
St7SetBrickPropertyType = _ST7API.St7SetBrickPropertyType
St7SetBrickPropertyType.argtypes = [c_long, c_long, c_long]
St7GetBrickPropertyType = _ST7API.St7GetBrickPropertyType
St7GetBrickPropertyType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetBrickNonlinearType = _ST7API.St7SetBrickNonlinearType
St7SetBrickNonlinearType.argtypes = [c_long, c_long, c_long, c_long]
St7GetBrickNonlinearType = _ST7API.St7GetBrickNonlinearType
St7GetBrickNonlinearType.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetBrickIsotropicMaterial = _ST7API.St7SetBrickIsotropicMaterial
St7SetBrickIsotropicMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickIsotropicMaterial = _ST7API.St7GetBrickIsotropicMaterial
St7GetBrickIsotropicMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickOrthotropicMaterial = _ST7API.St7SetBrickOrthotropicMaterial
St7SetBrickOrthotropicMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickOrthotropicMaterial = _ST7API.St7GetBrickOrthotropicMaterial
St7GetBrickOrthotropicMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickAnisotropicMaterial = _ST7API.St7SetBrickAnisotropicMaterial
St7SetBrickAnisotropicMaterial.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickAnisotropicMaterial = _ST7API.St7GetBrickAnisotropicMaterial
St7GetBrickAnisotropicMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBrickRubberMaterial = _ST7API.St7SetBrickRubberMaterial
St7SetBrickRubberMaterial.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickRubberMaterial = _ST7API.St7GetBrickRubberMaterial
St7GetBrickRubberMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBrickMCDPMaterial = _ST7API.St7SetBrickMCDPMaterial
St7SetBrickMCDPMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickMCDPMaterial = _ST7API.St7GetBrickMCDPMaterial
St7GetBrickMCDPMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickSoilDCMaterial = _ST7API.St7SetBrickSoilDCMaterial
St7SetBrickSoilDCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBrickSoilDCMaterial = _ST7API.St7GetBrickSoilDCMaterial
St7GetBrickSoilDCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBrickSoilCCMaterial = _ST7API.St7SetBrickSoilCCMaterial
St7SetBrickSoilCCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBrickSoilCCMaterial = _ST7API.St7GetBrickSoilCCMaterial
St7GetBrickSoilCCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBrickSoilMCMaterial = _ST7API.St7SetBrickSoilMCMaterial
St7SetBrickSoilMCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBrickSoilMCMaterial = _ST7API.St7GetBrickSoilMCMaterial
St7GetBrickSoilMCMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBrickSoilDPMaterial = _ST7API.St7SetBrickSoilDPMaterial
St7SetBrickSoilDPMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBrickSoilDPMaterial = _ST7API.St7GetBrickSoilDPMaterial
St7GetBrickSoilDPMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBrickSoilLSMaterial = _ST7API.St7SetBrickSoilLSMaterial
St7SetBrickSoilLSMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBrickSoilLSMaterial = _ST7API.St7GetBrickSoilLSMaterial
St7GetBrickSoilLSMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetBrickFluidMaterial = _ST7API.St7SetBrickFluidMaterial
St7SetBrickFluidMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickFluidMaterial = _ST7API.St7GetBrickFluidMaterial
St7GetBrickFluidMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetBrickAddBubbleFunction = _ST7API.St7SetBrickAddBubbleFunction
St7SetBrickAddBubbleFunction.argtypes = [c_long, c_long, c_bool]
St7GetBrickAddBubbleFunction = _ST7API.St7GetBrickAddBubbleFunction
St7GetBrickAddBubbleFunction.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetBrickIntegrationPoints = _ST7API.St7SetBrickIntegrationPoints
St7SetBrickIntegrationPoints.argtypes = [c_long, c_long, c_long, c_long, c_long]
St7GetBrickIntegrationPoints = _ST7API.St7GetBrickIntegrationPoints
St7GetBrickIntegrationPoints.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7DeleteProperty = _ST7API.St7DeleteProperty
St7DeleteProperty.argtypes = [c_long, c_long, c_long]
St7DeleteUnusedProperties = _ST7API.St7DeleteUnusedProperties
St7DeleteUnusedProperties.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7UpdateElementPropertyData = _ST7API.St7UpdateElementPropertyData
St7UpdateElementPropertyData.argtypes = [c_long, c_long, c_long]
St7NewPlyProperty = _ST7API.St7NewPlyProperty
St7NewPlyProperty.argtypes = [c_long, c_long, c_char_p]
St7SetPlyMaterial = _ST7API.St7SetPlyMaterial
St7SetPlyMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetPlyMaterial = _ST7API.St7GetPlyMaterial
St7GetPlyMaterial.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7NewLaminate = _ST7API.St7NewLaminate
St7NewLaminate.argtypes = [c_long, c_long, c_char_p]
St7SetLaminateName = _ST7API.St7SetLaminateName
St7SetLaminateName.argtypes = [c_long, c_long, c_char_p]
St7GetLaminateName = _ST7API.St7GetLaminateName
St7GetLaminateName.argtypes = [c_long, c_long, c_char_p, c_long]
St7GetLaminateNumPlies = _ST7API.St7GetLaminateNumPlies
St7GetLaminateNumPlies.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetLaminatePly = _ST7API.St7SetLaminatePly
St7SetLaminatePly.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetLaminatePly = _ST7API.St7GetLaminatePly
St7GetLaminatePly.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7AddLaminatePly = _ST7API.St7AddLaminatePly
St7AddLaminatePly.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7DeleteLaminatePly = _ST7API.St7DeleteLaminatePly
St7DeleteLaminatePly.argtypes = [c_long, c_long, c_long]
St7InsertLaminatePly = _ST7API.St7InsertLaminatePly
St7InsertLaminatePly.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetLaminateMatrices = _ST7API.St7SetLaminateMatrices
St7SetLaminateMatrices.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetLaminateMatrices = _ST7API.St7GetLaminateMatrices
St7GetLaminateMatrices.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7DeleteLaminate = _ST7API.St7DeleteLaminate
St7DeleteLaminate.argtypes = [c_long, c_long]
St7DeleteUnusedLaminates = _ST7API.St7DeleteUnusedLaminates
St7DeleteUnusedLaminates.argtypes = [c_long, ctypes.POINTER(c_long)]
St7NewReinforcementLayout = _ST7API.St7NewReinforcementLayout
St7NewReinforcementLayout.argtypes = [c_long, c_long, c_char_p]
St7SetReinforcementName = _ST7API.St7SetReinforcementName
St7SetReinforcementName.argtypes = [c_long, c_long, c_char_p]
St7GetReinforcementName = _ST7API.St7GetReinforcementName
St7GetReinforcementName.argtypes = [c_long, c_long, c_char_p, c_long]
St7SetReinforcementData = _ST7API.St7SetReinforcementData
St7SetReinforcementData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetReinforcementData = _ST7API.St7GetReinforcementData
St7GetReinforcementData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7DeleteReinforcementLayout = _ST7API.St7DeleteReinforcementLayout
St7DeleteReinforcementLayout.argtypes = [c_long, c_long]
St7NewCreepDefinition = _ST7API.St7NewCreepDefinition
St7NewCreepDefinition.argtypes = [c_long, c_long, c_char_p]
St7SetCreepDefinitionName = _ST7API.St7SetCreepDefinitionName
St7SetCreepDefinitionName.argtypes = [c_long, c_long, c_char_p]
St7GetCreepDefinitionName = _ST7API.St7GetCreepDefinitionName
St7GetCreepDefinitionName.argtypes = [c_long, c_long, c_char_p, c_long]
St7SetCreepLaw = _ST7API.St7SetCreepLaw
St7SetCreepLaw.argtypes = [c_long, c_long, c_long]
St7GetCreepLaw = _ST7API.St7GetCreepLaw
St7GetCreepLaw.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetCreepBasicData = _ST7API.St7SetCreepBasicData
St7SetCreepBasicData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetCreepBasicData = _ST7API.St7GetCreepBasicData
St7GetCreepBasicData.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7EnableCreepUserTable = _ST7API.St7EnableCreepUserTable
St7EnableCreepUserTable.argtypes = [c_long, c_long, c_long]
St7DisableCreepUserTable = _ST7API.St7DisableCreepUserTable
St7DisableCreepUserTable.argtypes = [c_long, c_long, c_long]
St7GetCreepUserTableState = _ST7API.St7GetCreepUserTableState
St7GetCreepUserTableState.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7SetCreepUserTableData = _ST7API.St7SetCreepUserTableData
St7SetCreepUserTableData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetCreepUserTableData = _ST7API.St7GetCreepUserTableData
St7GetCreepUserTableData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetCreepHardeningType = _ST7API.St7SetCreepHardeningType
St7SetCreepHardeningType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetCreepHardeningType = _ST7API.St7GetCreepHardeningType
St7GetCreepHardeningType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetCreepTimeUnit = _ST7API.St7SetCreepTimeUnit
St7SetCreepTimeUnit.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetCreepTimeUnit = _ST7API.St7GetCreepTimeUnit
St7GetCreepTimeUnit.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetCreepTemperatureInclude = _ST7API.St7SetCreepTemperatureInclude
St7SetCreepTemperatureInclude.argtypes = [c_long, c_long, c_bool]
St7GetCreepTemperatureInclude = _ST7API.St7GetCreepTemperatureInclude
St7GetCreepTemperatureInclude.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetCreepConcreteHyperbolicData = _ST7API.St7SetCreepConcreteHyperbolicData
St7SetCreepConcreteHyperbolicData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetCreepConcreteHyperbolicData = _ST7API.St7GetCreepConcreteHyperbolicData
St7GetCreepConcreteHyperbolicData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetCreepConcreteViscoChainData = _ST7API.St7SetCreepConcreteViscoChainData
St7SetCreepConcreteViscoChainData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetCreepConcreteViscoChainData = _ST7API.St7GetCreepConcreteViscoChainData
St7GetCreepConcreteViscoChainData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7EnableCreepConcreteUserTable = _ST7API.St7EnableCreepConcreteUserTable
St7EnableCreepConcreteUserTable.argtypes = [c_long, c_long, c_long]
St7DisableCreepConcreteUserTable = _ST7API.St7DisableCreepConcreteUserTable
St7DisableCreepConcreteUserTable.argtypes = [c_long, c_long, c_long]
St7GetCreepConcreteUserTableState = _ST7API.St7GetCreepConcreteUserTableState
St7GetCreepConcreteUserTableState.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7SetCreepConcreteUserTableData = _ST7API.St7SetCreepConcreteUserTableData
St7SetCreepConcreteUserTableData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetCreepConcreteUserTableData = _ST7API.St7GetCreepConcreteUserTableData
St7GetCreepConcreteUserTableData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetCreepConcreteFunctionType = _ST7API.St7SetCreepConcreteFunctionType
St7SetCreepConcreteFunctionType.argtypes = [c_long, c_long, c_long]
St7GetCreepConcreteFunctionType = _ST7API.St7GetCreepConcreteFunctionType
St7GetCreepConcreteFunctionType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetCreepConcreteLoadingAge = _ST7API.St7SetCreepConcreteLoadingAge
St7SetCreepConcreteLoadingAge.argtypes = [c_long, c_long, c_double]
St7GetCreepConcreteLoadingAge = _ST7API.St7GetCreepConcreteLoadingAge
St7GetCreepConcreteLoadingAge.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetCreepConcreteLoadingTimeUnit = _ST7API.St7SetCreepConcreteLoadingTimeUnit
St7SetCreepConcreteLoadingTimeUnit.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetCreepConcreteLoadingTimeUnit = _ST7API.St7GetCreepConcreteLoadingTimeUnit
St7GetCreepConcreteLoadingTimeUnit.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetCreepConcreteShrinkageType = _ST7API.St7SetCreepConcreteShrinkageType
St7SetCreepConcreteShrinkageType.argtypes = [c_long, c_long, c_long]
St7GetCreepConcreteShrinkageType = _ST7API.St7GetCreepConcreteShrinkageType
St7GetCreepConcreteShrinkageType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetCreepConcreteShrinkageFormulaData = _ST7API.St7SetCreepConcreteShrinkageFormulaData
St7SetCreepConcreteShrinkageFormulaData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetCreepConcreteShrinkageFormulaData = _ST7API.St7GetCreepConcreteShrinkageFormulaData
St7GetCreepConcreteShrinkageFormulaData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetCreepConcreteShrinkageTableData = _ST7API.St7SetCreepConcreteShrinkageTableData
St7SetCreepConcreteShrinkageTableData.argtypes = [c_long, c_long, c_long]
St7GetCreepConcreteShrinkageTableData = _ST7API.St7GetCreepConcreteShrinkageTableData
St7GetCreepConcreteShrinkageTableData.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetCreepConcreteTemperatureData = _ST7API.St7SetCreepConcreteTemperatureData
St7SetCreepConcreteTemperatureData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetCreepConcreteTemperatureData = _ST7API.St7GetCreepConcreteTemperatureData
St7GetCreepConcreteTemperatureData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetCreepConcreteCementCuringData = _ST7API.St7SetCreepConcreteCementCuringData
St7SetCreepConcreteCementCuringData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetCreepConcreteCementCuringData = _ST7API.St7GetCreepConcreteCementCuringData
St7GetCreepConcreteCementCuringData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7DeleteCreepDefinition = _ST7API.St7DeleteCreepDefinition
St7DeleteCreepDefinition.argtypes = [c_long, c_long]
St7NewLoadPathTemplate = _ST7API.St7NewLoadPathTemplate
St7NewLoadPathTemplate.argtypes = [c_long, c_long, c_char_p]
St7SetLoadPathTemplateName = _ST7API.St7SetLoadPathTemplateName
St7SetLoadPathTemplateName.argtypes = [c_long, c_long, c_char_p]
St7GetLoadPathTemplateName = _ST7API.St7GetLoadPathTemplateName
St7GetLoadPathTemplateName.argtypes = [c_long, c_long, c_char_p, c_long]
St7SetLoadPathTemplateParameters = _ST7API.St7SetLoadPathTemplateParameters
St7SetLoadPathTemplateParameters.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetLoadPathTemplateParameters = _ST7API.St7GetLoadPathTemplateParameters
St7GetLoadPathTemplateParameters.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetLoadPathTemplateLaneFactor = _ST7API.St7SetLoadPathTemplateLaneFactor
St7SetLoadPathTemplateLaneFactor.argtypes = [c_long, c_long, c_long, c_double]
St7GetLoadPathTemplateLaneFactor = _ST7API.St7GetLoadPathTemplateLaneFactor
St7GetLoadPathTemplateLaneFactor.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7AddLoadPathTemplateVehicle = _ST7API.St7AddLoadPathTemplateVehicle
St7AddLoadPathTemplateVehicle.argtypes = [c_long, c_long]
St7SetLoadPathTemplateVehicleName = _ST7API.St7SetLoadPathTemplateVehicleName
St7SetLoadPathTemplateVehicleName.argtypes = [c_long, c_long, c_long, c_char_p]
St7GetLoadPathTemplateVehicleName = _ST7API.St7GetLoadPathTemplateVehicleName
St7GetLoadPathTemplateVehicleName.argtypes = [c_long, c_long, c_long, c_char_p, c_long]
St7InsertLoadPathTemplateVehicle = _ST7API.St7InsertLoadPathTemplateVehicle
St7InsertLoadPathTemplateVehicle.argtypes = [c_long, c_long, c_long]
St7CloneLoadPathTemplateVehicle = _ST7API.St7CloneLoadPathTemplateVehicle
St7CloneLoadPathTemplateVehicle.argtypes = [c_long, c_long, c_long]
St7DeleteLoadPathTemplateVehicle = _ST7API.St7DeleteLoadPathTemplateVehicle
St7DeleteLoadPathTemplateVehicle.argtypes = [c_long, c_long, c_long]
St7GetNumLoadPathTemplateVehicles = _ST7API.St7GetNumLoadPathTemplateVehicles
St7GetNumLoadPathTemplateVehicles.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetLoadPathTemplateVehicleData = _ST7API.St7SetLoadPathTemplateVehicleData
St7SetLoadPathTemplateVehicleData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetLoadPathTemplateVehicleData = _ST7API.St7GetLoadPathTemplateVehicleData
St7GetLoadPathTemplateVehicleData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7EnableLoadPathTemplateVehicleLane = _ST7API.St7EnableLoadPathTemplateVehicleLane
St7EnableLoadPathTemplateVehicleLane.argtypes = [c_long, c_long, c_long, c_long]
St7DisableLoadPathTemplateVehicleLane = _ST7API.St7DisableLoadPathTemplateVehicleLane
St7DisableLoadPathTemplateVehicleLane.argtypes = [c_long, c_long, c_long, c_long]
St7GetLoadPathTemplateVehicleLaneState = _ST7API.St7GetLoadPathTemplateVehicleLaneState
St7GetLoadPathTemplateVehicleLaneState.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7AddLoadPathTemplatePointForce = _ST7API.St7AddLoadPathTemplatePointForce
St7AddLoadPathTemplatePointForce.argtypes = [c_long, c_long, c_long]
St7InsertLoadPathTemplatePointForce = _ST7API.St7InsertLoadPathTemplatePointForce
St7InsertLoadPathTemplatePointForce.argtypes = [c_long, c_long, c_long, c_long]
St7DeleteLoadPathTemplatePointForce = _ST7API.St7DeleteLoadPathTemplatePointForce
St7DeleteLoadPathTemplatePointForce.argtypes = [c_long, c_long, c_long, c_long]
St7GetNumLoadPathTemplatePointForces = _ST7API.St7GetNumLoadPathTemplatePointForces
St7GetNumLoadPathTemplatePointForces.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetLoadPathTemplatePointForceData = _ST7API.St7SetLoadPathTemplatePointForceData
St7SetLoadPathTemplatePointForceData.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetLoadPathTemplatePointForceData = _ST7API.St7GetLoadPathTemplatePointForceData
St7GetLoadPathTemplatePointForceData.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7AddLoadPathTemplateDistributedForce = _ST7API.St7AddLoadPathTemplateDistributedForce
St7AddLoadPathTemplateDistributedForce.argtypes = [c_long, c_long, c_long]
St7InsertLoadPathTemplateDistributedForce = _ST7API.St7InsertLoadPathTemplateDistributedForce
St7InsertLoadPathTemplateDistributedForce.argtypes = [c_long, c_long, c_long, c_long]
St7DeleteLoadPathTemplateDistributedForce = _ST7API.St7DeleteLoadPathTemplateDistributedForce
St7DeleteLoadPathTemplateDistributedForce.argtypes = [c_long, c_long, c_long, c_long]
St7GetNumLoadPathTemplateDistributedForces = _ST7API.St7GetNumLoadPathTemplateDistributedForces
St7GetNumLoadPathTemplateDistributedForces.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetLoadPathTemplateDistributedForceData = _ST7API.St7SetLoadPathTemplateDistributedForceData
St7SetLoadPathTemplateDistributedForceData.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetLoadPathTemplateDistributedForceData = _ST7API.St7GetLoadPathTemplateDistributedForceData
St7GetLoadPathTemplateDistributedForceData.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7AddLoadPathTemplateHeatSource = _ST7API.St7AddLoadPathTemplateHeatSource
St7AddLoadPathTemplateHeatSource.argtypes = [c_long, c_long, c_long]
St7InsertLoadPathTemplateHeatSource = _ST7API.St7InsertLoadPathTemplateHeatSource
St7InsertLoadPathTemplateHeatSource.argtypes = [c_long, c_long, c_long, c_long]
St7DeleteLoadPathTemplateHeatSource = _ST7API.St7DeleteLoadPathTemplateHeatSource
St7DeleteLoadPathTemplateHeatSource.argtypes = [c_long, c_long, c_long, c_long]
St7GetNumLoadPathTemplateHeatSources = _ST7API.St7GetNumLoadPathTemplateHeatSources
St7GetNumLoadPathTemplateHeatSources.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetLoadPathTemplateHeatSourceData = _ST7API.St7SetLoadPathTemplateHeatSourceData
St7SetLoadPathTemplateHeatSourceData.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetLoadPathTemplateHeatSourceData = _ST7API.St7GetLoadPathTemplateHeatSourceData
St7GetLoadPathTemplateHeatSourceData.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetLoadPathTemplateVehicleSet = _ST7API.St7SetLoadPathTemplateVehicleSet
St7SetLoadPathTemplateVehicleSet.argtypes = [c_long, c_long, c_long, c_char_p]
St7GetLoadPathTemplateVehicleSet = _ST7API.St7GetLoadPathTemplateVehicleSet
St7GetLoadPathTemplateVehicleSet.argtypes = [c_long, c_long, c_long, c_char_p, c_long]
St7DeleteLoadPathTemplate = _ST7API.St7DeleteLoadPathTemplate
St7DeleteLoadPathTemplate.argtypes = [c_long, c_long]
St7SetLoadPathTemplateCentrifugalData = _ST7API.St7SetLoadPathTemplateCentrifugalData
St7SetLoadPathTemplateCentrifugalData.argtypes = [c_long, c_long, c_char_p, c_char_p, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetLoadPathTemplateCentrifugalData = _ST7API.St7GetLoadPathTemplateCentrifugalData
St7GetLoadPathTemplateCentrifugalData.argtypes = [c_long, c_long, c_char_p, c_char_p, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetNumLibraries = _ST7API.St7GetNumLibraries
St7GetNumLibraries.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetLibraryName = _ST7API.St7GetLibraryName
St7GetLibraryName.argtypes = [c_long, c_long, c_char_p, c_long]
St7GetLibraryID = _ST7API.St7GetLibraryID
St7GetLibraryID.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long)]
St7GetNumLibraryItems = _ST7API.St7GetNumLibraryItems
St7GetNumLibraryItems.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetLibraryItemName = _ST7API.St7GetLibraryItemName
St7GetLibraryItemName.argtypes = [c_long, c_long, c_long, c_char_p, c_long]
St7GetLibraryItemID = _ST7API.St7GetLibraryItemID
St7GetLibraryItemID.argtypes = [c_long, c_long, c_char_p, ctypes.POINTER(c_long)]
St7GetLibraryBeamSectionPropertyDataBSL = _ST7API.St7GetLibraryBeamSectionPropertyDataBSL
St7GetLibraryBeamSectionPropertyDataBSL.argtypes = [c_long, c_long, c_long, c_char_p, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetLibraryBeamSectionPropertyDataBGL = _ST7API.St7GetLibraryBeamSectionPropertyDataBGL
St7GetLibraryBeamSectionPropertyDataBGL.argtypes = [c_long, c_long, c_long, c_char_p, c_long, ctypes.POINTER(c_double)]
St7GetLibraryBeamSectionGeometryBGL = _ST7API.St7GetLibraryBeamSectionGeometryBGL
St7GetLibraryBeamSectionGeometryBGL.argtypes = [c_long, c_long, c_long, c_char_p, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7AssignLibraryMaterial = _ST7API.St7AssignLibraryMaterial
St7AssignLibraryMaterial.argtypes = [c_long, c_long, c_long, c_long, c_long]
St7AssignLibraryComposite = _ST7API.St7AssignLibraryComposite
St7AssignLibraryComposite.argtypes = [c_long, c_long, c_long, c_long]
St7AssignLibraryBeamSection = _ST7API.St7AssignLibraryBeamSection
St7AssignLibraryBeamSection.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7AssignLibraryBeamSectionBGL = _ST7API.St7AssignLibraryBeamSectionBGL
St7AssignLibraryBeamSectionBGL.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7AssignLibraryCreepDefinition = _ST7API.St7AssignLibraryCreepDefinition
St7AssignLibraryCreepDefinition.argtypes = [c_long, c_long, c_long, c_long]
St7AssignLibraryLoadPathTemplate = _ST7API.St7AssignLibraryLoadPathTemplate
St7AssignLibraryLoadPathTemplate.argtypes = [c_long, c_long, c_long, c_long]
St7AssignLibraryReinforcementLayout = _ST7API.St7AssignLibraryReinforcementLayout
St7AssignLibraryReinforcementLayout.argtypes = [c_long, c_long, c_long, c_long]
St7NewTableType = _ST7API.St7NewTableType
St7NewTableType.argtypes = [c_long, c_long, c_long, c_long, c_char_p, ctypes.POINTER(c_double)]
St7DeleteTableType = _ST7API.St7DeleteTableType
St7DeleteTableType.argtypes = [c_long, c_long, c_long]
St7GetTableTypeName = _ST7API.St7GetTableTypeName
St7GetTableTypeName.argtypes = [c_long, c_long, c_long, c_char_p, c_long]
St7GetTableID = _ST7API.St7GetTableID
St7GetTableID.argtypes = [c_long, c_char_p, c_long, ctypes.POINTER(c_long)]
St7GetNumTableTypeRows = _ST7API.St7GetNumTableTypeRows
St7GetNumTableTypeRows.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetTableTypeData = _ST7API.St7SetTableTypeData
St7SetTableTypeData.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetTableTypeData = _ST7API.St7GetTableTypeData
St7GetTableTypeData.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetFrequencyTable = _ST7API.St7SetFrequencyTable
St7SetFrequencyTable.argtypes = [c_long, c_long, c_long]
St7GetFrequencyTable = _ST7API.St7GetFrequencyTable
St7GetFrequencyTable.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetTimeTableUnits = _ST7API.St7SetTimeTableUnits
St7SetTimeTableUnits.argtypes = [c_long, c_long, c_long, c_long]
St7GetTimeTableUnits = _ST7API.St7GetTimeTableUnits
St7GetTimeTableUnits.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7ConvertTimeTableUnits = _ST7API.St7ConvertTimeTableUnits
St7ConvertTimeTableUnits.argtypes = [c_long, c_long, c_long, c_long]
St7SetFrequencyPeriodTableUnits = _ST7API.St7SetFrequencyPeriodTableUnits
St7SetFrequencyPeriodTableUnits.argtypes = [c_long, c_long, c_long]
St7GetFrequencyPeriodTableUnits = _ST7API.St7GetFrequencyPeriodTableUnits
St7GetFrequencyPeriodTableUnits.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetAccVsTimeTableUnits = _ST7API.St7SetAccVsTimeTableUnits
St7SetAccVsTimeTableUnits.argtypes = [c_long, c_long, c_long]
St7GetAccVsTimeTableUnits = _ST7API.St7GetAccVsTimeTableUnits
St7GetAccVsTimeTableUnits.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetNumTables = _ST7API.St7GetNumTables
St7GetNumTables.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetTableInfoByIndex = _ST7API.St7GetTableInfoByIndex
St7GetTableInfoByIndex.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), c_char_p, c_long]
St7EnableLSALoadCase = _ST7API.St7EnableLSALoadCase
St7EnableLSALoadCase.argtypes = [c_long, c_long, c_long]
St7DisableLSALoadCase = _ST7API.St7DisableLSALoadCase
St7DisableLSALoadCase.argtypes = [c_long, c_long, c_long]
St7GetLSALoadCaseState = _ST7API.St7GetLSALoadCaseState
St7GetLSALoadCaseState.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7EnableLSAInitialPCGFile = _ST7API.St7EnableLSAInitialPCGFile
St7EnableLSAInitialPCGFile.argtypes = [c_long]
St7DisableLSAInitialPCGFile = _ST7API.St7DisableLSAInitialPCGFile
St7DisableLSAInitialPCGFile.argtypes = [c_long]
St7GetLSAInitialPCGFileState = _ST7API.St7GetLSAInitialPCGFileState
St7GetLSAInitialPCGFileState.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetLSAInitialPCGFile = _ST7API.St7SetLSAInitialPCGFile
St7SetLSAInitialPCGFile.argtypes = [c_long, c_char_p]
St7GetLSAInitialPCGFile = _ST7API.St7GetLSAInitialPCGFile
St7GetLSAInitialPCGFile.argtypes = [c_long, c_char_p, c_long]
St7SetLBAInitial = _ST7API.St7SetLBAInitial
St7SetLBAInitial.argtypes = [c_long, c_char_p, c_long, c_long]
St7GetLBAInitial = _ST7API.St7GetLBAInitial
St7GetLBAInitial.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), ctypes.POINTER(c_long), c_long]
St7SetLBANumModes = _ST7API.St7SetLBANumModes
St7SetLBANumModes.argtypes = [c_long, c_long]
St7GetLBANumModes = _ST7API.St7GetLBANumModes
St7GetLBANumModes.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetLBAShift = _ST7API.St7SetLBAShift
St7SetLBAShift.argtypes = [c_long, c_double]
St7GetLBAShift = _ST7API.St7GetLBAShift
St7GetLBAShift.argtypes = [c_long, ctypes.POINTER(c_double)]
St7EnableLIALoadCase = _ST7API.St7EnableLIALoadCase
St7EnableLIALoadCase.argtypes = [c_long, c_long, c_long]
St7DisableLIALoadCase = _ST7API.St7DisableLIALoadCase
St7DisableLIALoadCase.argtypes = [c_long, c_long, c_long]
St7GetLIALoadCaseState = _ST7API.St7GetLIALoadCaseState
St7GetLIALoadCaseState.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7SetNLAStagedAnalysis = _ST7API.St7SetNLAStagedAnalysis
St7SetNLAStagedAnalysis.argtypes = [c_long, c_bool]
St7GetNLAStagedAnalysis = _ST7API.St7GetNLAStagedAnalysis
St7GetNLAStagedAnalysis.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7EnableNLAStage = _ST7API.St7EnableNLAStage
St7EnableNLAStage.argtypes = [c_long, c_long]
St7DisableNLAStage = _ST7API.St7DisableNLAStage
St7DisableNLAStage.argtypes = [c_long, c_long]
St7GetNLAStageState = _ST7API.St7GetNLAStageState
St7GetNLAStageState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7AddNLAIncrement = _ST7API.St7AddNLAIncrement
St7AddNLAIncrement.argtypes = [c_long, c_long, c_char_p]
St7GetNLAIncrementName = _ST7API.St7GetNLAIncrementName
St7GetNLAIncrementName.argtypes = [c_long, c_long, c_long, c_char_p, c_long]
St7InsertNLAIncrement = _ST7API.St7InsertNLAIncrement
St7InsertNLAIncrement.argtypes = [c_long, c_long, c_long, c_char_p]
St7DeleteNLAIncrement = _ST7API.St7DeleteNLAIncrement
St7DeleteNLAIncrement.argtypes = [c_long, c_long, c_long]
St7GetNumNLAIncrements = _ST7API.St7GetNumNLAIncrements
St7GetNumNLAIncrements.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetNLALoadIncrementFactor = _ST7API.St7SetNLALoadIncrementFactor
St7SetNLALoadIncrementFactor.argtypes = [c_long, c_long, c_long, c_long, c_double]
St7GetNLALoadIncrementFactor = _ST7API.St7GetNLALoadIncrementFactor
St7GetNLALoadIncrementFactor.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetNLAFreedomIncrementFactor = _ST7API.St7SetNLAFreedomIncrementFactor
St7SetNLAFreedomIncrementFactor.argtypes = [c_long, c_long, c_long, c_long, c_double]
St7GetNLAFreedomIncrementFactor = _ST7API.St7GetNLAFreedomIncrementFactor
St7GetNLAFreedomIncrementFactor.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7EnableNLALoadCase = _ST7API.St7EnableNLALoadCase
St7EnableNLALoadCase.argtypes = [c_long, c_long, c_long]
St7DisableNLALoadCase = _ST7API.St7DisableNLALoadCase
St7DisableNLALoadCase.argtypes = [c_long, c_long, c_long]
St7EnableNLAFreedomCase = _ST7API.St7EnableNLAFreedomCase
St7EnableNLAFreedomCase.argtypes = [c_long, c_long, c_long]
St7DisableNLAFreedomCase = _ST7API.St7DisableNLAFreedomCase
St7DisableNLAFreedomCase.argtypes = [c_long, c_long, c_long]
St7GetNLALoadCaseState = _ST7API.St7GetNLALoadCaseState
St7GetNLALoadCaseState.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7GetNLAFreedomCaseState = _ST7API.St7GetNLAFreedomCaseState
St7GetNLAFreedomCaseState.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7SetNLAInitial = _ST7API.St7SetNLAInitial
St7SetNLAInitial.argtypes = [c_long, c_char_p, c_long]
St7GetNLAInitial = _ST7API.St7GetNLAInitial
St7GetNLAInitial.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7SetNLAPseudoTime = _ST7API.St7SetNLAPseudoTime
St7SetNLAPseudoTime.argtypes = [c_long, c_long, c_long, c_double]
St7GetNLAPseudoTime = _ST7API.St7GetNLAPseudoTime
St7GetNLAPseudoTime.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7EnableNLAPseudoTime = _ST7API.St7EnableNLAPseudoTime
St7EnableNLAPseudoTime.argtypes = [c_long, c_long]
St7DisableNLAPseudoTime = _ST7API.St7DisableNLAPseudoTime
St7DisableNLAPseudoTime.argtypes = [c_long, c_long]
St7GetNLAPseudoTimeState = _ST7API.St7GetNLAPseudoTimeState
St7GetNLAPseudoTimeState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetNLAResetAtIncrement = _ST7API.St7SetNLAResetAtIncrement
St7SetNLAResetAtIncrement.argtypes = [c_long, c_long, c_bool]
St7GetNLAResetAtIncrement = _ST7API.St7GetNLAResetAtIncrement
St7GetNLAResetAtIncrement.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetQSAInitial = _ST7API.St7SetQSAInitial
St7SetQSAInitial.argtypes = [c_long, c_char_p, c_long]
St7GetQSAInitial = _ST7API.St7GetQSAInitial
St7GetQSAInitial.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7SetNFAInitial = _ST7API.St7SetNFAInitial
St7SetNFAInitial.argtypes = [c_long, c_char_p, c_long]
St7GetNFAInitial = _ST7API.St7GetNFAInitial
St7GetNFAInitial.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7SetNFANumModes = _ST7API.St7SetNFANumModes
St7SetNFANumModes.argtypes = [c_long, c_long]
St7GetNFANumModes = _ST7API.St7GetNFANumModes
St7GetNFANumModes.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetNFAShift = _ST7API.St7SetNFAShift
St7SetNFAShift.argtypes = [c_long, c_double]
St7GetNFAShift = _ST7API.St7GetNFAShift
St7GetNFAShift.argtypes = [c_long, ctypes.POINTER(c_double)]
St7SetNFAModeParticipationCalculate = _ST7API.St7SetNFAModeParticipationCalculate
St7SetNFAModeParticipationCalculate.argtypes = [c_long, c_bool]
St7GetNFAModeParticipationCalculate = _ST7API.St7GetNFAModeParticipationCalculate
St7GetNFAModeParticipationCalculate.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetNFAModeParticipationVectors = _ST7API.St7SetNFAModeParticipationVectors
St7SetNFAModeParticipationVectors.argtypes = [c_long, ctypes.POINTER(c_double)]
St7GetNFAModeParticipationVectors = _ST7API.St7GetNFAModeParticipationVectors
St7GetNFAModeParticipationVectors.argtypes = [c_long, ctypes.POINTER(c_double)]
St7SetHRARange = _ST7API.St7SetHRARange
St7SetHRARange.argtypes = [c_long, c_long, c_double, c_double, c_bool]
St7GetHRARange = _ST7API.St7GetHRARange
St7GetHRARange.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_bool)]
St7SetHRABaseVector = _ST7API.St7SetHRABaseVector
St7SetHRABaseVector.argtypes = [c_long, ctypes.POINTER(c_double)]
St7GetHRABaseVector = _ST7API.St7GetHRABaseVector
St7GetHRABaseVector.argtypes = [c_long, ctypes.POINTER(c_double)]
St7SetHRALoadCase = _ST7API.St7SetHRALoadCase
St7SetHRALoadCase.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetHRALoadCase = _ST7API.St7GetHRALoadCase
St7GetHRALoadCase.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetHRALoadType = _ST7API.St7SetHRALoadType
St7SetHRALoadType.argtypes = [c_long, c_long]
St7GetHRALoadType = _ST7API.St7GetHRALoadType
St7GetHRALoadType.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetHRAMode = _ST7API.St7SetHRAMode
St7SetHRAMode.argtypes = [c_long, c_long]
St7GetHRAMode = _ST7API.St7GetHRAMode
St7GetHRAMode.argtypes = [c_long, ctypes.POINTER(c_long)]
St7AddSRALoadCase = _ST7API.St7AddSRALoadCase
St7AddSRALoadCase.argtypes = [c_long, c_char_p]
St7InsertSRALoadCase = _ST7API.St7InsertSRALoadCase
St7InsertSRALoadCase.argtypes = [c_long, c_long, c_char_p]
St7DeleteSRALoadCase = _ST7API.St7DeleteSRALoadCase
St7DeleteSRALoadCase.argtypes = [c_long, c_long]
St7GetNumSRALoadCases = _ST7API.St7GetNumSRALoadCases
St7GetNumSRALoadCases.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetSRALoadCaseTable = _ST7API.St7SetSRALoadCaseTable
St7SetSRALoadCaseTable.argtypes = [c_long, c_long, c_long, c_long]
St7GetSRALoadCaseTable = _ST7API.St7GetSRALoadCaseTable
St7GetSRALoadCaseTable.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7AddSRADirectionVector = _ST7API.St7AddSRADirectionVector
St7AddSRADirectionVector.argtypes = [c_long, c_char_p]
St7InsertSRADirectionVector = _ST7API.St7InsertSRADirectionVector
St7InsertSRADirectionVector.argtypes = [c_long, c_long, c_char_p]
St7DeleteSRADirectionVector = _ST7API.St7DeleteSRADirectionVector
St7DeleteSRADirectionVector.argtypes = [c_long, c_long]
St7GetNumSRADirectionVectors = _ST7API.St7GetNumSRADirectionVectors
St7GetNumSRADirectionVectors.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetSRADirectionVectorTable = _ST7API.St7SetSRADirectionVectorTable
St7SetSRADirectionVectorTable.argtypes = [c_long, c_long, c_long]
St7GetSRADirectionVectorTable = _ST7API.St7GetSRADirectionVectorTable
St7GetSRADirectionVectorTable.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetSRADirectionVectorType = _ST7API.St7SetSRADirectionVectorType
St7SetSRADirectionVectorType.argtypes = [c_long, c_long, c_long]
St7GetSRADirectionVectorType = _ST7API.St7GetSRADirectionVectorType
St7GetSRADirectionVectorType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetSRADirectionVectorFactors = _ST7API.St7SetSRADirectionVectorFactors
St7SetSRADirectionVectorFactors.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetSRADirectionVectorFactors = _ST7API.St7GetSRADirectionVectorFactors
St7GetSRADirectionVectorFactors.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetSRAResultModal = _ST7API.St7SetSRAResultModal
St7SetSRAResultModal.argtypes = [c_long, c_bool]
St7GetSRAResultModal = _ST7API.St7GetSRAResultModal
St7GetSRAResultModal.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetSRAResultSRSS = _ST7API.St7SetSRAResultSRSS
St7SetSRAResultSRSS.argtypes = [c_long, c_bool]
St7GetSRAResultSRSS = _ST7API.St7GetSRAResultSRSS
St7GetSRAResultSRSS.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetSRAResultCQC = _ST7API.St7SetSRAResultCQC
St7SetSRAResultCQC.argtypes = [c_long, c_bool]
St7GetSRAResultCQC = _ST7API.St7GetSRAResultCQC
St7GetSRAResultCQC.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetSRAType = _ST7API.St7SetSRAType
St7SetSRAType.argtypes = [c_long, c_long]
St7GetSRAType = _ST7API.St7GetSRAType
St7GetSRAType.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetSRAResultsSign = _ST7API.St7SetSRAResultsSign
St7SetSRAResultsSign.argtypes = [c_long, c_long]
St7GetSRAResultsSign = _ST7API.St7GetSRAResultsSign
St7GetSRAResultsSign.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetSRABaseExcitation = _ST7API.St7SetSRABaseExcitation
St7SetSRABaseExcitation.argtypes = [c_long, c_bool]
St7GetSRABaseExcitation = _ST7API.St7GetSRABaseExcitation
St7GetSRABaseExcitation.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetSRALoadExcitation = _ST7API.St7SetSRALoadExcitation
St7SetSRALoadExcitation.argtypes = [c_long, c_bool]
St7GetSRALoadExcitation = _ST7API.St7GetSRALoadExcitation
St7GetSRALoadExcitation.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7EnableSRACase = _ST7API.St7EnableSRACase
St7EnableSRACase.argtypes = [c_long, c_long]
St7DisableSRACase = _ST7API.St7DisableSRACase
St7DisableSRACase.argtypes = [c_long, c_long]
St7EnableSRADirectionVector = _ST7API.St7EnableSRADirectionVector
St7EnableSRADirectionVector.argtypes = [c_long, c_long]
St7DisableSRADirectionVector = _ST7API.St7DisableSRADirectionVector
St7DisableSRADirectionVector.argtypes = [c_long, c_long]
St7GetSRACaseStatus = _ST7API.St7GetSRACaseStatus
St7GetSRACaseStatus.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7GetSRADirectionVectorStatus = _ST7API.St7GetSRADirectionVectorStatus
St7GetSRADirectionVectorStatus.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetLTAInitial = _ST7API.St7SetLTAInitial
St7SetLTAInitial.argtypes = [c_long, c_char_p, c_long]
St7GetLTAInitial = _ST7API.St7GetLTAInitial
St7GetLTAInitial.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7SetLTAMethod = _ST7API.St7SetLTAMethod
St7SetLTAMethod.argtypes = [c_long, c_long]
St7GetLTAMethod = _ST7API.St7GetLTAMethod
St7GetLTAMethod.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetLTASolutionType = _ST7API.St7SetLTASolutionType
St7SetLTASolutionType.argtypes = [c_long, c_long]
St7GetLTASolutionType = _ST7API.St7GetLTASolutionType
St7GetLTASolutionType.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetNTAInitial = _ST7API.St7SetNTAInitial
St7SetNTAInitial.argtypes = [c_long, c_char_p, c_long]
St7GetNTAInitial = _ST7API.St7GetNTAInitial
St7GetNTAInitial.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7EnableHeatLoadCase = _ST7API.St7EnableHeatLoadCase
St7EnableHeatLoadCase.argtypes = [c_long, c_long]
St7DisableHeatLoadCase = _ST7API.St7DisableHeatLoadCase
St7DisableHeatLoadCase.argtypes = [c_long, c_long]
St7GetHeatLoadCaseState = _ST7API.St7GetHeatLoadCaseState
St7GetHeatLoadCaseState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetTHAInitial = _ST7API.St7SetTHAInitial
St7SetTHAInitial.argtypes = [c_long, c_char_p, c_long]
St7GetTHAInitial = _ST7API.St7GetTHAInitial
St7GetTHAInitial.argtypes = [c_long, c_char_p, ctypes.POINTER(c_long), c_long]
St7SetTHATemperatureLoadCase = _ST7API.St7SetTHATemperatureLoadCase
St7SetTHATemperatureLoadCase.argtypes = [c_long, c_long]
St7GetTHATemperatureLoadCase = _ST7API.St7GetTHATemperatureLoadCase
St7GetTHATemperatureLoadCase.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetTHAInitialAttributeOverride = _ST7API.St7SetTHAInitialAttributeOverride
St7SetTHAInitialAttributeOverride.argtypes = [c_long, c_bool]
St7GetTHAInitialAttributeOverride = _ST7API.St7GetTHAInitialAttributeOverride
St7GetTHAInitialAttributeOverride.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetModalSuperpositionFile = _ST7API.St7SetModalSuperpositionFile
St7SetModalSuperpositionFile.argtypes = [c_long, c_char_p]
St7GetModalSuperpositionFile = _ST7API.St7GetModalSuperpositionFile
St7GetModalSuperpositionFile.argtypes = [c_long, c_char_p, c_long]
St7GetNumModesInModalFile = _ST7API.St7GetNumModesInModalFile
St7GetNumModesInModalFile.argtypes = [c_long, ctypes.POINTER(c_long)]
St7EnableMode = _ST7API.St7EnableMode
St7EnableMode.argtypes = [c_long, c_long]
St7DisableMode = _ST7API.St7DisableMode
St7DisableMode.argtypes = [c_long, c_long]
St7SetModeDampingRatio = _ST7API.St7SetModeDampingRatio
St7SetModeDampingRatio.argtypes = [c_long, c_long, c_double]
St7GetModeDampingRatio = _ST7API.St7GetModeDampingRatio
St7GetModeDampingRatio.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetTransientInitialConditionsType = _ST7API.St7SetTransientInitialConditionsType
St7SetTransientInitialConditionsType.argtypes = [c_long, c_long]
St7GetTransientInitialConditionsType = _ST7API.St7GetTransientInitialConditionsType
St7GetTransientInitialConditionsType.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetTransientInitialConditionsVectors = _ST7API.St7SetTransientInitialConditionsVectors
St7SetTransientInitialConditionsVectors.argtypes = [c_long, ctypes.POINTER(c_double)]
St7GetTransientInitialConditionsVectors = _ST7API.St7GetTransientInitialConditionsVectors
St7GetTransientInitialConditionsVectors.argtypes = [c_long, ctypes.POINTER(c_double)]
St7SetTransientInitialConditionsNodalVelocity = _ST7API.St7SetTransientInitialConditionsNodalVelocity
St7SetTransientInitialConditionsNodalVelocity.argtypes = [c_long, c_long]
St7GetTransientInitialConditionsNodalVelocity = _ST7API.St7GetTransientInitialConditionsNodalVelocity
St7GetTransientInitialConditionsNodalVelocity.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetTransientBaseExcitation = _ST7API.St7SetTransientBaseExcitation
St7SetTransientBaseExcitation.argtypes = [c_long, c_long]
St7GetTransientBaseExcitation = _ST7API.St7GetTransientBaseExcitation
St7GetTransientBaseExcitation.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetTransientBaseVector = _ST7API.St7SetTransientBaseVector
St7SetTransientBaseVector.argtypes = [c_long, ctypes.POINTER(c_double)]
St7GetTransientBaseVector = _ST7API.St7GetTransientBaseVector
St7GetTransientBaseVector.argtypes = [c_long, ctypes.POINTER(c_double)]
St7SetTransientBaseAcceleration = _ST7API.St7SetTransientBaseAcceleration
St7SetTransientBaseAcceleration.argtypes = [c_long, ctypes.POINTER(c_double)]
St7GetTransientBaseAcceleration = _ST7API.St7GetTransientBaseAcceleration
St7GetTransientBaseAcceleration.argtypes = [c_long, ctypes.POINTER(c_double)]
St7SetTransientBaseVelocity = _ST7API.St7SetTransientBaseVelocity
St7SetTransientBaseVelocity.argtypes = [c_long, ctypes.POINTER(c_double)]
St7GetTransientBaseVelocity = _ST7API.St7GetTransientBaseVelocity
St7GetTransientBaseVelocity.argtypes = [c_long, ctypes.POINTER(c_double)]
St7SetTransientBaseDisplacement = _ST7API.St7SetTransientBaseDisplacement
St7SetTransientBaseDisplacement.argtypes = [c_long, ctypes.POINTER(c_double)]
St7GetTransientBaseDisplacement = _ST7API.St7GetTransientBaseDisplacement
St7GetTransientBaseDisplacement.argtypes = [c_long, ctypes.POINTER(c_double)]
St7SetTransientBaseTables = _ST7API.St7SetTransientBaseTables
St7SetTransientBaseTables.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetTransientBaseTables = _ST7API.St7GetTransientBaseTables
St7GetTransientBaseTables.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7AddTransientNodeHistoryCase = _ST7API.St7AddTransientNodeHistoryCase
St7AddTransientNodeHistoryCase.argtypes = [c_long, c_long]
St7InsertTransientNodeHistoryCase = _ST7API.St7InsertTransientNodeHistoryCase
St7InsertTransientNodeHistoryCase.argtypes = [c_long, c_long, c_long]
St7DeleteTransientNodeHistoryCase = _ST7API.St7DeleteTransientNodeHistoryCase
St7DeleteTransientNodeHistoryCase.argtypes = [c_long, c_long]
St7GetNumTransientNodeHistoryCases = _ST7API.St7GetNumTransientNodeHistoryCases
St7GetNumTransientNodeHistoryCases.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetTransientNodeHistoryCaseData = _ST7API.St7SetTransientNodeHistoryCaseData
St7SetTransientNodeHistoryCaseData.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7GetTransientNodeHistoryCaseData = _ST7API.St7GetTransientNodeHistoryCaseData
St7GetTransientNodeHistoryCaseData.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetTransientTemperatureInputType = _ST7API.St7SetTransientTemperatureInputType
St7SetTransientTemperatureInputType.argtypes = [c_long, c_long]
St7SetTransientHeatFile = _ST7API.St7SetTransientHeatFile
St7SetTransientHeatFile.argtypes = [c_long, c_char_p, c_double]
St7GetTransientHeatFile = _ST7API.St7GetTransientHeatFile
St7GetTransientHeatFile.argtypes = [c_long, c_char_p, c_long, ctypes.POINTER(c_double)]
St7SetTransientLoadPositionTable = _ST7API.St7SetTransientLoadPositionTable
St7SetTransientLoadPositionTable.argtypes = [c_long, c_long, c_long, c_long, c_long]
St7GetTransientLoadPositionTable = _ST7API.St7GetTransientLoadPositionTable
St7GetTransientLoadPositionTable.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetTransientFreedomPositionTable = _ST7API.St7SetTransientFreedomPositionTable
St7SetTransientFreedomPositionTable.argtypes = [c_long, c_long, c_long, c_long, c_long]
St7GetTransientFreedomPositionTable = _ST7API.St7GetTransientFreedomPositionTable
St7GetTransientFreedomPositionTable.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7EnableTransientLoadCase = _ST7API.St7EnableTransientLoadCase
St7EnableTransientLoadCase.argtypes = [c_long, c_long]
St7DisableTransientLoadCase = _ST7API.St7DisableTransientLoadCase
St7DisableTransientLoadCase.argtypes = [c_long, c_long]
St7EnableTransientFreedomCase = _ST7API.St7EnableTransientFreedomCase
St7EnableTransientFreedomCase.argtypes = [c_long, c_long]
St7DisableTransientFreedomCase = _ST7API.St7DisableTransientFreedomCase
St7DisableTransientFreedomCase.argtypes = [c_long, c_long]
St7GetTransientLoadCaseState = _ST7API.St7GetTransientLoadCaseState
St7GetTransientLoadCaseState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7GetTransientFreedomCaseState = _ST7API.St7GetTransientFreedomCaseState
St7GetTransientFreedomCaseState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetTransientLoadTimeTable = _ST7API.St7SetTransientLoadTimeTable
St7SetTransientLoadTimeTable.argtypes = [c_long, c_long, c_long, c_bool]
St7GetTransientLoadTimeTable = _ST7API.St7GetTransientLoadTimeTable
St7GetTransientLoadTimeTable.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_bool)]
St7SetTransientFreedomTimeTable = _ST7API.St7SetTransientFreedomTimeTable
St7SetTransientFreedomTimeTable.argtypes = [c_long, c_long, c_long, c_bool]
St7GetTransientFreedomTimeTable = _ST7API.St7GetTransientFreedomTimeTable
St7GetTransientFreedomTimeTable.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_bool)]
St7SetNumTimeStepRows = _ST7API.St7SetNumTimeStepRows
St7SetNumTimeStepRows.argtypes = [c_long, c_long]
St7GetNumTimeStepRows = _ST7API.St7GetNumTimeStepRows
St7GetNumTimeStepRows.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetTimeStepData = _ST7API.St7SetTimeStepData
St7SetTimeStepData.argtypes = [c_long, c_long, c_long, c_long, c_double]
St7GetTimeStepData = _ST7API.St7GetTimeStepData
St7GetTimeStepData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetTimeStepUnit = _ST7API.St7SetTimeStepUnit
St7SetTimeStepUnit.argtypes = [c_long, c_long]
St7GetTimeStepUnit = _ST7API.St7GetTimeStepUnit
St7GetTimeStepUnit.argtypes = [c_long, ctypes.POINTER(c_long)]
St7EnableMovingLoad = _ST7API.St7EnableMovingLoad
St7EnableMovingLoad.argtypes = [c_long, c_long]
St7DisableMovingLoad = _ST7API.St7DisableMovingLoad
St7DisableMovingLoad.argtypes = [c_long, c_long]
St7GetMovingLoadState = _ST7API.St7GetMovingLoadState
St7GetMovingLoadState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetMovingLoadTimeTable = _ST7API.St7SetMovingLoadTimeTable
St7SetMovingLoadTimeTable.argtypes = [c_long, c_long, c_long]
St7GetMovingLoadTimeTable = _ST7API.St7GetMovingLoadTimeTable
St7GetMovingLoadTimeTable.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetMovingLoadAutoDivisions = _ST7API.St7SetMovingLoadAutoDivisions
St7SetMovingLoadAutoDivisions.argtypes = [c_long, c_long, c_bool]
St7GetMovingLoadAutoDivisions = _ST7API.St7GetMovingLoadAutoDivisions
St7GetMovingLoadAutoDivisions.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetSolverHeatNonlinear = _ST7API.St7SetSolverHeatNonlinear
St7SetSolverHeatNonlinear.argtypes = [c_long, c_bool]
St7SetSolverFontName = _ST7API.St7SetSolverFontName
St7SetSolverFontName.argtypes = [c_char_p]
St7GetSolverFontName = _ST7API.St7GetSolverFontName
St7GetSolverFontName.argtypes = [c_char_p, c_long]
St7SetSolverNumCPU = _ST7API.St7SetSolverNumCPU
St7SetSolverNumCPU.argtypes = [c_long]
St7GetSolverNumCPU = _ST7API.St7GetSolverNumCPU
St7GetSolverNumCPU.argtypes = [ctypes.POINTER(c_long)]
St7SetSolverScheme = _ST7API.St7SetSolverScheme
St7SetSolverScheme.argtypes = [c_long, c_long]
St7GetSolverScheme = _ST7API.St7GetSolverScheme
St7GetSolverScheme.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetSolverSort = _ST7API.St7SetSolverSort
St7SetSolverSort.argtypes = [c_long, c_long]
St7GetSolverSort = _ST7API.St7GetSolverSort
St7GetSolverSort.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetSolverTreeStartNumber = _ST7API.St7SetSolverTreeStartNumber
St7SetSolverTreeStartNumber.argtypes = [c_long, c_long]
St7GetSolverTreeStartNumber = _ST7API.St7GetSolverTreeStartNumber
St7GetSolverTreeStartNumber.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetSolverActiveStage = _ST7API.St7SetSolverActiveStage
St7SetSolverActiveStage.argtypes = [c_long, c_long]
St7GetSolverActiveStage = _ST7API.St7GetSolverActiveStage
St7GetSolverActiveStage.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetSolverTemperatureDependence = _ST7API.St7SetSolverTemperatureDependence
St7SetSolverTemperatureDependence.argtypes = [c_long, c_long]
St7GetSolverTemperatureDependence = _ST7API.St7GetSolverTemperatureDependence
St7GetSolverTemperatureDependence.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetSolverLoadCaseTemperatureDependence = _ST7API.St7SetSolverLoadCaseTemperatureDependence
St7SetSolverLoadCaseTemperatureDependence.argtypes = [c_long, c_long]
St7GetSolverLoadCaseTemperatureDependence = _ST7API.St7GetSolverLoadCaseTemperatureDependence
St7GetSolverLoadCaseTemperatureDependence.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetSolverFreedomCase = _ST7API.St7SetSolverFreedomCase
St7SetSolverFreedomCase.argtypes = [c_long, c_long]
St7GetSolverFreedomCase = _ST7API.St7GetSolverFreedomCase
St7GetSolverFreedomCase.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetDampingType = _ST7API.St7SetDampingType
St7SetDampingType.argtypes = [c_long, c_long]
St7GetDampingType = _ST7API.St7GetDampingType
St7GetDampingType.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetRayleighFactors = _ST7API.St7SetRayleighFactors
St7SetRayleighFactors.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetRayleighFactors = _ST7API.St7GetRayleighFactors
St7GetRayleighFactors.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetSoilFluidOptions = _ST7API.St7SetSoilFluidOptions
St7SetSoilFluidOptions.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetSoilFluidOptions = _ST7API.St7GetSoilFluidOptions
St7GetSoilFluidOptions.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetSoilAutoDrained = _ST7API.St7SetSoilAutoDrained
St7SetSoilAutoDrained.argtypes = [c_long, c_bool]
St7GetSoilAutoDrained = _ST7API.St7GetSoilAutoDrained
St7GetSoilAutoDrained.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetSturmCheck = _ST7API.St7SetSturmCheck
St7SetSturmCheck.argtypes = [c_long, c_bool]
St7GetSturmCheck = _ST7API.St7GetSturmCheck
St7GetSturmCheck.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetSolverNonlinearGeometry = _ST7API.St7SetSolverNonlinearGeometry
St7SetSolverNonlinearGeometry.argtypes = [c_long, c_bool]
St7GetSolverNonlinearGeometry = _ST7API.St7GetSolverNonlinearGeometry
St7GetSolverNonlinearGeometry.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetSolverNonlinearMaterial = _ST7API.St7SetSolverNonlinearMaterial
St7SetSolverNonlinearMaterial.argtypes = [c_long, c_bool]
St7GetSolverNonlinearMaterial = _ST7API.St7GetSolverNonlinearMaterial
St7GetSolverNonlinearMaterial.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetSolverCreep = _ST7API.St7SetSolverCreep
St7SetSolverCreep.argtypes = [c_long, c_bool]
St7GetSolverCreep = _ST7API.St7GetSolverCreep
St7GetSolverCreep.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetSolverIncludeKG = _ST7API.St7SetSolverIncludeKG
St7SetSolverIncludeKG.argtypes = [c_long, c_bool]
St7GetSolverIncludeKG = _ST7API.St7GetSolverIncludeKG
St7GetSolverIncludeKG.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetSolverStressStiffening = _ST7API.St7SetSolverStressStiffening
St7SetSolverStressStiffening.argtypes = [c_long, c_bool]
St7GetSolverStressStiffening = _ST7API.St7GetSolverStressStiffening
St7GetSolverStressStiffening.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetEntityResult = _ST7API.St7SetEntityResult
St7SetEntityResult.argtypes = [c_long, c_long, c_long]
St7GetEntityResult = _ST7API.St7GetEntityResult
St7GetEntityResult.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7EnableResultGroup = _ST7API.St7EnableResultGroup
St7EnableResultGroup.argtypes = [c_long, c_long]
St7DisableResultGroup = _ST7API.St7DisableResultGroup
St7DisableResultGroup.argtypes = [c_long, c_long]
St7GetResultGroupState = _ST7API.St7GetResultGroupState
St7GetResultGroupState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7EnableResultProperty = _ST7API.St7EnableResultProperty
St7EnableResultProperty.argtypes = [c_long, c_long, c_long]
St7DisableResultProperty = _ST7API.St7DisableResultProperty
St7DisableResultProperty.argtypes = [c_long, c_long, c_long]
St7GetResultPropertyState = _ST7API.St7GetResultPropertyState
St7GetResultPropertyState.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7SetResultFileName = _ST7API.St7SetResultFileName
St7SetResultFileName.argtypes = [c_long, c_char_p]
St7SetResultLogFileName = _ST7API.St7SetResultLogFileName
St7SetResultLogFileName.argtypes = [c_long, c_char_p]
St7SetStaticRestartFile = _ST7API.St7SetStaticRestartFile
St7SetStaticRestartFile.argtypes = [c_long, c_char_p]
St7GetStaticRestartFile = _ST7API.St7GetStaticRestartFile
St7GetStaticRestartFile.argtypes = [c_long, c_char_p, c_long]
St7SetDynamicRestartFile = _ST7API.St7SetDynamicRestartFile
St7SetDynamicRestartFile.argtypes = [c_long, c_char_p]
St7GetDynamicRestartFile = _ST7API.St7GetDynamicRestartFile
St7GetDynamicRestartFile.argtypes = [c_long, c_char_p, c_long]
St7SetQuasiStaticRestartFile = _ST7API.St7SetQuasiStaticRestartFile
St7SetQuasiStaticRestartFile.argtypes = [c_long, c_char_p]
St7GetQuasiStaticRestartFile = _ST7API.St7GetQuasiStaticRestartFile
St7GetQuasiStaticRestartFile.argtypes = [c_long, c_char_p, c_long]
St7SetNodeHistoryFile = _ST7API.St7SetNodeHistoryFile
St7SetNodeHistoryFile.argtypes = [c_long, c_char_p]
St7GetNodeHistoryFile = _ST7API.St7GetNodeHistoryFile
St7GetNodeHistoryFile.argtypes = [c_long, c_char_p, c_long]
St7EnableSaveRestart = _ST7API.St7EnableSaveRestart
St7EnableSaveRestart.argtypes = [c_long]
St7DisableSaveRestart = _ST7API.St7DisableSaveRestart
St7DisableSaveRestart.argtypes = [c_long]
St7EnableSaveLastRestartStep = _ST7API.St7EnableSaveLastRestartStep
St7EnableSaveLastRestartStep.argtypes = [c_long]
St7DisableSaveLastRestartStep = _ST7API.St7DisableSaveLastRestartStep
St7DisableSaveLastRestartStep.argtypes = [c_long]
St7SetAppendSRA = _ST7API.St7SetAppendSRA
St7SetAppendSRA.argtypes = [c_long, c_bool]
St7GetAppendSRA = _ST7API.St7GetAppendSRA
St7GetAppendSRA.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7EnableNSMassCaseInMassMatrix = _ST7API.St7EnableNSMassCaseInMassMatrix
St7EnableNSMassCaseInMassMatrix.argtypes = [c_long, c_long]
St7DisableNSMassCaseInMassMatrix = _ST7API.St7DisableNSMassCaseInMassMatrix
St7DisableNSMassCaseInMassMatrix.argtypes = [c_long, c_long]
St7GetNSMassCaseInMassMatrixState = _ST7API.St7GetNSMassCaseInMassMatrixState
St7GetNSMassCaseInMassMatrixState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetSolverDefaultsLogical = _ST7API.St7SetSolverDefaultsLogical
St7SetSolverDefaultsLogical.argtypes = [c_long, c_long, c_bool]
St7GetSolverDefaultsLogical = _ST7API.St7GetSolverDefaultsLogical
St7GetSolverDefaultsLogical.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetSolverDefaultsInteger = _ST7API.St7SetSolverDefaultsInteger
St7SetSolverDefaultsInteger.argtypes = [c_long, c_long, c_long]
St7GetSolverDefaultsInteger = _ST7API.St7GetSolverDefaultsInteger
St7GetSolverDefaultsInteger.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetSolverDefaultsDouble = _ST7API.St7SetSolverDefaultsDouble
St7SetSolverDefaultsDouble.argtypes = [c_long, c_long, c_double]
St7GetSolverDefaultsDouble = _ST7API.St7GetSolverDefaultsDouble
St7GetSolverDefaultsDouble.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetUseSolverDLL = _ST7API.St7SetUseSolverDLL
St7SetUseSolverDLL.argtypes = [c_bool]
St7GetUseSolverDLL = _ST7API.St7GetUseSolverDLL
St7GetUseSolverDLL.argtypes = [ctypes.POINTER(c_bool)]
St7CheckSolverRunning = _ST7API.St7CheckSolverRunning
St7CheckSolverRunning.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7SetSolverWindowPos = _ST7API.St7SetSolverWindowPos
St7SetSolverWindowPos.argtypes = [c_long, c_long, c_long, c_long]
St7ClearSolverWindowPos = _ST7API.St7ClearSolverWindowPos
St7ClearSolverWindowPos.argtypes = []
St7RunSolver = _ST7API.St7RunSolver
St7RunSolver.argtypes = [c_long, c_long, c_long, c_long]
St7RunSolverProcess = _ST7API.St7RunSolverProcess
St7RunSolverProcess.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetResultOptions = _ST7API.St7SetResultOptions
St7SetResultOptions.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetResultOptions = _ST7API.St7GetResultOptions
St7GetResultOptions.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetLimitEnvelopeAveragingOrder = _ST7API.St7SetLimitEnvelopeAveragingOrder
St7SetLimitEnvelopeAveragingOrder.argtypes = [c_long, c_long]
St7GetLimitEnvelopeAveragingOrder = _ST7API.St7GetLimitEnvelopeAveragingOrder
St7GetLimitEnvelopeAveragingOrder.argtypes = [c_long, ctypes.POINTER(c_long)]
St7EnableModelStrainUnit = _ST7API.St7EnableModelStrainUnit
St7EnableModelStrainUnit.argtypes = [c_long]
St7DisableModelStrainUnit = _ST7API.St7DisableModelStrainUnit
St7DisableModelStrainUnit.argtypes = [c_long]
St7EnableModelRotationUnit = _ST7API.St7EnableModelRotationUnit
St7EnableModelRotationUnit.argtypes = [c_long]
St7DisableModelRotationUnit = _ST7API.St7DisableModelRotationUnit
St7DisableModelRotationUnit.argtypes = [c_long]
St7EnableModelRCUnit = _ST7API.St7EnableModelRCUnit
St7EnableModelRCUnit.argtypes = [c_long]
St7DisableModelRCUnit = _ST7API.St7DisableModelRCUnit
St7DisableModelRCUnit.argtypes = [c_long]
St7GetResultCaseName = _ST7API.St7GetResultCaseName
St7GetResultCaseName.argtypes = [c_long, c_long, c_char_p, c_long]
St7GetResultFreedomCaseName = _ST7API.St7GetResultFreedomCaseName
St7GetResultFreedomCaseName.argtypes = [c_long, c_char_p, c_long]
St7GetResultCaseStage = _ST7API.St7GetResultCaseStage
St7GetResultCaseStage.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7GetResultCaseConvergence = _ST7API.St7GetResultCaseConvergence
St7GetResultCaseConvergence.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7GetResultCaseReset = _ST7API.St7GetResultCaseReset
St7GetResultCaseReset.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7GetResultCaseTime = _ST7API.St7GetResultCaseTime
St7GetResultCaseTime.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetResultCaseFactor = _ST7API.St7GetResultCaseFactor
St7GetResultCaseFactor.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetFrequency = _ST7API.St7GetFrequency
St7GetFrequency.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetNumModes = _ST7API.St7GetNumModes
St7GetNumModes.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetNumSRACases = _ST7API.St7GetNumSRACases
St7GetNumSRACases.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetModalResultsNFA = _ST7API.St7GetModalResultsNFA
St7GetModalResultsNFA.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetModalResultsSRA = _ST7API.St7GetModalResultsSRA
St7GetModalResultsSRA.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetModalResultsHRA = _ST7API.St7GetModalResultsHRA
St7GetModalResultsHRA.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetInertiaReliefResults = _ST7API.St7GetInertiaReliefResults
St7GetInertiaReliefResults.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBuckFactor = _ST7API.St7GetBuckFactor
St7GetBuckFactor.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeResult = _ST7API.St7GetNodeResult
St7GetNodeResult.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetNodeResultUCS = _ST7API.St7GetNodeResultUCS
St7GetNodeResultUCS.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBeamResultArray = _ST7API.St7GetBeamResultArray
St7GetBeamResultArray.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7GetBeamResultArrayPos = _ST7API.St7GetBeamResultArrayPos
St7GetBeamResultArrayPos.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamResultEndPos = _ST7API.St7GetBeamResultEndPos
St7GetBeamResultEndPos.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamResultSinglePos = _ST7API.St7GetBeamResultSinglePos
St7GetBeamResultSinglePos.argtypes = [c_long, c_long, c_long, c_long, c_long, c_double, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBeamSectionResult = _ST7API.St7GetBeamSectionResult
St7GetBeamSectionResult.argtypes = [c_long, c_long, c_long, c_long, c_double, c_double, c_double, ctypes.POINTER(c_double)]
St7GetBeamReleaseResult = _ST7API.St7GetBeamReleaseResult
St7GetBeamReleaseResult.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool), ctypes.POINTER(c_double)]
St7GetPlateResultArray = _ST7API.St7GetPlateResultArray
St7GetPlateResultArray.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetPlateResultMaxJunctionAngle = _ST7API.St7SetPlateResultMaxJunctionAngle
St7SetPlateResultMaxJunctionAngle.argtypes = [c_long, c_double]
St7GetPlateResultMaxJunctionAngle = _ST7API.St7GetPlateResultMaxJunctionAngle
St7GetPlateResultMaxJunctionAngle.argtypes = [c_long, ctypes.POINTER(c_double)]
St7GetPlateResultGaussPoints = _ST7API.St7GetPlateResultGaussPoints
St7GetPlateResultGaussPoints.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetBrickResultArray = _ST7API.St7GetBrickResultArray
St7GetBrickResultArray.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetLinkResultArray = _ST7API.St7GetLinkResultArray
St7GetLinkResultArray.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double), c_long]
St7GetMultiPointLinkReactionSum = _ST7API.St7GetMultiPointLinkReactionSum
St7GetMultiPointLinkReactionSum.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetMultiPointLinkFluxSum = _ST7API.St7GetMultiPointLinkFluxSum
St7GetMultiPointLinkFluxSum.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickResultGaussPoints = _ST7API.St7GetBrickResultGaussPoints
St7GetBrickResultGaussPoints.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetReferenceDisplacement = _ST7API.St7SetReferenceDisplacement
St7SetReferenceDisplacement.argtypes = [c_long, c_long, c_bool]
St7GetNodeReactionSum = _ST7API.St7GetNodeReactionSum
St7GetNodeReactionSum.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double), c_long, ctypes.POINTER(c_double)]
St7GetElementNodeForceSum = _ST7API.St7GetElementNodeForceSum
St7GetElementNodeForceSum.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetNodeFluxSum = _ST7API.St7GetNodeFluxSum
St7GetNodeFluxSum.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetElementNodeFluxSum = _ST7API.St7GetElementNodeFluxSum
St7GetElementNodeFluxSum.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7EnablePlyPropertyResults = _ST7API.St7EnablePlyPropertyResults
St7EnablePlyPropertyResults.argtypes = [c_long, c_long]
St7DisablePlyPropertyResults = _ST7API.St7DisablePlyPropertyResults
St7DisablePlyPropertyResults.argtypes = [c_long, c_long]
St7GetPlyPropertyResultsState = _ST7API.St7GetPlyPropertyResultsState
St7GetPlyPropertyResultsState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetResultUserEquation = _ST7API.St7SetResultUserEquation
St7SetResultUserEquation.argtypes = [c_long, c_long, c_char_p, c_long]
St7GetResultUserEquation = _ST7API.St7GetResultUserEquation
St7GetResultUserEquation.argtypes = [c_long, c_long, c_char_p, c_long, ctypes.POINTER(c_long)]
St7StoreResultUserEquation = _ST7API.St7StoreResultUserEquation
St7StoreResultUserEquation.argtypes = [c_long, c_long, c_char_p, c_char_p, c_long]
St7DeleteStoredResultUserEquation = _ST7API.St7DeleteStoredResultUserEquation
St7DeleteStoredResultUserEquation.argtypes = [c_long, c_long, c_long]
St7ReplaceStoredResultUserEquation = _ST7API.St7ReplaceStoredResultUserEquation
St7ReplaceStoredResultUserEquation.argtypes = [c_long, c_long, c_long, c_char_p, c_char_p, c_long]
St7RetrieveStoredResultUserEquation = _ST7API.St7RetrieveStoredResultUserEquation
St7RetrieveStoredResultUserEquation.argtypes = [c_long, c_long, c_long, c_char_p, c_char_p, c_long, ctypes.POINTER(c_long)]
St7GetNumStoredResultUserEquations = _ST7API.St7GetNumStoredResultUserEquations
St7GetNumStoredResultUserEquations.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetStoredResultUserEquation = _ST7API.St7SetStoredResultUserEquation
St7SetStoredResultUserEquation.argtypes = [c_long, c_long, c_long]
St7GeneratePlateContourFile = _ST7API.St7GeneratePlateContourFile
St7GeneratePlateContourFile.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GenerateBrickContourFile = _ST7API.St7GenerateBrickContourFile
St7GenerateBrickContourFile.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7LoadPlateContourFile = _ST7API.St7LoadPlateContourFile
St7LoadPlateContourFile.argtypes = [c_long, c_long]
St7LoadBrickContourFile = _ST7API.St7LoadBrickContourFile
St7LoadBrickContourFile.argtypes = [c_long, c_long]
St7GetPlateContourFileResult = _ST7API.St7GetPlateContourFileResult
St7GetPlateContourFileResult.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetBrickContourFileResult = _ST7API.St7GetBrickContourFileResult
St7GetBrickContourFileResult.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7GetNumLSACombinations = _ST7API.St7GetNumLSACombinations
St7GetNumLSACombinations.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetLSACombinationName = _ST7API.St7SetLSACombinationName
St7SetLSACombinationName.argtypes = [c_long, c_long, c_char_p]
St7GetLSACombinationName = _ST7API.St7GetLSACombinationName
St7GetLSACombinationName.argtypes = [c_long, c_long, c_char_p, c_long]
St7SetLSACombinationSRAName = _ST7API.St7SetLSACombinationSRAName
St7SetLSACombinationSRAName.argtypes = [c_long, c_char_p]
St7GetLSACombinationSRAName = _ST7API.St7GetLSACombinationSRAName
St7GetLSACombinationSRAName.argtypes = [c_long, c_char_p, c_long]
St7AddLSACombination = _ST7API.St7AddLSACombination
St7AddLSACombination.argtypes = [c_long, c_char_p]
St7InsertLSACombination = _ST7API.St7InsertLSACombination
St7InsertLSACombination.argtypes = [c_long, c_long, c_char_p]
St7DeleteLSACombination = _ST7API.St7DeleteLSACombination
St7DeleteLSACombination.argtypes = [c_long, c_long]
St7SetLSACombinationFactor = _ST7API.St7SetLSACombinationFactor
St7SetLSACombinationFactor.argtypes = [c_long, c_long, c_long, c_long, c_long, c_double]
St7GetLSACombinationFactor = _ST7API.St7GetLSACombinationFactor
St7GetLSACombinationFactor.argtypes = [c_long, c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetLSACombinationState = _ST7API.St7SetLSACombinationState
St7SetLSACombinationState.argtypes = [c_long, c_long, c_bool]
St7GetLSACombinationState = _ST7API.St7GetLSACombinationState
St7GetLSACombinationState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7GetNumCombinedLSACombinations = _ST7API.St7GetNumCombinedLSACombinations
St7GetNumCombinedLSACombinations.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetCombinedLSACombinationName = _ST7API.St7SetCombinedLSACombinationName
St7SetCombinedLSACombinationName.argtypes = [c_long, c_long, c_char_p]
St7GetCombinedLSACombinationName = _ST7API.St7GetCombinedLSACombinationName
St7GetCombinedLSACombinationName.argtypes = [c_long, c_long, c_char_p, c_long]
St7SetCombinedLSACombinationState = _ST7API.St7SetCombinedLSACombinationState
St7SetCombinedLSACombinationState.argtypes = [c_long, c_long, c_bool]
St7GetCombinedLSACombinationState = _ST7API.St7GetCombinedLSACombinationState
St7GetCombinedLSACombinationState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7AddCombinedLSACombination = _ST7API.St7AddCombinedLSACombination
St7AddCombinedLSACombination.argtypes = [c_long, c_char_p]
St7InsertCombinedLSACombination = _ST7API.St7InsertCombinedLSACombination
St7InsertCombinedLSACombination.argtypes = [c_long, c_long, c_char_p]
St7DeleteCombinedLSACombination = _ST7API.St7DeleteCombinedLSACombination
St7DeleteCombinedLSACombination.argtypes = [c_long, c_long]
St7SetCombinedLSACombinationFactor = _ST7API.St7SetCombinedLSACombinationFactor
St7SetCombinedLSACombinationFactor.argtypes = [c_long, c_long, c_long, c_double]
St7GetCombinedLSACombinationFactor = _ST7API.St7GetCombinedLSACombinationFactor
St7GetCombinedLSACombinationFactor.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetHRABaseCombinationFactor = _ST7API.St7SetHRABaseCombinationFactor
St7SetHRABaseCombinationFactor.argtypes = [c_long, c_double]
St7GetHRABaseCombinationFactor = _ST7API.St7GetHRABaseCombinationFactor
St7GetHRABaseCombinationFactor.argtypes = [c_long, ctypes.POINTER(c_double)]
St7SetHRACaseCombinationFactor = _ST7API.St7SetHRACaseCombinationFactor
St7SetHRACaseCombinationFactor.argtypes = [c_long, c_long, c_double]
St7GetHRACaseCombinationFactor = _ST7API.St7GetHRACaseCombinationFactor
St7GetHRACaseCombinationFactor.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetHRACombinationFactorLSA = _ST7API.St7SetHRACombinationFactorLSA
St7SetHRACombinationFactorLSA.argtypes = [c_long, c_long, c_long, c_double]
St7GetHRACombinationFactorLSA = _ST7API.St7GetHRACombinationFactorLSA
St7GetHRACombinationFactorLSA.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetHRACombinationLSAName = _ST7API.St7SetHRACombinationLSAName
St7SetHRACombinationLSAName.argtypes = [c_long, c_char_p]
St7GetHRACombinationLSAName = _ST7API.St7GetHRACombinationLSAName
St7GetHRACombinationLSAName.argtypes = [c_long, c_char_p, c_long]
St7SetInfluenceFileName = _ST7API.St7SetInfluenceFileName
St7SetInfluenceFileName.argtypes = [c_long, c_char_p]
St7GetInfluenceFileName = _ST7API.St7GetInfluenceFileName
St7GetInfluenceFileName.argtypes = [c_long, c_char_p, c_long]
St7GetNumInfluenceVariables = _ST7API.St7GetNumInfluenceVariables
St7GetNumInfluenceVariables.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetNumInfluenceMultiVariableCases = _ST7API.St7GetNumInfluenceMultiVariableCases
St7GetNumInfluenceMultiVariableCases.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetInfluenceVariableData = _ST7API.St7GetInfluenceVariableData
St7GetInfluenceVariableData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7SetInfluenceMinVariableState = _ST7API.St7SetInfluenceMinVariableState
St7SetInfluenceMinVariableState.argtypes = [c_long, c_long, c_bool]
St7GetInfluenceMinVariableState = _ST7API.St7GetInfluenceMinVariableState
St7GetInfluenceMinVariableState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetInfluenceMaxVariableState = _ST7API.St7SetInfluenceMaxVariableState
St7SetInfluenceMaxVariableState.argtypes = [c_long, c_long, c_bool]
St7GetInfluenceMaxVariableState = _ST7API.St7GetInfluenceMaxVariableState
St7GetInfluenceMaxVariableState.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetInfluenceMultiVariableState = _ST7API.St7SetInfluenceMultiVariableState
St7SetInfluenceMultiVariableState.argtypes = [c_long, c_long, c_long, c_bool]
St7GetInfluenceMultiVariableState = _ST7API.St7GetInfluenceMultiVariableState
St7GetInfluenceMultiVariableState.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7SetInfluenceMultiVariableType = _ST7API.St7SetInfluenceMultiVariableType
St7SetInfluenceMultiVariableType.argtypes = [c_long, c_long, c_long]
St7GetInfluenceMultiVariableType = _ST7API.St7GetInfluenceMultiVariableType
St7GetInfluenceMultiVariableType.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7AddInfluenceMultiVariableCase = _ST7API.St7AddInfluenceMultiVariableCase
St7AddInfluenceMultiVariableCase.argtypes = [c_long, c_long, c_char_p]
St7DeleteInfluenceMultiVariableCase = _ST7API.St7DeleteInfluenceMultiVariableCase
St7DeleteInfluenceMultiVariableCase.argtypes = [c_long, c_long]
St7SetInfluenceMultiVariableName = _ST7API.St7SetInfluenceMultiVariableName
St7SetInfluenceMultiVariableName.argtypes = [c_long, c_long, c_char_p]
St7GetInfluenceMultiVariableName = _ST7API.St7GetInfluenceMultiVariableName
St7GetInfluenceMultiVariableName.argtypes = [c_long, c_long, c_char_p, c_long]
St7SetInfluenceGroupStatus = _ST7API.St7SetInfluenceGroupStatus
St7SetInfluenceGroupStatus.argtypes = [c_long, c_long, c_bool]
St7GetInfluenceGroupStatus = _ST7API.St7GetInfluenceGroupStatus
St7GetInfluenceGroupStatus.argtypes = [c_long, c_long, ctypes.POINTER(c_bool)]
St7SetInfluencePropertyStatus = _ST7API.St7SetInfluencePropertyStatus
St7SetInfluencePropertyStatus.argtypes = [c_long, c_long, c_long, c_bool]
St7GetInfluencePropertyStatus = _ST7API.St7GetInfluencePropertyStatus
St7GetInfluencePropertyStatus.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7SetInfluenceCombinationOptions = _ST7API.St7SetInfluenceCombinationOptions
St7SetInfluenceCombinationOptions.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetInfluenceCombinationOptions = _ST7API.St7GetInfluenceCombinationOptions
St7GetInfluenceCombinationOptions.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GenerateInfluenceCases = _ST7API.St7GenerateInfluenceCases
St7GenerateInfluenceCases.argtypes = [c_long, c_bool, c_bool, c_bool, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetNumEnvelopes = _ST7API.St7GetNumEnvelopes
St7GetNumEnvelopes.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7AddLimitEnvelope = _ST7API.St7AddLimitEnvelope
St7AddLimitEnvelope.argtypes = [c_long, c_long, c_char_p]
St7InsertLimitEnvelope = _ST7API.St7InsertLimitEnvelope
St7InsertLimitEnvelope.argtypes = [c_long, c_long, c_long, c_char_p]
St7DeleteLimitEnvelope = _ST7API.St7DeleteLimitEnvelope
St7DeleteLimitEnvelope.argtypes = [c_long, c_long]
St7EnableLimitEnvelopeCase = _ST7API.St7EnableLimitEnvelopeCase
St7EnableLimitEnvelopeCase.argtypes = [c_long, c_long, c_long]
St7DisableLimitEnvelopeCase = _ST7API.St7DisableLimitEnvelopeCase
St7DisableLimitEnvelopeCase.argtypes = [c_long, c_long, c_long]
St7GetLimitEnvelopeCaseState = _ST7API.St7GetLimitEnvelopeCaseState
St7GetLimitEnvelopeCaseState.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7SetLimitEnvelopeData = _ST7API.St7SetLimitEnvelopeData
St7SetLimitEnvelopeData.argtypes = [c_long, c_long, c_long, c_char_p]
St7GetLimitEnvelopeData = _ST7API.St7GetLimitEnvelopeData
St7GetLimitEnvelopeData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), c_char_p, c_long]
St7AddCombinationEnvelope = _ST7API.St7AddCombinationEnvelope
St7AddCombinationEnvelope.argtypes = [c_long, c_long, c_char_p]
St7InsertCombinationEnvelope = _ST7API.St7InsertCombinationEnvelope
St7InsertCombinationEnvelope.argtypes = [c_long, c_long, c_long, c_char_p]
St7DeleteCombinationEnvelope = _ST7API.St7DeleteCombinationEnvelope
St7DeleteCombinationEnvelope.argtypes = [c_long, c_long]
St7SetCombinationEnvelopeCase = _ST7API.St7SetCombinationEnvelopeCase
St7SetCombinationEnvelopeCase.argtypes = [c_long, c_long, c_long, c_long]
St7GetCombinationEnvelopeCase = _ST7API.St7GetCombinationEnvelopeCase
St7GetCombinationEnvelopeCase.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7SetCombinationEnvelopeData = _ST7API.St7SetCombinationEnvelopeData
St7SetCombinationEnvelopeData.argtypes = [c_long, c_long, c_long, c_char_p]
St7GetCombinationEnvelopeData = _ST7API.St7GetCombinationEnvelopeData
St7GetCombinationEnvelopeData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), c_char_p, c_long]
St7AddFactorsEnvelope = _ST7API.St7AddFactorsEnvelope
St7AddFactorsEnvelope.argtypes = [c_long, c_long, c_char_p]
St7InsertFactorsEnvelope = _ST7API.St7InsertFactorsEnvelope
St7InsertFactorsEnvelope.argtypes = [c_long, c_long, c_long, c_char_p]
St7DeleteFactorsEnvelope = _ST7API.St7DeleteFactorsEnvelope
St7DeleteFactorsEnvelope.argtypes = [c_long, c_long]
St7SetFactorsEnvelopeData = _ST7API.St7SetFactorsEnvelopeData
St7SetFactorsEnvelopeData.argtypes = [c_long, c_long, c_long, c_char_p]
St7GetFactorsEnvelopeData = _ST7API.St7GetFactorsEnvelopeData
St7GetFactorsEnvelopeData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), c_char_p, c_long]
St7AddFactorsEnvelopeCase = _ST7API.St7AddFactorsEnvelopeCase
St7AddFactorsEnvelopeCase.argtypes = [c_long, c_long]
St7InsertFactorsEnvelopeCase = _ST7API.St7InsertFactorsEnvelopeCase
St7InsertFactorsEnvelopeCase.argtypes = [c_long, c_long, c_long]
St7DeleteFactorsEnvelopeCase = _ST7API.St7DeleteFactorsEnvelopeCase
St7DeleteFactorsEnvelopeCase.argtypes = [c_long, c_long, c_long]
St7SetFactorsEnvelopeCaseData = _ST7API.St7SetFactorsEnvelopeCaseData
St7SetFactorsEnvelopeCaseData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetFactorsEnvelopeCaseData = _ST7API.St7GetFactorsEnvelopeCaseData
St7GetFactorsEnvelopeCaseData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7AddFactorsEnvelopeSet = _ST7API.St7AddFactorsEnvelopeSet
St7AddFactorsEnvelopeSet.argtypes = [c_long]
St7InsertFactorsEnvelopeSet = _ST7API.St7InsertFactorsEnvelopeSet
St7InsertFactorsEnvelopeSet.argtypes = [c_long, c_long]
St7DeleteFactorsEnvelopeSet = _ST7API.St7DeleteFactorsEnvelopeSet
St7DeleteFactorsEnvelopeSet.argtypes = [c_long, c_long]
St7GetNumFactorsEnvelopeSets = _ST7API.St7GetNumFactorsEnvelopeSets
St7GetNumFactorsEnvelopeSets.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetFactorsEnvelopeSetData = _ST7API.St7SetFactorsEnvelopeSetData
St7SetFactorsEnvelopeSetData.argtypes = [c_long, c_long, c_long, c_char_p, c_char_p]
St7GetFactorsEnvelopeSetData = _ST7API.St7GetFactorsEnvelopeSetData
St7GetFactorsEnvelopeSetData.argtypes = [c_long, c_long, ctypes.POINTER(c_long), c_char_p, c_char_p, c_long]
St7SetResultFileCombTargetFileName = _ST7API.St7SetResultFileCombTargetFileName
St7SetResultFileCombTargetFileName.argtypes = [c_long, c_char_p]
St7GetResultFileCombTargetFileName = _ST7API.St7GetResultFileCombTargetFileName
St7GetResultFileCombTargetFileName.argtypes = [c_long, c_char_p, c_long]
St7AddResultFileCombFileName = _ST7API.St7AddResultFileCombFileName
St7AddResultFileCombFileName.argtypes = [c_long, c_char_p]
St7DeleteResultFileCombFileName = _ST7API.St7DeleteResultFileCombFileName
St7DeleteResultFileCombFileName.argtypes = [c_long, c_long]
St7SetResultFileCombFileName = _ST7API.St7SetResultFileCombFileName
St7SetResultFileCombFileName.argtypes = [c_long, c_long, c_char_p]
St7GetResultFileCombFileName = _ST7API.St7GetResultFileCombFileName
St7GetResultFileCombFileName.argtypes = [c_long, c_long, c_char_p, c_long]
St7AddResultFileCombCase = _ST7API.St7AddResultFileCombCase
St7AddResultFileCombCase.argtypes = [c_long, c_char_p]
St7DeleteResultFileCombCase = _ST7API.St7DeleteResultFileCombCase
St7DeleteResultFileCombCase.argtypes = [c_long, c_long]
St7SetResultFileCombCaseData = _ST7API.St7SetResultFileCombCaseData
St7SetResultFileCombCaseData.argtypes = [c_long, c_long, c_long, c_long, c_double]
St7GetResultFileCombCaseData = _ST7API.St7GetResultFileCombCaseData
St7GetResultFileCombCaseData.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetResultFileCombCaseName = _ST7API.St7SetResultFileCombCaseName
St7SetResultFileCombCaseName.argtypes = [c_long, c_long, c_char_p]
St7GetResultFileCombCaseName = _ST7API.St7GetResultFileCombCaseName
St7GetResultFileCombCaseName.argtypes = [c_long, c_long, c_char_p, c_long]
St7GenerateResultFileComb = _ST7API.St7GenerateResultFileComb
St7GenerateResultFileComb.argtypes = [c_long, c_long]
St7RetrieveResultFileComb = _ST7API.St7RetrieveResultFileComb
St7RetrieveResultFileComb.argtypes = [c_long, c_char_p]
St7GenerateHRATimeHistory = _ST7API.St7GenerateHRATimeHistory
St7GenerateHRATimeHistory.argtypes = [c_long, c_double, c_double, c_long, ctypes.POINTER(c_long)]
St7ClearHRATimeHistory = _ST7API.St7ClearHRATimeHistory
St7ClearHRATimeHistory.argtypes = [c_long]
St7NewResFile = _ST7API.St7NewResFile
St7NewResFile.argtypes = [c_long, c_char_p, c_long]
St7GetResFileUnits = _ST7API.St7GetResFileUnits
St7GetResFileUnits.argtypes = [c_long, ctypes.POINTER(c_long)]
St7OpenResFile = _ST7API.St7OpenResFile
St7OpenResFile.argtypes = [c_long, c_char_p]
St7CloseResFile = _ST7API.St7CloseResFile
St7CloseResFile.argtypes = [c_long]
St7SetResFileDescription = _ST7API.St7SetResFileDescription
St7SetResFileDescription.argtypes = [c_long, c_char_p]
St7GetResFileDescription = _ST7API.St7GetResFileDescription
St7GetResFileDescription.argtypes = [c_long, c_char_p, c_long]
St7SetResFileNumCases = _ST7API.St7SetResFileNumCases
St7SetResFileNumCases.argtypes = [c_long, c_long]
St7SetResFileCaseName = _ST7API.St7SetResFileCaseName
St7SetResFileCaseName.argtypes = [c_long, c_long, c_char_p]
St7AssociateResFileCase = _ST7API.St7AssociateResFileCase
St7AssociateResFileCase.argtypes = [c_long, c_long, c_long, c_long]
St7AssociateResFileStage = _ST7API.St7AssociateResFileStage
St7AssociateResFileStage.argtypes = [c_long, c_long, c_long]
St7AssociateResFileNSMassCase = _ST7API.St7AssociateResFileNSMassCase
St7AssociateResFileNSMassCase.argtypes = [c_long, c_long, c_double]
St7SetResFileSRACases = _ST7API.St7SetResFileSRACases
St7SetResFileSRACases.argtypes = [c_long, c_long]
St7SetResFileMode = _ST7API.St7SetResFileMode
St7SetResFileMode.argtypes = [c_long, c_long, c_double]
St7GetResFileMode = _ST7API.St7GetResFileMode
St7GetResFileMode.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetResFileTime = _ST7API.St7SetResFileTime
St7SetResFileTime.argtypes = [c_long, c_long, c_double]
St7GetResFileTime = _ST7API.St7GetResFileTime
St7GetResFileTime.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7SetResFileTimeUnit = _ST7API.St7SetResFileTimeUnit
St7SetResFileTimeUnit.argtypes = [c_long, c_long]
St7GetResFileTimeUnit = _ST7API.St7GetResFileTimeUnit
St7GetResFileTimeUnit.argtypes = [c_long, ctypes.POINTER(c_long)]
St7SetResFileQuantity = _ST7API.St7SetResFileQuantity
St7SetResFileQuantity.argtypes = [c_long, c_long, c_long, c_long]
St7SetResFileFreedomCase = _ST7API.St7SetResFileFreedomCase
St7SetResFileFreedomCase.argtypes = [c_long, c_long]
St7GetResFileFreedomCase = _ST7API.St7GetResFileFreedomCase
St7GetResFileFreedomCase.argtypes = [c_long, ctypes.POINTER(c_long)]
St7ClearResFileQuantity = _ST7API.St7ClearResFileQuantity
St7ClearResFileQuantity.argtypes = [c_long, c_long, c_long, c_long]
St7GetResFileQuantity = _ST7API.St7GetResFileQuantity
St7GetResFileQuantity.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_bool)]
St7SetResFileNodeResult = _ST7API.St7SetResFileNodeResult
St7SetResFileNodeResult.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetResFileNodeResult = _ST7API.St7GetResFileNodeResult
St7GetResFileNodeResult.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetResFileBeamResult = _ST7API.St7SetResFileBeamResult
St7SetResFileBeamResult.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetResFileBeamResult = _ST7API.St7GetResFileBeamResult
St7GetResFileBeamResult.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetResFileBeamReleaseResult = _ST7API.St7SetResFileBeamReleaseResult
St7SetResFileBeamReleaseResult.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool), ctypes.POINTER(c_double)]
St7GetResFileBeamReleaseResult = _ST7API.St7GetResFileBeamReleaseResult
St7GetResFileBeamReleaseResult.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_bool), ctypes.POINTER(c_double)]
St7SetResFileBeamStations = _ST7API.St7SetResFileBeamStations
St7SetResFileBeamStations.argtypes = [c_long, c_long, c_long]
St7GetResFileBeamStations = _ST7API.St7GetResFileBeamStations
St7GetResFileBeamStations.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetResFilePlateResult = _ST7API.St7SetResFilePlateResult
St7SetResFilePlateResult.argtypes = [c_long, c_long, c_long, c_long, c_bool, ctypes.POINTER(c_double)]
St7GetResFilePlateResult = _ST7API.St7GetResFilePlateResult
St7GetResFilePlateResult.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_bool), ctypes.POINTER(c_double)]
St7SetResFilePlatePressureResult = _ST7API.St7SetResFilePlatePressureResult
St7SetResFilePlatePressureResult.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetResFilePlatePressureResult = _ST7API.St7GetResFilePlatePressureResult
St7GetResFilePlatePressureResult.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetResFileBrickResult = _ST7API.St7SetResFileBrickResult
St7SetResFileBrickResult.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7GetResFileBrickResult = _ST7API.St7GetResFileBrickResult
St7GetResFileBrickResult.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SetToolOptions = _ST7API.St7SetToolOptions
St7SetToolOptions.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetToolOptions = _ST7API.St7GetToolOptions
St7GetToolOptions.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7SetCleanMeshOptions = _ST7API.St7SetCleanMeshOptions
St7SetCleanMeshOptions.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7GetCleanMeshOptions = _ST7API.St7GetCleanMeshOptions
St7GetCleanMeshOptions.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double)]
St7CleanMesh = _ST7API.St7CleanMesh
St7CleanMesh.argtypes = [c_long]
St7SurfaceMesh = _ST7API.St7SurfaceMesh
St7SurfaceMesh.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double), c_long]
St7SolidTetMesh = _ST7API.St7SolidTetMesh
St7SolidTetMesh.argtypes = [c_long, ctypes.POINTER(c_long), c_long]
St7DirectSolidTetMesh = _ST7API.St7DirectSolidTetMesh
St7DirectSolidTetMesh.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double), c_long]
St7MeshFromLoops = _ST7API.St7MeshFromLoops
St7MeshFromLoops.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_double), ctypes.POINTER(c_long), ctypes.POINTER(c_double), c_long]
St7DeleteUnusedNodes = _ST7API.St7DeleteUnusedNodes
St7DeleteUnusedNodes.argtypes = [c_long, ctypes.POINTER(c_long)]
St7InvalidateElement = _ST7API.St7InvalidateElement
St7InvalidateElement.argtypes = [c_long, c_long, c_long]
St7DeleteInvalidElements = _ST7API.St7DeleteInvalidElements
St7DeleteInvalidElements.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SetPasteOptions = _ST7API.St7SetPasteOptions
St7SetPasteOptions.argtypes = [c_long, ctypes.POINTER(c_long)]
St7CopyToSt7Clipboard = _ST7API.St7CopyToSt7Clipboard
St7CopyToSt7Clipboard.argtypes = [c_long]
St7CutToSt7Clipboard = _ST7API.St7CutToSt7Clipboard
St7CutToSt7Clipboard.argtypes = [c_long]
St7PasteFromSt7ClipboardByIncrements = _ST7API.St7PasteFromSt7ClipboardByIncrements
St7PasteFromSt7ClipboardByIncrements.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double]
St7PasteFromSt7ClipboardByAnchors = _ST7API.St7PasteFromSt7ClipboardByAnchors
St7PasteFromSt7ClipboardByAnchors.argtypes = [c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double]
St7SetProjectDirectionAsSource = _ST7API.St7SetProjectDirectionAsSource
St7SetProjectDirectionAsSource.argtypes = [c_long]
St7SetProjectDirectionAsTarget = _ST7API.St7SetProjectDirectionAsTarget
St7SetProjectDirectionAsTarget.argtypes = [c_long]
St7SetProjectDirectionAsConical = _ST7API.St7SetProjectDirectionAsConical
St7SetProjectDirectionAsConical.argtypes = [c_long, ctypes.POINTER(c_double)]
St7SetProjectDirectionAsParallel = _ST7API.St7SetProjectDirectionAsParallel
St7SetProjectDirectionAsParallel.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7SetPropertyIncrement = _ST7API.St7SetPropertyIncrement
St7SetPropertyIncrement.argtypes = [c_long, c_long]
St7SetKeepSelect = _ST7API.St7SetKeepSelect
St7SetKeepSelect.argtypes = [c_long, c_bool]
St7SetCopyFlags = _ST7API.St7SetCopyFlags
St7SetCopyFlags.argtypes = [c_long, c_bool, c_bool, c_bool, c_bool]
St7SetExtrudeTarget = _ST7API.St7SetExtrudeTarget
St7SetExtrudeTarget.argtypes = [c_long, c_long, c_long, c_long, c_long]
St7SetSourceAction = _ST7API.St7SetSourceAction
St7SetSourceAction.argtypes = [c_long, c_long]
St7SetPLTarget = _ST7API.St7SetPLTarget
St7SetPLTarget.argtypes = [c_long, c_long, c_long]
St7DefineLineN2 = _ST7API.St7DefineLineN2
St7DefineLineN2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7DefineLineV2 = _ST7API.St7DefineLineV2
St7DefineLineV2.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7DefineLineNV = _ST7API.St7DefineLineNV
St7DefineLineNV.argtypes = [c_long, c_long, c_long, c_bool, ctypes.POINTER(c_long)]
St7DefineLineP2 = _ST7API.St7DefineLineP2
St7DefineLineP2.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_long)]
St7DefinePlaneGlobalN = _ST7API.St7DefinePlaneGlobalN
St7DefinePlaneGlobalN.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7DefinePlaneGlobalV = _ST7API.St7DefinePlaneGlobalV
St7DefinePlaneGlobalV.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7DefinePlaneP3 = _ST7API.St7DefinePlaneP3
St7DefinePlaneP3.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_long)]
St7DefinePlaneUCS = _ST7API.St7DefinePlaneUCS
St7DefinePlaneUCS.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long)]
St7DefineEntityCollection = _ST7API.St7DefineEntityCollection
St7DefineEntityCollection.argtypes = [c_long, ctypes.POINTER(c_long)]
St7CopyByIncrement = _ST7API.St7CopyByIncrement
St7CopyByIncrement.argtypes = [c_long, ctypes.POINTER(c_double), c_long, c_long]
St7CopyByRotation = _ST7API.St7CopyByRotation
St7CopyByRotation.argtypes = [c_long, c_long, c_long, c_double, ctypes.POINTER(c_double), c_long]
St7CopyByProjectionToLine = _ST7API.St7CopyByProjectionToLine
St7CopyByProjectionToLine.argtypes = [c_long, c_long, c_bool]
St7CopyByProjectionToPlane = _ST7API.St7CopyByProjectionToPlane
St7CopyByProjectionToPlane.argtypes = [c_long, c_long]
St7CopyByProjectionToUCS = _ST7API.St7CopyByProjectionToUCS
St7CopyByProjectionToUCS.argtypes = [c_long, c_long, c_long, c_double]
St7CopyByProjectionToEntityFace = _ST7API.St7CopyByProjectionToEntityFace
St7CopyByProjectionToEntityFace.argtypes = [c_long, c_long]
St7CopyByThickness = _ST7API.St7CopyByThickness
St7CopyByThickness.argtypes = [c_long, c_double, c_long, c_long, c_long, c_bool, c_bool]
St7CopyByMirror = _ST7API.St7CopyByMirror
St7CopyByMirror.argtypes = [c_long, c_long]
St7CopyToAbsolute = _ST7API.St7CopyToAbsolute
St7CopyToAbsolute.argtypes = [c_long, c_double, c_long, c_long]
St7MoveByIncrement = _ST7API.St7MoveByIncrement
St7MoveByIncrement.argtypes = [c_long, ctypes.POINTER(c_double), c_long]
St7MoveByRotation = _ST7API.St7MoveByRotation
St7MoveByRotation.argtypes = [c_long, c_long, c_long, c_double, ctypes.POINTER(c_double)]
St7MoveByProjectionToLine = _ST7API.St7MoveByProjectionToLine
St7MoveByProjectionToLine.argtypes = [c_long, c_long, c_bool]
St7MoveByProjectionToPlane = _ST7API.St7MoveByProjectionToPlane
St7MoveByProjectionToPlane.argtypes = [c_long, c_long]
St7MoveByProjectionToUCS = _ST7API.St7MoveByProjectionToUCS
St7MoveByProjectionToUCS.argtypes = [c_long, c_long, c_long, c_double]
St7MoveByProjectionToEntityFace = _ST7API.St7MoveByProjectionToEntityFace
St7MoveByProjectionToEntityFace.argtypes = [c_long, c_long]
St7MoveByThickness = _ST7API.St7MoveByThickness
St7MoveByThickness.argtypes = [c_long, c_double, c_long, c_long, c_long, c_bool, c_bool]
St7MoveByMirror = _ST7API.St7MoveByMirror
St7MoveByMirror.argtypes = [c_long, c_long]
St7MoveBySkew = _ST7API.St7MoveBySkew
St7MoveBySkew.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_long]
St7MoveToAbsolute = _ST7API.St7MoveToAbsolute
St7MoveToAbsolute.argtypes = [c_long, c_double, c_long, c_long]
St7MoveToUCSIntersection = _ST7API.St7MoveToUCSIntersection
St7MoveToUCSIntersection.argtypes = [c_long, c_long, c_long, c_double, c_double]
St7MoveToOriginByPoint = _ST7API.St7MoveToOriginByPoint
St7MoveToOriginByPoint.argtypes = [c_long, c_long, ctypes.POINTER(c_double)]
St7MoveToOriginMinXYZ = _ST7API.St7MoveToOriginMinXYZ
St7MoveToOriginMinXYZ.argtypes = [c_long, c_long]
St7MoveToPlane = _ST7API.St7MoveToPlane
St7MoveToPlane.argtypes = [c_long, c_long, c_long]
St7ExtrudeByIncrement = _ST7API.St7ExtrudeByIncrement
St7ExtrudeByIncrement.argtypes = [c_long, ctypes.POINTER(c_double), c_long, c_long]
St7ExtrudeByRotation = _ST7API.St7ExtrudeByRotation
St7ExtrudeByRotation.argtypes = [c_long, c_long, c_long, c_double, ctypes.POINTER(c_double), c_long]
St7ExtrudeByProjectionToPoint = _ST7API.St7ExtrudeByProjectionToPoint
St7ExtrudeByProjectionToPoint.argtypes = [c_long, ctypes.POINTER(c_double)]
St7ExtrudeByProjectionToLine = _ST7API.St7ExtrudeByProjectionToLine
St7ExtrudeByProjectionToLine.argtypes = [c_long, c_long, c_bool]
St7ExtrudeByProjectionToPlane = _ST7API.St7ExtrudeByProjectionToPlane
St7ExtrudeByProjectionToPlane.argtypes = [c_long, c_long]
St7ExtrudeByProjectionToUCS = _ST7API.St7ExtrudeByProjectionToUCS
St7ExtrudeByProjectionToUCS.argtypes = [c_long, c_long, c_long, c_double]
St7ExtrudeByProjectionToEntityFace = _ST7API.St7ExtrudeByProjectionToEntityFace
St7ExtrudeByProjectionToEntityFace.argtypes = [c_long, c_long]
St7ExtrudeByThickness = _ST7API.St7ExtrudeByThickness
St7ExtrudeByThickness.argtypes = [c_long, c_double, c_long, c_long, c_bool, c_bool]
St7ExtrudeByLine = _ST7API.St7ExtrudeByLine
St7ExtrudeByLine.argtypes = [c_long, c_long, c_long, c_long]
St7ExtrudeToAbsolute = _ST7API.St7ExtrudeToAbsolute
St7ExtrudeToAbsolute.argtypes = [c_long, c_double, c_long, c_long]
St7ScaleByCartesianUCS = _ST7API.St7ScaleByCartesianUCS
St7ScaleByCartesianUCS.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7ScaleByCylindricalUCS = _ST7API.St7ScaleByCylindricalUCS
St7ScaleByCylindricalUCS.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double]
St7ScaleBySphericalUCS = _ST7API.St7ScaleBySphericalUCS
St7ScaleBySphericalUCS.argtypes = [c_long, c_long, c_double]
St7ScaleByToroidalUCS = _ST7API.St7ScaleByToroidalUCS
St7ScaleByToroidalUCS.argtypes = [c_long, c_long, c_double]
St7ScaleByTaper = _ST7API.St7ScaleByTaper
St7ScaleByTaper.argtypes = [c_long, c_long, c_long, c_long, c_double, c_double]
St7GraftEdgesToFaces = _ST7API.St7GraftEdgesToFaces
St7GraftEdgesToFaces.argtypes = [c_long, c_long, c_double]
St7IntersectEdges = _ST7API.St7IntersectEdges
St7IntersectEdges.argtypes = [c_long, c_long, c_double, c_bool]
St7MorphEdges = _ST7API.St7MorphEdges
St7MorphEdges.argtypes = [c_long]
St7SplitFaceByVertices = _ST7API.St7SplitFaceByVertices
St7SplitFaceByVertices.argtypes = [c_long, c_long, ctypes.POINTER(c_long)]
St7SplitFaceByPlane = _ST7API.St7SplitFaceByPlane
St7SplitFaceByPlane.argtypes = [c_long, c_long, c_long, c_long, c_double]
St7FaceFromPlate = _ST7API.St7FaceFromPlate
St7FaceFromPlate.argtypes = [c_long, c_bool, c_bool, c_bool]
St7FaceFromBeamPolygon = _ST7API.St7FaceFromBeamPolygon
St7FaceFromBeamPolygon.argtypes = [c_long, c_long, c_long, c_double, c_bool, c_bool]
St7FaceFromCavity = _ST7API.St7FaceFromCavity
St7FaceFromCavity.argtypes = [c_long]
St7RebuildFaces = _ST7API.St7RebuildFaces
St7RebuildFaces.argtypes = [c_long]
St7RebuildFacesUV = _ST7API.St7RebuildFacesUV
St7RebuildFacesUV.argtypes = [c_long, c_long, c_long, c_long, c_long]
St7ConvertToNURBS = _ST7API.St7ConvertToNURBS
St7ConvertToNURBS.argtypes = [c_long]
St7DeleteCavityLoops = _ST7API.St7DeleteCavityLoops
St7DeleteCavityLoops.argtypes = [c_long]
St7DetachFaces = _ST7API.St7DetachFaces
St7DetachFaces.argtypes = [c_long, c_long]
St7InsertVerticesOnEdge = _ST7API.St7InsertVerticesOnEdge
St7InsertVerticesOnEdge.argtypes = [c_long, c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7SubdivideEdges = _ST7API.St7SubdivideEdges
St7SubdivideEdges.argtypes = [c_long, c_long, c_long]
St7Subdivide = _ST7API.St7Subdivide
St7Subdivide.argtypes = [c_long, c_long, c_long, c_long, c_long, c_long]
St7Grade = _ST7API.St7Grade
St7Grade.argtypes = [c_long, c_long, c_double]
St7CutElementsByLine = _ST7API.St7CutElementsByLine
St7CutElementsByLine.argtypes = [c_long, c_long, c_long, c_long, c_long]
St7CutElementsByPlane = _ST7API.St7CutElementsByPlane
St7CutElementsByPlane.argtypes = [c_long, c_long, c_long, c_long, c_long]
St7CutElementsByUCS = _ST7API.St7CutElementsByUCS
St7CutElementsByUCS.argtypes = [c_long, c_long, c_long, c_long, c_long, c_double]
St7SplitBeams = _ST7API.St7SplitBeams
St7SplitBeams.argtypes = [c_long, c_double]
St7SubdivideBeams = _ST7API.St7SubdivideBeams
St7SubdivideBeams.argtypes = [c_long, c_double]
St7InterpolateBeamSections = _ST7API.St7InterpolateBeamSections
St7InterpolateBeamSections.argtypes = [c_long, c_long, c_long, c_long]
St7IntersectBeamsAndLinks = _ST7API.St7IntersectBeamsAndLinks
St7IntersectBeamsAndLinks.argtypes = [c_long, c_double, c_double, c_bool, c_bool, c_bool]
St7LoftBeams = _ST7API.St7LoftBeams
St7LoftBeams.argtypes = [c_long, c_long, c_long, c_long, c_long, c_bool, c_bool]
St7SliceOnPlane = _ST7API.St7SliceOnPlane
St7SliceOnPlane.argtypes = [c_long, c_long, c_long, c_long, c_double, c_bool, c_bool]
St7FilletPlates = _ST7API.St7FilletPlates
St7FilletPlates.argtypes = [c_long, c_double, c_bool]
St7MidPlanePlateProjection = _ST7API.St7MidPlanePlateProjection
St7MidPlanePlateProjection.argtypes = [c_long, c_long]
St7RepairTri3Mesh = _ST7API.St7RepairTri3Mesh
St7RepairTri3Mesh.argtypes = [c_long, c_double]
St7DetachElements = _ST7API.St7DetachElements
St7DetachElements.argtypes = [c_long, c_long, c_long, c_long, c_long]
St7PLLine2 = _ST7API.St7PLLine2
St7PLLine2.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_long]
St7PLParabola3 = _ST7API.St7PLParabola3
St7PLParabola3.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_long]
St7PLNormal3 = _ST7API.St7PLNormal3
St7PLNormal3.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7PLNormal3R = _ST7API.St7PLNormal3R
St7PLNormal3R.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double, c_long]
St7PLExtend2R = _ST7API.St7PLExtend2R
St7PLExtend2R.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double, c_long]
St7PLAverage2 = _ST7API.St7PLAverage2
St7PLAverage2.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_long]
St7PLFillet3R = _ST7API.St7PLFillet3R
St7PLFillet3R.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double, c_long]
St7PLFillet4R = _ST7API.St7PLFillet4R
St7PLFillet4R.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double, c_long]
St7PLCircleO3 = _ST7API.St7PLCircleO3
St7PLCircleO3.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_long, c_bool]
St7PLEllipseO3 = _ST7API.St7PLEllipseO3
St7PLEllipseO3.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_long, c_bool]
St7PLCurve3 = _ST7API.St7PLCurve3
St7PLCurve3.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_long]
St7PLCircleC3 = _ST7API.St7PLCircleC3
St7PLCircleC3.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_long, c_bool]
St7PLCirclesTangent3R = _ST7API.St7PLCirclesTangent3R
St7PLCirclesTangent3R.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double, c_double]
St7PLIntersect4 = _ST7API.St7PLIntersect4
St7PLIntersect4.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7PLCircleTangent3R = _ST7API.St7PLCircleTangent3R
St7PLCircleTangent3R.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double]
St7PLCircleCentre3 = _ST7API.St7PLCircleCentre3
St7PLCircleCentre3.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
St7PLCirclesIntersect3R = _ST7API.St7PLCirclesIntersect3R
St7PLCirclesIntersect3R.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double, c_double]
St7PLCircleLineInnerFillet3R = _ST7API.St7PLCircleLineInnerFillet3R
St7PLCircleLineInnerFillet3R.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double, c_double, c_long, c_bool]
St7PLCircleLineOuterFillet3R = _ST7API.St7PLCircleLineOuterFillet3R
St7PLCircleLineOuterFillet3R.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double, c_double, c_long, c_bool]
St7PLCircleLineIntersect3 = _ST7API.St7PLCircleLineIntersect3
St7PLCircleLineIntersect3.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double]
St7PLCirclesFillet3R = _ST7API.St7PLCirclesFillet3R
St7PLCirclesFillet3R.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double), c_double, c_double, c_double, c_long, c_bool]
St7CreateRigidLinkCluster = _ST7API.St7CreateRigidLinkCluster
St7CreateRigidLinkCluster.argtypes = [c_long, c_long, c_long, c_long]
St7CreatePinnedLinkCluster = _ST7API.St7CreatePinnedLinkCluster
St7CreatePinnedLinkCluster.argtypes = [c_long, c_long]
St7CreateMasterSlaveLinkCluster = _ST7API.St7CreateMasterSlaveLinkCluster
St7CreateMasterSlaveLinkCluster.argtypes = [c_long, c_long, c_long, c_long]
St7CreateSectorSymmetryLinkCluster = _ST7API.St7CreateSectorSymmetryLinkCluster
St7CreateSectorSymmetryLinkCluster.argtypes = [c_long, c_long, c_double, c_double, c_double, c_double]
St7CreateInterpolatedMultiPointLink = _ST7API.St7CreateInterpolatedMultiPointLink
St7CreateInterpolatedMultiPointLink.argtypes = [c_long, c_long, c_long]
St7CreateRigidMultiPointLink = _ST7API.St7CreateRigidMultiPointLink
St7CreateRigidMultiPointLink.argtypes = [c_long, c_long, c_long, c_long]
St7CreatePinnedMultiPointLink = _ST7API.St7CreatePinnedMultiPointLink
St7CreatePinnedMultiPointLink.argtypes = [c_long, c_long]
St7CreateMasterSlaveMultiPointLink = _ST7API.St7CreateMasterSlaveMultiPointLink
St7CreateMasterSlaveMultiPointLink.argtypes = [c_long, c_long, c_long, c_long]
St7CreateReactionMultiPointLink = _ST7API.St7CreateReactionMultiPointLink
St7CreateReactionMultiPointLink.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_double)]
St7CreateLinksFromMultiPointLink = _ST7API.St7CreateLinksFromMultiPointLink
St7CreateLinksFromMultiPointLink.argtypes = [c_long, c_bool]
St7CreateBeamsOnElementEdges = _ST7API.St7CreateBeamsOnElementEdges
St7CreateBeamsOnElementEdges.argtypes = [c_long, c_long, c_long, c_long, c_double, c_bool, c_bool, c_bool, c_bool, c_bool]
St7CreateBeamsOnGeometryEdges = _ST7API.St7CreateBeamsOnGeometryEdges
St7CreateBeamsOnGeometryEdges.argtypes = [c_long, c_long, c_long]
St7CreatePlatesOnBricks = _ST7API.St7CreatePlatesOnBricks
St7CreatePlatesOnBricks.argtypes = [c_long, c_bool]
St7CreateEntityUCS = _ST7API.St7CreateEntityUCS
St7CreateEntityUCS.argtypes = [c_long, c_long, c_long]
St7CreateLoadPatches = _ST7API.St7CreateLoadPatches
St7CreateLoadPatches.argtypes = [c_long, c_double, c_bool]
St7CreateAttachments = _ST7API.St7CreateAttachments
St7CreateAttachments.argtypes = [c_long, c_long, c_double, c_bool]
St7CreateCartesianSymmetryRestraints = _ST7API.St7CreateCartesianSymmetryRestraints
St7CreateCartesianSymmetryRestraints.argtypes = [c_long, c_long]
St7CreateCylindricalSymmetryRestraints = _ST7API.St7CreateCylindricalSymmetryRestraints
St7CreateCylindricalSymmetryRestraints.argtypes = [c_long, c_long, c_long, c_double, c_double, c_double]
St7MergeElementPairs = _ST7API.St7MergeElementPairs
St7MergeElementPairs.argtypes = [c_long, c_bool]
St7MergeLineOfBeams = _ST7API.St7MergeLineOfBeams
St7MergeLineOfBeams.argtypes = [c_long, c_double, c_long]
St7MergeTriToQuad = _ST7API.St7MergeTriToQuad
St7MergeTriToQuad.argtypes = [c_long, c_double, c_double, c_double]
St7ConvertBeamsToLinks = _ST7API.St7ConvertBeamsToLinks
St7ConvertBeamsToLinks.argtypes = [c_long, c_long, c_long, c_long]
St7ConvertLinksToBeams = _ST7API.St7ConvertLinksToBeams
St7ConvertLinksToBeams.argtypes = [c_long, c_long]
St7ConvertPatchLoads = _ST7API.St7ConvertPatchLoads
St7ConvertPatchLoads.argtypes = [c_long, c_long, c_bool]
St7CheckPatchLoads = _ST7API.St7CheckPatchLoads
St7CheckPatchLoads.argtypes = [c_long, c_long]
St7ConvertLoadPathsToLoadCases = _ST7API.St7ConvertLoadPathsToLoadCases
St7ConvertLoadPathsToLoadCases.argtypes = [c_long, c_bool, c_bool, c_bool]
St7ConvertBeamPolygonsToPlates = _ST7API.St7ConvertBeamPolygonsToPlates
St7ConvertBeamPolygonsToPlates.argtypes = [c_long, c_double, c_double, c_double, c_bool]
St7AdjustMidsideNodes = _ST7API.St7AdjustMidsideNodes
St7AdjustMidsideNodes.argtypes = [c_long, c_bool]
St7SmoothPlates = _ST7API.St7SmoothPlates
St7SmoothPlates.argtypes = [c_long, c_long, c_bool]
St7ReorderNodesTree = _ST7API.St7ReorderNodesTree
St7ReorderNodesTree.argtypes = [c_long, c_long]
St7ReorderNodesGeometry = _ST7API.St7ReorderNodesGeometry
St7ReorderNodesGeometry.argtypes = [c_long, ctypes.POINTER(c_double)]
St7ReorderNodesAMD = _ST7API.St7ReorderNodesAMD
St7ReorderNodesAMD.argtypes = [c_long]
St7CorrectAttachmentLinkGroups = _ST7API.St7CorrectAttachmentLinkGroups
St7CorrectAttachmentLinkGroups.argtypes = [c_long]
St7TrimMultiPointLinks = _ST7API.St7TrimMultiPointLinks
St7TrimMultiPointLinks.argtypes = [c_long]
St7BeamOffsetsByCrossSection = _ST7API.St7BeamOffsetsByCrossSection
St7BeamOffsetsByCrossSection.argtypes = [c_long, ctypes.POINTER(c_long)]
St7AlignBeamAxesToUCS = _ST7API.St7AlignBeamAxesToUCS
St7AlignBeamAxesToUCS.argtypes = [c_long, c_long, c_long, c_long, c_long, c_double, c_bool]
St7AlignBeamAxesToFramework = _ST7API.St7AlignBeamAxesToFramework
St7AlignBeamAxesToFramework.argtypes = [c_long, c_long, c_long, c_bool]
St7AlignBeamAxesToPlate = _ST7API.St7AlignBeamAxesToPlate
St7AlignBeamAxesToPlate.argtypes = [c_long, c_long, c_long, c_bool]
St7AlignBeamAxisToVector = _ST7API.St7AlignBeamAxisToVector
St7AlignBeamAxisToVector.argtypes = [c_long, c_long, c_long, c_double, ctypes.POINTER(c_double)]
St7RemoveBeamReferenceNode = _ST7API.St7RemoveBeamReferenceNode
St7RemoveBeamReferenceNode.argtypes = [c_long]
St7PlateOffsetByThickness = _ST7API.St7PlateOffsetByThickness
St7PlateOffsetByThickness.argtypes = [c_long, c_long]
St7AlignPlateAxesToUCS = _ST7API.St7AlignPlateAxesToUCS
St7AlignPlateAxesToUCS.argtypes = [c_long, c_long, c_long, c_long, c_double]
St7AlignPlateNormalByConnection = _ST7API.St7AlignPlateNormalByConnection
St7AlignPlateNormalByConnection.argtypes = [c_long, c_long]
St7AlignPlateRCDirectionsToUCS = _ST7API.St7AlignPlateRCDirectionsToUCS
St7AlignPlateRCDirectionsToUCS.argtypes = [c_long, c_long, c_long, c_long, c_double]
St7AlignFaceNormalByConnection = _ST7API.St7AlignFaceNormalByConnection
St7AlignFaceNormalByConnection.argtypes = [c_long, c_long]
St7AlignBeam3AxisByConnection = _ST7API.St7AlignBeam3AxisByConnection
St7AlignBeam3AxisByConnection.argtypes = [c_long, c_bool]
St7AlignPlateAxesByConnection = _ST7API.St7AlignPlateAxesByConnection
St7AlignPlateAxesByConnection.argtypes = [c_long, c_long, c_double]
St7RotatePlateConnections = _ST7API.St7RotatePlateConnections
St7RotatePlateConnections.argtypes = [c_long, c_bool]
St7FlipEntity = _ST7API.St7FlipEntity
St7FlipEntity.argtypes = [c_long]
St7InvertPathNormal = _ST7API.St7InvertPathNormal
St7InvertPathNormal.argtypes = [c_long]
St7InsituStress = _ST7API.St7InsituStress
St7InsituStress.argtypes = [c_long, c_long, c_long, ctypes.POINTER(c_long), ctypes.POINTER(c_long), ctypes.POINTER(c_long)]
St7GetGlobalIntegerValue = _ST7API.St7GetGlobalIntegerValue
St7GetGlobalIntegerValue.argtypes = [c_long, ctypes.POINTER(c_long)]
St7GetGlobalLogicalValue = _ST7API.St7GetGlobalLogicalValue
St7GetGlobalLogicalValue.argtypes = [c_long, ctypes.POINTER(c_bool)]
St7GetGlobalStringValue = _ST7API.St7GetGlobalStringValue
St7GetGlobalStringValue.argtypes = [c_long, c_char_p, c_long]
St7ClearGlobalIntegerValues = _ST7API.St7ClearGlobalIntegerValues
St7ClearGlobalIntegerValues.argtypes = []
St7ClearGlobalLogicalValues = _ST7API.St7ClearGlobalLogicalValues
St7ClearGlobalLogicalValues.argtypes = []
St7ClearGlobalStringValues = _ST7API.St7ClearGlobalStringValues
St7ClearGlobalStringValues.argtypes = []
St7RGBToColour = _ST7API.St7RGBToColour
St7RGBToColour.argtypes = [c_double, c_double, c_double, ctypes.POINTER(c_long)]
St7ColourToRGB = _ST7API.St7ColourToRGB
St7ColourToRGB.argtypes = [c_long, ctypes.POINTER(c_double), ctypes.POINTER(c_double), ctypes.POINTER(c_double)]
