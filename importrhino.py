import St7API
import ctypes
import os
import sys
import tkinter as tk
from tkinter import filedialog

#push test

def ChkErr(ErrorCode):
    ErrorOccured = (ErrorCode != 0)
    if ErrorOccured:
        ErrorString = ctypes.create_string_buffer(St7API.kMaxStrLen)

        # Attempt to get API error string
        iErr = St7API.St7GetAPIErrorString(ErrorCode, ErrorString, St7API.kMaxStrLen)

        # If that failed, attempt to retrive a solver error string
        if iErr:
            iErr = St7API.St7GetSolverErrorString(ErrorCode, ErrorString, St7API.kMaxStrLen)

        if not iErr:
            raise Exception('%s (%d)' % (ErrorString.value, ErrorCode))
        else:
            raise Exception('An unknown error occured (%d)' % ErrorCode)

    return ErrorOccured


root = tk.Tk()
root.withdraw()

rhinoProfileFilePath = filedialog.askopenfilename(title="Select .3dm file containing surface definition of cross section:")
rhinoProfileFile = rhinoProfileFilePath.encode()
head,tail=os.path.split(rhinoProfileFilePath)
profilename = tail.split('.')[0]

SectionFolderPath = head + '/' + profilename
ScratchFolder = head + '/' + profilename + '/' + "Scratch"

if not os.path.exists(SectionFolderPath):
    os.makedirs(SectionFolderPath)

if not os.path.exists(ScratchFolder):
    os.makedirs(ScratchFolder)

try:

    # Set up folders.
    inFile = (SectionFolderPath +'/'+profilename+".st7").replace('/','\\').encode()
    scratch = (ScratchFolder).replace('/','\\').encode()
    BXSName = (SectionFolderPath+'/'+profilename+".bxs").encode()

    # Initialize new file.
    ChkErr(St7API.St7Init())
    ChkErr(St7API.St7NewFile(1, inFile, scratch))

    # Input required from the user.
    extrusionlength = int(input("Input the length of the extrusion (in inches):"))

    # Perform Rhino Import.
    options = (ctypes.c_long * 5)()
    options[St7API.ipGeomImportProperty] = 0
    options[St7API.ipGeomImportCurvesToBeams] = St7API.btFalse
    options[St7API.ipGeomImportGroupsAs] = St7API.ggNone
    options[St7API.ipGeomImportColourAsProperty] = St7API.btTrue
    options[St7API.ipGeomImportLengthUnit] = St7API.luGeomInch

    tolerances = (ctypes.c_double * 1)()
    tolerances[St7API.ipGeomImportTol] = 0.01

    ChkErr(St7API.St7ImportRhino(1, rhinoProfileFile, options, tolerances, 0))

    # Automesh surface.
    St7SurfaceMeshIntegers = (ctypes.c_long * 11)()
    St7SurfaceMeshIntegers[St7API.ipSurfaceMeshMode] = St7API.mmAuto
    St7SurfaceMeshIntegers[St7API.ipSurfaceMeshSizeMode] = St7API.smPercentage
    St7SurfaceMeshIntegers[St7API.ipSurfaceMeshTargetNodes] = 4
    St7SurfaceMeshIntegers[St7API.ipSurfaceMeshTargetPropertyID] = -1
    St7SurfaceMeshIntegers[St7API.ipSurfaceMeshAutoCreateProperties] = St7API.btTrue
    St7SurfaceMeshIntegers[St7API.ipSurfaceMeshMinEdgesPerCircle] = 12
    St7SurfaceMeshIntegers[St7API.ipSurfaceMeshApplyTransitioning] = St7API.btTrue
    St7SurfaceMeshIntegers[St7API.ipSurfaceMeshApplySurfaceCurvature] = St7API.btTrue
    St7SurfaceMeshIntegers[St7API.ipSurfaceMeshAllowUserStop] = St7API.btFalse
    St7SurfaceMeshIntegers[St7API.ipSurfaceMeshConsiderNearVertex] = St7API.btTrue
    St7SurfaceMeshIntegers[St7API.ipSurfaceMeshSelectedFaces] = St7API.btFalse

    St7SurfaceMeshDoubles = (ctypes.c_double * 4)()
    St7SurfaceMeshDoubles[St7API.ipSurfaceMeshSize] = 3
    St7SurfaceMeshDoubles[St7API.ipSurfaceMeshLengthRatio] = 0.1
    St7SurfaceMeshDoubles[St7API.ipSurfaceMeshMaximumIncrease] = 40
    St7SurfaceMeshDoubles[St7API.ipSurfaceMeshOnEdgesLongerThan] = 0.0

    ChkErr(St7API.St7SurfaceMesh(1, St7SurfaceMeshIntegers, St7SurfaceMeshDoubles, 0))

    # Generate BXS section.
    BXSprop = (ctypes.c_double * 39)()

    ChkErr(St7API.St7GenerateBXS(1, BXSName, BXSprop))

    Sxxtop = BXSprop[St7API.ipBXSZxxPlus]
    Sxxbot = BXSprop[St7API.ipBXSZxxMinus]
    Syyleft = BXSprop[St7API.ipBXSZyyMinus]
    Syyright = BXSprop[St7API.ipBXSZyyPlus]

    # Extrude 2D plates into 3D solids.
    St7API.St7SetAllEntitySelectState(1, St7API.tyPLATE, True)

    DXYZ = (ctypes.c_double * 3)()
    DXYZ[0] = 0
    DXYZ[1] = 0
    DXYZ[2] = 1

    ChkErr(St7API.St7ExtrudeByIncrement(1, DXYZ, 1, extrusionlength))

    # Output total number of nodes in model prior to creation of additional master elements at ends of extrusion.
    totalnodes = (ctypes.c_long * 1)()
    St7API.St7GetTotal(1, St7API.tyNODE, totalnodes)

    # Deselect all plate elements.
    St7API.St7SetAllEntitySelectState(1, St7API.tyPLATE, False)

    # Get nodes at z=0, ith member end and create rigid link cluster.
    pointcoordinatesarray = ctypes.c_double * 3
    pointcoordinates = pointcoordinatesarray()

    for node in range(1, totalnodes[0]+1):
        St7API.St7GetNodeXYZ(1, node, pointcoordinates)
        if round(pointcoordinates[2], 5) == 0:
            St7API.St7SetEntitySelectState(1, St7API.tyNODE, node, 0, True)

    St7API.St7CreateRigidLinkCluster(1, 1, St7API.rlPlaneXYZ, 0)

    # Get nodes at z=extrusionlength, jth member end and create rigid link cluster.

    for node in range(1, totalnodes[0]+1):
        St7API.St7GetNodeXYZ(1, node, pointcoordinates)
        if round(pointcoordinates[2], 5) == extrusionlength:
            St7API.St7SetEntitySelectState(1, St7API.tyNODE, node, 0, True)

    St7API.St7CreateRigidLinkCluster(1, 1, St7API.rlPlaneXYZ, 0)

    # Set Node Restraints at ith member end
    St7SetNodeRestraint6_Status = (ctypes.c_long * 6)()
    St7SetNodeRestraint6_Status[0] = St7API.btTrue
    St7SetNodeRestraint6_Status[1] = St7API.btTrue
    St7SetNodeRestraint6_Status[2] = St7API.btTrue
    St7SetNodeRestraint6_Status[3] = St7API.btFalse
    St7SetNodeRestraint6_Status[4] = St7API.btTrue
    St7SetNodeRestraint6_Status[5] = St7API.btTrue

    St7SetNodeRestraint6_Doubles = (ctypes.c_double * 6)()

    St7API.St7SetNodeRestraint6(1, totalnodes[0] + 1, 1, 1, St7SetNodeRestraint6_Status, St7SetNodeRestraint6_Doubles)

    # Set Node Restraints at jth member end
    St7SetNodeRestraint6_Status = (ctypes.c_long * 6)()
    St7SetNodeRestraint6_Status[0] = St7API.btTrue
    St7SetNodeRestraint6_Status[1] = St7API.btTrue
    St7SetNodeRestraint6_Status[2] = St7API.btFalse
    St7SetNodeRestraint6_Status[3] = St7API.btFalse
    St7SetNodeRestraint6_Status[4] = St7API.btTrue
    St7SetNodeRestraint6_Status[5] = St7API.btTrue

    St7SetNodeRestraint6_Doubles = (ctypes.c_double * 6)()

    St7API.St7SetNodeRestraint6(1, totalnodes[0] + 2, 1, 1, St7SetNodeRestraint6_Status, St7SetNodeRestraint6_Doubles)

    # Create brick material properties.
    St7API.St7NewBrickProperty(1, 1, St7API.mtIsotropic, r'Aluminum'.encode())

    St7SetBrickIsotropicMaterial_Doubles = (ctypes.c_double * 8)()
    St7SetBrickIsotropicMaterial_Doubles[0] = 10100
    St7SetBrickIsotropicMaterial_Doubles[1] = 0.3
    St7SetBrickIsotropicMaterial_Doubles[2] = 1
    St7SetBrickIsotropicMaterial_Doubles[3] = 1
    St7SetBrickIsotropicMaterial_Doubles[4] = 1
    St7SetBrickIsotropicMaterial_Doubles[5] = 1
    St7SetBrickIsotropicMaterial_Doubles[6] = 1

    St7API.St7SetBrickIsotropicMaterial(1, 1, St7SetBrickIsotropicMaterial_Doubles)

    #Assume 6063-T6
    Fty = 25

    # Apply nodal forces at member end i.
    St7SetNodeMoment3_Doubles = (ctypes.c_double * 3)()
    St7SetNodeMoment3_Doubles[0] = 1000
    St7SetNodeMoment3_Doubles[1] = 0
    St7SetNodeMoment3_Doubles[2] = 0

    St7API.St7SetNodeMoment3(1, totalnodes[0] + 1, 1, St7SetNodeMoment3_Doubles)

    # Apply nodal forces at member end j.
    St7SetNodeMoment3_Doubles = (ctypes.c_double * 3)()
    St7SetNodeMoment3_Doubles[0] = -1000
    St7SetNodeMoment3_Doubles[1] = 0
    St7SetNodeMoment3_Doubles[2] = 0

    St7API.St7SetNodeMoment3(1, totalnodes[0] + 2, 1, St7SetNodeMoment3_Doubles)

    # Assign material to bricks
    St7API.St7SetAllEntitySelectState(1, St7API.tyBRICK, True)

    St7GetTotal_Total = (ctypes.c_long * 1)()
    St7API.St7GetTotal(1, St7API.tyBRICK, St7GetTotal_Total)

    for i in range(1, St7GetTotal_Total[0] + 1):
        St7API.St7SetElementProperty(1, St7API.ptBRICKPROP, i, 1)


    # Manual calculatin of yield moment.
    Mxyield = Fty * min(Sxxtop, Sxxbot)

    print("Profile Name = ", profilename)
    print("Total Bricks = ", St7GetTotal_Total[0])
    print("Total Nodes = ", totalnodes[0])

    print("*** Bending About X (Positive Moment) ***")
    print("M_yield = ", "{:.1f}".format(Mxyield), "kip-in")


    # Prepare Linear Static Solver
    St7API.St7EnableLSALoadCase(1, 1, 1)
    St7API.St7RunSolver(1, St7API.stLinearStatic, St7API.smProgressRun, St7API.btTrue)

    # Linear Buckling Analysis Solver
    St7API.St7SetLBANumModes(1, 2)
    St7API.St7RunSolver(1, St7API.stLinearBuckling, St7API.smProgressRun, St7API.btTrue)


    ResultPath = (SectionFolderPath +'/'+profilename+".LBA").replace('/','\\').encode()


    NumPrimary = (ctypes.c_long * 1)()
    NumSecondary = (ctypes.c_long * 1)()
    St7API.St7OpenResultFile(1,ResultPath,"0".encode(),St7API.kNoCombinations, NumPrimary, NumSecondary)


    Factor = (ctypes.c_double * 1)()
    ChkErr(St7API.St7GetResultCaseFactor(1,1,Factor))
    print("M_crit = ", "{:.1f}".format(Factor[0]), "kip-in")



    ChkErr(St7API.St7SaveFile(1))



finally:
    St7API.St7Release()
