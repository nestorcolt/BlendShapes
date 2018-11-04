from contextlib import contextmanager
import maya.api.OpenMaya as om
import time
import math

###################################################################################################
# Globals


@contextmanager
def benchmark():
    # Measures code execution time
    start = time.time()
    yield
    end = time.time()
    print("Code execution time: {}".format(end - start))

###################################################################################################


class MirrorBlendshapes(object):
    """Object that gets a base shape as reference to fip a target mesh from + to - or - to + in X axis"""
    @staticmethod
    def getDag(stringName):
        MSelList = om.MSelectionList()
        MSelList.add(stringName)
        MDag = MSelList.getDagPath(0)
        return MDag

    @staticmethod
    def distanceBetweenTwoPoints(a=[0, 0, 0], b=[0, 0, 0]):
        distanceX = a[0] - b[0]
        distanceY = a[1] - b[1]
        distanceZ = a[2] - b[2]
        distance = math.sqrt((distanceX * distanceX) + (distanceY * distanceY) + (distanceZ * distanceZ))
        return distance

    def __init__(self, baseMesh):
        super(MirrorBlendshapes, self).__init__()
        print("Init : %s" % self.__class__)

        #
        self.TargetString = ''
        self.MBaseDag = self.getDag(baseMesh)
        self.MTargetDag = ""
        self.vertexCount = 0
        #
        self.currentNonZeroArray = []
        self.basePositionArray = {}
        self.targetPositionArray = {}

    ###################################################################################################
    # Gather the data from new target mesh
    #
    def gatherMeshData(self, targetMesh):
        """
            @Param targetMesh: string - target mesh
        """
        self.getTargetMesh(targetMesh)
        self.currentNonZeroArray = self.getNonZeroVertex()

    ###################################################################################################
    # Get dag path from string in the target mesh
    #
    def getTargetMesh(self, targetMesh):
        """
            @Param targetMesh: String - Target mesh
        """
        self.TargetString = targetMesh
        self.MTargetDag = self.getDag(targetMesh)

    ###################################################################################################
    # Get vertex coordenates given a dag and vertex ID
    #

    def getVertexCoords(self, MMeshDag, vertexIndex):
        """
            Comment: get vertex coordenates given a dag and vertex ID
            @Param MMeshDag: dagPath - TargetMesh
            @Param vertexIndex: INT - Vertex Index
            @Return vtxPos: vector - Point coordenates
        """
        singleIndexComponent = om.MFnSingleIndexedComponent()
        vertexComponent = singleIndexComponent.create(om.MFn.kMeshVertComponent)
        singleIndexComponent.addElement(vertexIndex)

        # Init Mesh vertex C++ class with MObjectDag, and mesh component (vertex)
        mItVtx = om.MItMeshVertex(MMeshDag, vertexComponent)
        point = mItVtx.position(om.MSpace.kObject)
        vtxPos = [round(point.x, 3), round(point.y, 3), round(point.z, 3)]

        return vtxPos

    ###################################################################################################
    # Sets the vertex coordenates given vector
    #

    def setVertexCoords(self, MMeshDag, vertexIndex, vector):
        """
            Comment: set mesh component new coordenates
            @Param MMeshDag: dagPath - TargetMesh
            @Param vertexIndex: INT - Vertex Index
            @Param vector: vector - coordenates to set
        """

        singleIndexComponent = om.MFnSingleIndexedComponent()
        vertexComponent = singleIndexComponent.create(om.MFn.kMeshVertComponent)
        singleIndexComponent.addElement(vertexIndex)

        # Init Mesh vertex C++ class with MObjectDag, and mesh component (vertex)
        mItVtx = om.MItMeshVertex(MMeshDag, vertexComponent)
        point = mItVtx.position(om.MSpace.kObject)

        MVectorDouble = om.MVector(vector[0], vector[1], vector[2])
        MPoint = om.MPoint(MVectorDouble)
        mItVtx.setPosition(MPoint, om.MSpace.kObject)

    ###################################################################################################
    # Get the non-zero vertex - (vertex with transforms != 0.000)
    #

    def getNonZeroVertex(self):
        """
            Description: Get the non-zero vertex - (vertex with transforms != 0.000)
            @return: Non zero vertex array
        """

        MMeshVtx = om.MItMeshVertex(self.MBaseDag)
        self.vertexCount = MMeshVtx.count()
        vertexArray = []
        counter = 0

        while not MMeshVtx.isDone():
            TBase = self.getVertexCoords(self.MBaseDag, counter)
            TTarget = self.getVertexCoords(self.MTargetDag, counter)

            # stored target data into property dictionary
            self.targetPositionArray[counter] = TTarget
            self.basePositionArray[counter] = TBase

            if TBase != TTarget:
                vertexArray.append(counter)

            counter += 1
            MMeshVtx.next()

        return vertexArray

    ###################################################################################################
    # returns the closest vertex given a mesh and a position [x,y,z] in world space.
    # Uses om.MfnMesh.getClosestPoint() returned face ID and iterates through face's vertices

    def getClosestVertex(self, baseDag, position=[0, 0, 0]):
        """
            Comment: get closest vertex on given point.
            @Param baseDag: dagPath -  dag path of mesh object
            @Param Position: Vector - coordenates to find the closest vertex
            @Return closestVertex: Vertex ID
        """

        MVector = om.MVector(position)  # using MVector type to represent position
        MMesh = om.MFnMesh(baseDag)
        ID = MMesh.getClosestPoint(om.MPoint(MVector), om.MSpace.kObject)[1]  # getting closest face ID
        #
        MItMPoly = om.MItMeshPolygon(baseDag)
        MItMPoly.setIndex(ID)
        points = MItMPoly.getPoints(om.MSpace.kObject)
        vertex = MItMPoly.getVertices()
        #
        vtxDistance = {self.distanceBetweenTwoPoints(position, pos): vtx for vtx, pos in zip(vertex, points)}
        #
        closestVertex = vtxDistance[min(vtxDistance.keys())]

        return closestVertex

    ###################################################################################################
    # Flip vertex from one side to the other in a mirrored geometry
    #

    def flipVertex(self, vertexArray):
        """
            Description: flip vertex from one side to the other in a mirrored geometry
            @Param vertexArray: Array - Non zero vertex array
        """

        for vtxIndex in vertexArray:
            negativePos = [self.basePositionArray[vtxIndex][0] * -1, self.basePositionArray[vtxIndex][1], self.basePositionArray[vtxIndex][2]]
            point = om.MPoint(negativePos[0], negativePos[1], negativePos[2])
            targetVertex = self.getClosestVertex(self.MBaseDag, point)

            if targetVertex is None:
                continue

            src_vtx_pos = self.targetPositionArray[vtxIndex]
            trg_vtx_pos = self.targetPositionArray[targetVertex]

            self.setVertexCoords(self.MTargetDag, targetVertex, (src_vtx_pos[0] * -1, src_vtx_pos[1], src_vtx_pos[2]))
            self.setVertexCoords(self.MTargetDag, vtxIndex, (trg_vtx_pos[0] * -1, trg_vtx_pos[1], trg_vtx_pos[2]))


###################################################################################################
#
if __name__ == '__main__':
    baseMesh = "base"
    targetMesh = "target"
    #
    with benchmark():
        MBsInstance = MirrorBlendshapes(baseMesh)
        MBsInstance.gatherMeshData(targetMesh)
        # Execute
        MBsInstance.flipVertex(MBsInstance.currentNonZeroArray)
