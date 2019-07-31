import numpy as np

def processPoints(transformationMatrix, vertexesPositionsNormals, positionVertexContainer, normalVertexContainer):
    for item in vertexesPositionsNormals:
        p = positionVertexContainer.Shape.getElement(item[0]).Point
        e = normalVertexContainer.Shape.getElement(item[1]).Point
        center = np.matmul(transformationMatrix, [[p[0]], [p[1]], [p[2]], [1]])
        normalEnd = np.matmul(transformationMatrix, [[e[0]], [e[1]], [e[2]], [1]])
        normal = normalEnd - center
        normal /= np.linalg.norm(normal)
        center /= 1000
        print(str(center[0][0]) + " " + str(center[1][0]) + " " + str(center[2][0]) + " " + str(normal[0][0]) + " " + str(normal[1][0]) + " " + str(normal[2][0]))
