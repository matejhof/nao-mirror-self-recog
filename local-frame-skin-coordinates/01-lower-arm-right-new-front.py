# Local frame
localFrameOrigin = App.ActiveDocument.lower_arm_right_new_front.Shape.getElement("Vertex4").Point

# Local axes
localAxisEdge = App.ActiveDocument.lower_arm_right_new_front.Shape.getElement("Edge1")
localAxisX = (localAxisEdge.Vertexes[1].Point - localAxisEdge.Vertexes[0].Point).normalize()
localAxisY = (App.ActiveDocument.lower_arm_right_new_front002.Shape.getElement("Edge965").Vertexes[1].Point - App.ActiveDocument.lower_arm_right_new_front002.Shape.getElement("Edge940").Vertexes[0].Point).normalize()
localAxisZ = localAxisX.cross(localAxisY)

# Transformation matrix
transformationMatrix = np.matmul(np.linalg.inv([
  [localAxisX[0], localAxisY[0], localAxisZ[0], 0],
  [localAxisX[1], localAxisY[1], localAxisZ[1], 0],
  [localAxisX[2], localAxisY[2], localAxisZ[2], 0],
  [0, 0, 0, 1]
]), [
  [1, 0, 0, -localFrameOrigin[0]],
  [0, 1, 0, -localFrameOrigin[1]],
  [0, 0, 1, -localFrameOrigin[2]],
  [0, 0, 0, 1]  
])

# Testing
#test = [[localFrameOrigin[0]], [localFrameOrigin[1]], [localFrameOrigin[2]], [1]]
#np.matmul(transformationMatrix, test)

# Points of interest: ['Position vertex', 'Normal ending vertex']
vertexesPositionsNormals = [['Vertex16', 'Vertex118'], ['Vertex9', 'Vertex96'], ['Vertex17', 'Vertex116'], ['Vertex6', 'Vertex94'], ['Vertex5', 'Vertex92'], ['Vertex15', 'Vertex114'], ['Vertex10', 'Vertex98'], ['Vertex2', 'Vertex90'], ['Vertex18', 'Vertex112'], ['Vertex7', 'Vertex100'], ['Vertex4', 'Vertex88'], ['Vertex14', 'Vertex108'], ['Vertex11', 'Vertex102'], ['Vertex1', 'Vertex86'], ['Vertex19', 'Vertex110'], ['Vertex8', 'Vertex106'], ['Vertex3', 'Vertex104'], ['Vertex13', 'Vertex122'], ['Vertex12', 'Vertex120']]
positionVertexContainer = App.ActiveDocument.lower_arm_right_new_front001
normalVertexContainer = App.ActiveDocument.lower_arm_right_new_front

# Output taxel coordinates
processPoints(transformationMatrix, vertexesPositionsNormals, positionVertexContainer, normalVertexContainer)
