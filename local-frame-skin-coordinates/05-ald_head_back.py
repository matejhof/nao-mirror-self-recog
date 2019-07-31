# Local frame
localFrameOrigin = App.Vector(
  App.ActiveDocument.ald_head_back001.Shape.getElement("Vertex1").Point.x,
  App.ActiveDocument.ald_head_back001.Shape.getElement("Vertex1").Point.y,
  App.ActiveDocument.ald_head_back001.Shape.getElement("Vertex4").Point.z,
)

# Local axes
localXAxisEdge = App.ActiveDocument.ald_head_back001.Shape.getElement("Edge13")
localAxisX = (localXAxisEdge.Vertexes[0].Point - localXAxisEdge.Vertexes[1].Point).normalize()
localYAxisEdge = App.ActiveDocument.ald_head_back001.Shape.getElement("Edge2")
localAxisY = (localYAxisEdge.Vertexes[0].Point - localYAxisEdge.Vertexes[1].Point).normalize()
localAxisZ = App.Vector(0,0,1)

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
