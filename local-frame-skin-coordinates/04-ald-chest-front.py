# Unity transformation matrix: torso taxels are drawn in correct frame
transformationMatrix = [
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
]

# Points of interest: ['Position vertex', 'Normal ending vertex']
vertexesPositionsNormals = [
  ['Vertex27', 'Vertex168'], ['Vertex22', 'Vertex144'], ['Vertex23', 'Vertex146'], ['Vertex26', 'Vertex166'],
  ['Vertex28', 'Vertex170'], ['Vertex19', 'Vertex140'], ['Vertex20', 'Vertex142'], ['Vertex21', 'Vertex148'], ['Vertex25', 'Vertex164'],
  ['Vertex29', 'Vertex172'], ['Vertex18', 'Vertex136'], ['Vertex17', 'Vertex138'], ['Vertex16', 'Vertex150'], ['Vertex24', 'Vertex162'],
  ['Vertex12', 'Vertex132'], ['Vertex13', 'Vertex134'], ['Vertex14', 'Vertex154'], ['Vertex15', 'Vertex152'], 
  ['Vertex11', 'Vertex130'], ['Vertex10', 'Vertex128'], ['Vertex9', 'Vertex156'], ['Vertex8', 'Vertex158'],
  ['Vertex5', 'Vertex124'], ['Vertex6', 'Vertex126'], ['Vertex7', 'Vertex160']
]
positionVertexContainer = App.ActiveDocument.ald_chest_front001
normalVertexContainer = App.ActiveDocument.ald_chest_front

processPoints(transformationMatrix, vertexesPositionsNormals, positionVertexContainer, normalVertexContainer)
