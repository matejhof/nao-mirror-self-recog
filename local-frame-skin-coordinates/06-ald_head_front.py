# Points of interest: ['Position vertex', 'Normal ending vertex']
vertexesPositionsNormals = [
  ['Vertex7', 'Vertex42'], ['Vertex13', 'Vertex40'], ['Vertex8', 'Vertex38'], ['Vertex12', 'Vertex36'], ['Vertex10', 'Vertex34'],
  ['Vertex5', 'Vertex44'], ['Vertex6', 'Vertex46'], ['Vertex4', 'Vertex48'], ['Vertex9', 'Vertex50'], ['Vertex3', 'Vertex52'], ['Vertex11', 'Vertex54'], ['Vertex2', 'Vertex32'],
  #['Vertex', 'Vertex'], ['Vertex', 'Vertex'], ['Vertex', 'Vertex'], ['Vertex', 'Vertex'], ['Vertex', 'Vertex'], ['Vertex', 'Vertex'], ['Vertex', 'Vertex'],
  #['Vertex', 'Vertex'], ['Vertex', 'Vertex'], ['Vertex', 'Vertex'], ['Vertex', 'Vertex'], ['Vertex', 'Vertex']
]
positionVertexContainer = App.ActiveDocument.ald_head_front001
normalVertexContainer = App.ActiveDocument.ald_head_front

processPoints(transformationMatrix, vertexesPositionsNormals, positionVertexContainer, normalVertexContainer)
