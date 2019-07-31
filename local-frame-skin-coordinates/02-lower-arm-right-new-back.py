# Points of interest: ['Position vertex', 'Normal ending vertex']
vertexesPositionsNormals = [['Vertex1', 'Vertex32'], ['Vertex2', 'Vertex46'], ['Vertex3', 'Vertex34'], ['Vertex4', 'Vertex36'], ['Vertex5', 'Vertex44'], ['Vertex6', 'Vertex42'], ['Vertex8', 'Vertex38'], ['Vertex7', 'Vertex40']]
positionVertexContainer = App.ActiveDocument.lower_arm_right_new_back001
normalVertexContainer = App.ActiveDocument.lower_arm_right_new_back

processPoints(transformationMatrix, vertexesPositionsNormals, positionVertexContainer, normalVertexContainer)
