import matlab.engine
eng = matlab.engine.start_matlab()
fis = eng.readfis('fuzzycontroller.fis')
input = matlab.double([[5.0,5.0]])
output = eng.evalfis(fis, input)
print("output = ", output)
eng.quit()
