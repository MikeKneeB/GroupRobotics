from keras.models import Sequential, load_model 

def SaveModel(model, filename):
    model.save(filename+'.h5')
    print "Save complete."

def LoadModel(filename):
    return load_model(filename+'.h5')
