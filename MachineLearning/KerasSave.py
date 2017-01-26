from keras.models import Sequential, model_from_json

def SaveModel(model, filename):
    model_json = model.to_json()
    with open(filename+".json", 'w') as json_file:
        json_file.write(model_json)

    model.save_weights(filename+".h5")
    print "Save complete."

def LoadModel(model, filename):
    json_file = open(filename+".json", "r")
    read_model = json_file.read()
    json_file.close()
    model = model_from_json(read_model)
    model.Load_weights(filename+".h5")
    print "Loaded model."
    return model
