from Network import Network
import Functions

#network = Network.load("model.json", "model.h5")
network = Network.new()

network.train("training_data.csv", epoch=1, batch=3)


network.save("model/model.json", "model/weights_gen0.h5")
