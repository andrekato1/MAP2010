import torch

class FeedForwardNN(torch.nn.Module):
	def __init__(self, in_dim, out_dim):
		"""
			Initializes the network and set up the layers.
		"""
		super(FeedForwardNN, self).__init__()

		self.layer1 = torch.nn.Linear(in_dim, 64)
		self.layer2 = torch.nn.Linear(64, 64)
		self.layer3 = torch.nn.Linear(64, out_dim)

	def forward(self, obs):
		"""
			Runs a forward pass on the neural network.
		"""
		if isinstance(obs, np.ndarray):
			obs = torch.tensor(obs, dtype=torch.float)

		activation1 = torch.nn.functional.relu(self.layer1(obs))
		activation2 = torch.nn.functional.relu(self.layer2(activation1))
		output = self.layer3(activation2)

		return output