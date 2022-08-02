import torch
from torch_geometric.data import Data
import torch.nn.functional as F

class SimpleMLP(torch.nn.Module):
    def __init__(self, input_size: int = 64, hidden_sizes: [int] = [255], reconstruction: bool = False):
        super(SimpleMLP, self).__init__()
        layers = []
        if reconstruction:
            hidden_sizes[-1] = hidden_sizes[-1] + input_size
        
        for hs in hidden_sizes[:1]:
            layers.append(torch.nn.Linear(input_size, hs))
            layers.append(torch.nn.ReLU())
            input_size = hs
        layers.append(torch.nn.Linear(input_size, hidden_sizes[-1]))

        self.layers = torch.nn.Sequential(*layers)
        self.output_features = hidden_sizes[-1]

    def forward(self, data: Data) -> torch.Tensor:
        count = torch.unique(data.batch, return_counts=True)[1]
        assert (count != count[0]).sum() == 0
        x = data.x
        # Checked data is correctly memory aligned and can be reshaped
        # If you change something in the dataloader make sure this is still working
        x = x.reshape((data.batch.max().item()+1,-1,x.shape[1]))
        x = self.layers(x)
        x[:,:,0] = F.sigmoid(x[:,:,0])
        return x.reshape( (-1, self.output_features))