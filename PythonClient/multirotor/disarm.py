import setup_path
import hercules as airsim

client = airsim.MultirotorClient()
client.armDisarm(False)
