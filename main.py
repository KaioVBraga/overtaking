from torcs_client import TorcsClient

if __name__ == '__main__':
    # Create a client and start the driving loop
    client = TorcsClient()
    client.connect()
    client.drive_loop()
