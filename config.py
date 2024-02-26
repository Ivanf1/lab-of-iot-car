class Config:
    server_ip = ""

    def __init__(self):
        pass

    def set_server_ip(self, ip):
        self.server_ip = ip

    def get_server_ip(self):
        return self.server_ip
