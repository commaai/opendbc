import cantools

class FordTronParser:
    def __init__(self, dbc_path):
        self.db = cantools.database.load_file(dbc_path)

    def decode_steering(self, data: bytes):
        return self.db.decode_message('STEERING_DATA', data)

    def decode_longitudinal(self, data: bytes):
        return self.db.decode_message('LONGITUDINAL_DATA', data)
