import unittest

from unitree_cleanup import close_rpc_client


class Channel:
    def __init__(self):
        self.closed = False

    def CloseReader(self):
        self.closed = True

    def CloseWriter(self):
        self.closed = True


class Stub:
    def __init__(self):
        self._ClientStub__recvChannel = Channel()
        self._ClientStub__sendChannel = Channel()


class Client:
    def __init__(self):
        self._ClientBase__stub = Stub()


class UnitreeCleanupTests(unittest.TestCase):
    def test_closes_rpc_reader_and_writer(self):
        client = Client()
        stub = client._ClientBase__stub
        close_rpc_client(client)
        self.assertTrue(stub._ClientStub__recvChannel.closed)
        self.assertTrue(stub._ClientStub__sendChannel.closed)

    def test_none_and_unknown_clients_are_safe(self):
        close_rpc_client(None)
        close_rpc_client(object())


if __name__ == "__main__":
    unittest.main()
