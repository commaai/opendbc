def gwm_checksum(address: int, sig, d: bytearray) -> int:
    """
    Calcula CRC-8 mantendo a interface original com address e sig
    mas usando o algoritmo CRC-8 encontrado pelo crcbeagle
    """
    # Parâmetros do CRC-8 encontrados
    poly = 0x1D
    crc = 0x00  # init value
    xor_out = 0x2D

    # Calcula CRC-8
    for byte in d:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc = (crc << 1)
            crc &= 0xFF  # manter em 8 bits

    return crc ^ xor_out

# Lista de mensagens
messages = [
    b'\xed\x22\xb6\x00\x00\x00\x00\x00',
    b'\xf0\x22\xb6\x00\x00\x00\x00\x01',
    b'\xd7\x22\xb6\x00\x00\x00\x00\x02',
    b'\xca\x22\xb6\x00\x00\x00\x00\x03',
    b'\x99\x22\xb6\x00\x00\x00\x00\x04',
    b'\x84\x22\xb6\x00\x00\x00\x00\x05',
    b'\xa3\x22\xb6\x00\x00\x00\x00\x06',
    b'\x05\x22\xb6\x00\x00\x00\x00\x08',
    b'\x99\x22\xb2\x00\x27\x00\x00\x0b',
    b'\xb1\x22\xb0\x00\x3b\x00\x00\x0c',
    b'\x9a\x22\xac\x00\x63\x00\x00\x0d',
]

# Para usar a função, precisamos criar um objeto 'sig' mock
# Vamos assumir alguns valores comuns para teste
class MockSig:
    def __init__(self, start_bit=0):
        self.start_bit = start_bit

# Teste com diferentes valores de address e start_bit
# addresses_to_test = [0x452, 0x38D, 0x42D, 0x123]  # incluindo um não mapeado
# start_bits_to_test = [0, 4, 8, 12]  # diferentes posições
addresses_to_test = [0xA1]  # incluindo um não mapeado
start_bits_to_test = [7]  # diferentes posições

print("Calculando checksums para as mensagens:\n")

for i, msg in enumerate(messages):
    original_checksum = msg[0]  # Primeiro byte é o checksum original
    data_without_checksum = msg[1:]  # Restante dos dados

    print(f"Mensagem {i}: {msg.hex()}")
    print(f"Checksum original: 0x{original_checksum:02X} ({original_checksum})")

    # Testar com diferentes parâmetros
    for address in addresses_to_test:
        for start_bit in start_bits_to_test:
            sig = MockSig(start_bit)

            # Converter para bytearray (cópia para não modificar o original)
            data_copy = bytearray(data_without_checksum)

            try:
                calculated_checksum = gwm_checksum(address, sig, data_copy)
                match = "✓" if calculated_checksum == (original_checksum & 0xFF) else "✗"
                print(f"  Address: 0x{address:02X}, Start_bit: {start_bit:2d} -> Calculado: 0x{calculated_checksum:02X} {match}")
            except Exception as e:
                print(f"  Address: 0x{address:02X}, Start_bit: {start_bit:2d} -> Erro: {e}")

    print()
