from unittest.mock import Mock
from opendbc.car.fw_query_definitions import (
  p16, StdQueries, STANDARD_VIN_ADDRS, ESSENTIAL_ECUS, ECU_NAME,
  FwQueryConfig, Request, AddrType, EcuAddrBusType, EcuAddrSubAddr
)
from opendbc.car import uds
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu


class TestFwQueryDefinitions:
  def test_p16_function(self):
    """Test p16 packs 16-bit values correctly"""
    # Test basic values
    assert p16(0x1234) == b'\x12\x34'
    assert p16(0x0000) == b'\x00\x00'
    assert p16(0xFFFF) == b'\xFF\xFF'
    
    # Test that it produces 2 bytes
    assert len(p16(0x1234)) == 2
    assert len(p16(0)) == 2

  def test_standard_vin_addrs_defined(self):
    """Test STANDARD_VIN_ADDRS contains expected addresses"""
    expected_addrs = [0x7e0, 0x7e2, 0x760, 0x7c6, 0x18da10f1, 0x18da0ef1]
    assert STANDARD_VIN_ADDRS == expected_addrs
    
    # Should be a list of integers
    for addr in STANDARD_VIN_ADDRS:
      assert isinstance(addr, int)
      assert addr > 0

  def test_essential_ecus_defined(self):
    """Test ESSENTIAL_ECUS contains expected ECU types"""
    expected_ecus = [Ecu.engine, Ecu.eps, Ecu.abs, Ecu.fwdRadar, Ecu.fwdCamera, Ecu.vsa]
    assert ESSENTIAL_ECUS == expected_ecus
    
    # Should all be ECU enum values
    for ecu in ESSENTIAL_ECUS:
      assert isinstance(ecu, int)  # Enum values are ints

  def test_ecu_name_mapping(self):
    """Test ECU_NAME provides reverse mapping"""
    # Should be a dict mapping ECU values to names
    assert isinstance(ECU_NAME, dict)
    assert len(ECU_NAME) > 0
    
    # Test some known ECU mappings exist
    for ecu in ESSENTIAL_ECUS:
      assert ecu in ECU_NAME
      assert isinstance(ECU_NAME[ecu], str)

  def test_std_queries_tester_present(self):
    """Test StdQueries tester present requests/responses"""
    # Test tester present request
    expected_request = bytes([uds.SERVICE_TYPE.TESTER_PRESENT, 0x0])
    assert StdQueries.TESTER_PRESENT_REQUEST == expected_request
    
    # Test tester present response
    expected_response = bytes([uds.SERVICE_TYPE.TESTER_PRESENT + 0x40, 0x0])
    assert StdQueries.TESTER_PRESENT_RESPONSE == expected_response
    
    # Test short versions
    short_request = bytes([uds.SERVICE_TYPE.TESTER_PRESENT])
    assert StdQueries.SHORT_TESTER_PRESENT_REQUEST == short_request
    
    short_response = bytes([uds.SERVICE_TYPE.TESTER_PRESENT + 0x40])
    assert StdQueries.SHORT_TESTER_PRESENT_RESPONSE == short_response

  def test_std_queries_diagnostic_session(self):
    """Test StdQueries diagnostic session requests/responses"""
    # Test default diagnostic request
    expected_default_req = bytes([
      uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL,
      uds.SESSION_TYPE.DEFAULT
    ])
    assert StdQueries.DEFAULT_DIAGNOSTIC_REQUEST == expected_default_req
    
    # Test extended diagnostic request
    expected_extended_req = bytes([
      uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL,
      uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC
    ])
    assert StdQueries.EXTENDED_DIAGNOSTIC_REQUEST == expected_extended_req
    
    # Responses should have service type + 0x40
    assert StdQueries.DEFAULT_DIAGNOSTIC_RESPONSE[0] == uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40
    assert StdQueries.EXTENDED_DIAGNOSTIC_RESPONSE[0] == uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40

  def test_std_queries_manufacturer_software_version(self):
    """Test manufacturer software version request"""
    expected_start = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER])
    assert StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST.startswith(expected_start)
    
    # Should include the data identifier
    expected_identifier = p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_NUMBER)
    assert expected_identifier in StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST

  def test_std_queries_supplier_software_version(self):
    """Test supplier software version request/response"""
    expected_req = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
                   p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_VERSION_NUMBER)
    assert StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST == expected_req
    
    expected_resp = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
                    p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_VERSION_NUMBER)
    assert StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE == expected_resp

  def test_std_queries_hardware_number(self):
    """Test ECU hardware number request/response"""
    expected_req = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
                   p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_HARDWARE_NUMBER)
    assert StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_REQUEST == expected_req
    
    expected_resp = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
                    p16(uds.DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_HARDWARE_NUMBER)
    assert StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_RESPONSE == expected_resp

  def test_std_queries_uds_version(self):
    """Test UDS version request/response"""
    expected_req = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
                   p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_SOFTWARE_IDENTIFICATION)
    assert StdQueries.UDS_VERSION_REQUEST == expected_req
    
    expected_resp = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
                    p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_SOFTWARE_IDENTIFICATION)
    assert StdQueries.UDS_VERSION_RESPONSE == expected_resp

  def test_std_queries_obd_version(self):
    """Test OBD version request/response"""
    assert StdQueries.OBD_VERSION_REQUEST == b'\x09\x04'
    assert StdQueries.OBD_VERSION_RESPONSE == b'\x49\x04'

  def test_std_queries_vin_requests(self):
    """Test VIN request/response pairs"""
    # OBD VIN
    assert StdQueries.OBD_VIN_REQUEST == b'\x09\x02'
    assert StdQueries.OBD_VIN_RESPONSE == b'\x49\x02\x01'
    
    # UDS VIN
    expected_uds_req = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + p16(uds.DATA_IDENTIFIER_TYPE.VIN)
    assert StdQueries.UDS_VIN_REQUEST == expected_uds_req
    
    expected_uds_resp = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + p16(uds.DATA_IDENTIFIER_TYPE.VIN)
    assert StdQueries.UDS_VIN_RESPONSE == expected_uds_resp
    
    # GM VIN
    assert StdQueries.GM_VIN_REQUEST == b'\x1a\x90'
    assert StdQueries.GM_VIN_RESPONSE == b'\x5a\x90'
    
    # KWP VIN
    assert StdQueries.KWP_VIN_REQUEST == b'\x21\x81'
    assert StdQueries.KWP_VIN_RESPONSE == b'\x61\x81'

  def test_request_dataclass_basic(self):
    """Test Request dataclass basic functionality"""
    request = Request(
      request=[b'\x22\xF1\x87'],
      response=[b'\x62\xF1\x87'],
      whitelist_ecus=[Ecu.engine, Ecu.abs]
    )
    
    assert request.request == [b'\x22\xF1\x87']
    assert request.response == [b'\x62\xF1\x87']
    assert request.whitelist_ecus == [Ecu.engine, Ecu.abs]
    
    # Test defaults
    assert request.rx_offset == 0x8
    assert request.bus == 1
    assert request.auxiliary == False
    assert request.obd_multiplexing == True

  def test_request_dataclass_custom_values(self):
    """Test Request dataclass with custom values"""
    request = Request(
      request=[b'\x22\xF1\x90'],
      response=[b'\x62\xF1\x90'],
      rx_offset=0x10,
      bus=0,
      auxiliary=True,
      obd_multiplexing=False
    )
    
    assert request.rx_offset == 0x10
    assert request.bus == 0
    assert request.auxiliary == True
    assert request.obd_multiplexing == False

  def test_fw_query_config_basic(self):
    """Test FwQueryConfig basic functionality"""
    request = Request(
      request=[b'\x22\xF1\x87'],
      response=[b'\x62\xF1\x87']
    )
    
    config = FwQueryConfig(requests=[request])
    
    assert config.requests == [request]
    assert config.non_essential_ecus == {}
    assert config.extra_ecus == []
    assert config.match_fw_to_car_fuzzy is None

  def test_fw_query_config_with_extra_ecus(self):
    """Test FwQueryConfig with extra ECUs"""
    request = Request(
      request=[b'\x22\xF1\x87'],
      response=[b'\x62\xF1\x87'],
      whitelist_ecus=[Ecu.engine]
    )
    
    config = FwQueryConfig(
      requests=[request],
      extra_ecus=[(Ecu.engine, 0x7e0, None)]
    )
    
    assert config.extra_ecus == [(Ecu.engine, 0x7e0, None)]

  def test_fw_query_config_auxiliary_request_duplication(self):
    """Test FwQueryConfig duplicates auxiliary requests"""
    request = Request(
      request=[b'\x22\xF1\x87'],
      response=[b'\x62\xF1\x87'],
      bus=0,
      auxiliary=True
    )
    
    config = FwQueryConfig(requests=[request])
    
    # Should have original + duplicated auxiliary request
    assert len(config.requests) == 2
    assert config.requests[0].bus == 0
    assert config.requests[1].bus == 4  # Original bus + 4

  def test_fw_query_config_validation_extra_ecus_no_requests(self):
    """Test FwQueryConfig validation fails when extra_ecus but no requests"""
    try:
      FwQueryConfig(requests=[], extra_ecus=[(Ecu.engine, 0x7e0, None)])
      assert False, "Should have raised assertion error"
    except AssertionError as e:
      assert "Must define a request with extra ecus" in str(e)

  def test_fw_query_config_validation_extra_ecu_not_in_request(self):
    """Test FwQueryConfig validation fails when extra ECU not in any request"""
    request = Request(
      request=[b'\x22\xF1\x87'],
      response=[b'\x62\xF1\x87'],
      whitelist_ecus=[Ecu.abs]  # Only ABS, not engine
    )
    
    try:
      FwQueryConfig(
        requests=[request],
        extra_ecus=[(Ecu.engine, 0x7e0, None)]  # Engine not in whitelist
      )
      assert False, "Should have raised assertion error"
    except AssertionError as e:
      assert "not in any request" in str(e)

  def test_fw_query_config_validation_request_response_length_mismatch(self):
    """Test FwQueryConfig validation fails on mismatched request/response lengths"""
    request = Request(
      request=[b'\x22\xF1\x87', b'\x22\xF1\x90'],  # 2 requests
      response=[b'\x62\xF1\x87']  # 1 response
    )
    
    try:
      FwQueryConfig(requests=[request])
      assert False, "Should have raised assertion error"
    except AssertionError as e:
      assert "Request and response lengths do not match" in str(e)

  def test_fw_query_config_validation_auxiliary_obd_multiplexing(self):
    """Test FwQueryConfig validation fails on auxiliary + OBD multiplexing"""
    request = Request(
      request=[b'\x22\xF1\x87'],
      response=[b'\x62\xF1\x87'],
      bus=1,  # OBD bus
      auxiliary=True,  # Auxiliary panda
      obd_multiplexing=True  # OBD multiplexing
    )
    
    try:
      FwQueryConfig(requests=[request])
      assert False, "Should have raised assertion error"
    except AssertionError as e:
      assert "OBD multiplexed request should not be marked auxiliary" in str(e)

  def test_fw_query_config_validation_unnecessary_non_essential_ecus(self):
    """Test FwQueryConfig validation fails on unnecessary non-essential ECU declarations"""
    request = Request(
      request=[b'\x22\xF1\x87'],
      response=[b'\x62\xF1\x87']
    )
    
    # Try to mark a non-essential ECU as non-essential (unnecessary)
    non_essential_ecu = None
    for ecu_val in range(50):  # Look for an ECU not in ESSENTIAL_ECUS
      if ecu_val not in ESSENTIAL_ECUS:
        non_essential_ecu = ecu_val
        break
    
    if non_essential_ecu is not None:
      try:
        FwQueryConfig(
          requests=[request],
          non_essential_ecus={non_essential_ecu: ["some_model"]}
        )
        assert False, "Should have raised assertion error"
      except AssertionError as e:
        assert "Declaring non-essential ECUs non-essential is not required" in str(e)

  def test_fw_query_config_get_all_ecus_basic(self):
    """Test FwQueryConfig.get_all_ecus basic functionality"""
    request = Request(
      request=[b'\x22\xF1\x87'],
      response=[b'\x62\xF1\x87']
    )
    
    config = FwQueryConfig(requests=[request])
    
    # Mock offline firmware versions
    offline_fw = {
      "BRAND_MODEL1": {
        (Ecu.engine, 0x7e0, None): [b'firmware1'],
        (Ecu.abs, 0x7e1, None): [b'firmware2']
      },
      "BRAND_MODEL2": {
        (Ecu.eps, 0x7e2, None): [b'firmware3']
      }
    }
    
    all_ecus = config.get_all_ecus(offline_fw)
    
    # Should include all ECUs from offline firmware
    expected_ecus = {
      (Ecu.engine, 0x7e0, None),
      (Ecu.abs, 0x7e1, None),
      (Ecu.eps, 0x7e2, None)
    }
    assert all_ecus == expected_ecus

  def test_fw_query_config_get_all_ecus_with_extra_ecus(self):
    """Test FwQueryConfig.get_all_ecus includes extra ECUs"""
    request = Request(
      request=[b'\x22\xF1\x87'],
      response=[b'\x62\xF1\x87'],
      whitelist_ecus=[Ecu.engine, Ecu.fwdCamera]
    )
    
    config = FwQueryConfig(
      requests=[request],
      extra_ecus=[(Ecu.fwdCamera, 0x7e3, None)]
    )
    
    offline_fw = {
      "BRAND_MODEL1": {
        (Ecu.engine, 0x7e0, None): [b'firmware1']
      }
    }
    
    all_ecus = config.get_all_ecus(offline_fw, include_extra_ecus=True)
    
    # Should include both offline and extra ECUs
    expected_ecus = {
      (Ecu.engine, 0x7e0, None),
      (Ecu.fwdCamera, 0x7e3, None)
    }
    assert all_ecus == expected_ecus

  def test_fw_query_config_get_all_ecus_exclude_extra(self):
    """Test FwQueryConfig.get_all_ecus excludes extra ECUs when requested"""
    request = Request(
      request=[b'\x22\xF1\x87'],
      response=[b'\x62\xF1\x87'],
      whitelist_ecus=[Ecu.engine, Ecu.fwdCamera]
    )
    
    config = FwQueryConfig(
      requests=[request],
      extra_ecus=[(Ecu.fwdCamera, 0x7e3, None)]
    )
    
    offline_fw = {
      "BRAND_MODEL1": {
        (Ecu.engine, 0x7e0, None): [b'firmware1']
      }
    }
    
    all_ecus = config.get_all_ecus(offline_fw, include_extra_ecus=False)
    
    # Should only include offline ECUs, not extra
    expected_ecus = {
      (Ecu.engine, 0x7e0, None)
    }
    assert all_ecus == expected_ecus

  def test_type_aliases(self):
    """Test type aliases are properly defined"""
    # Test AddrType
    addr: AddrType = (0x7e0, None)
    assert addr[0] == 0x7e0
    assert addr[1] is None
    
    addr_with_subaddr: AddrType = (0x7e0, 0x10)
    assert addr_with_subaddr[1] == 0x10
    
    # Test EcuAddrBusType
    ecu_addr_bus: EcuAddrBusType = (0x7e0, None, 0)
    assert len(ecu_addr_bus) == 3
    
    # Test EcuAddrSubAddr
    ecu_addr_sub: EcuAddrSubAddr = (Ecu.engine, 0x7e0, None)
    assert ecu_addr_sub[0] == Ecu.engine