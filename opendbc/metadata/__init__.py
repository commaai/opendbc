from opendbc.metadata.tesla.attributes import METADATA as TESLA_METADATA
from opendbc.metadata.rivian.attributes import METADATA as RIVIAN_METADATA

METADATA = {
  **TESLA_METADATA,
  **RIVIAN_METADATA,
}

def get_brand_metadata(platform):
  return METADATA.get(str(platform))