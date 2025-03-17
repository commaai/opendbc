import pandas as pd
from opendbc.can.packer import CANPacker
from opendbc.car.psa.psacan import calculate_checksum

# DBC and message settings for 3F2
DBC_NAME = 'AEE2010_R3'
MSG_NAME = 'LANE_KEEP_ASSIST'
CHECKSUM_FIELD = 'CHECKSUM'
CHECKSUM_INIT = 0xB
BUS = 2


def recalculate_and_compare_checksums(csv_file: str, save_file: str = None):
    # Load the CSV data
    df = pd.read_csv(csv_file)

    # Init CANPacker for PSA LKA (AEE2010_R3)
    packer = CANPacker(DBC_NAME)

    # Statistics counters
    matches = 0
    mismatches = 0

    for idx, row in df.iterrows():
        # Build the dictionary of DBC values from the row
        values = {
            'DRIVE': row['DRIVE'],
            'COUNTER': row['COUNTER'],
            CHECKSUM_FIELD: 0,  # Zero checksum for calculation
            'STATUS': row['STATUS'],
            'LXA_ACTIVATION': row['LXA_ACTIVATION'],
            'TORQUE_FACTOR': row['TORQUE_FACTOR'],
            'SET_ANGLE': row['SET_ANGLE'],
            # Optional unknown fields if mapped in DBC
            'unknown2': row['unknown2'],
            'unknown4': row['unknown4'],
        }

        # Pack the message to get raw bytes for checksum calculation
        msg_bytes = packer.make_can_msg(MSG_NAME, BUS, values)[1]

        # Calculate checksum
        calculated_checksum = calculate_checksum(msg_bytes, CHECKSUM_INIT)

        # Compare recalculated checksum with recorded one
        recorded_checksum = row[CHECKSUM_FIELD]

        if calculated_checksum == recorded_checksum:
            result = "MATCH"
            matches += 1
        else:
            result = f"MISMATCH (Recorded: {recorded_checksum}, Calc: {calculated_checksum})"
            mismatches += 1

        # Debug output
        print(f"Row {idx}: {result}")

        # Store recalculated values in the dataframe
        df.at[idx, 'RECALCULATED_CHECKSUM'] = calculated_checksum
        df.at[idx, 'MATCH'] = (calculated_checksum == recorded_checksum)

    # Summary
    print(f"\nTotal rows: {len(df)}")
    print(f"Matching checksums: {matches}")
    print(f"Mismatched checksums: {mismatches}")

    # Save updated CSV if requested
    if save_file:
        df.to_csv(save_file, index=False)
        print(f"\nUpdated CSV saved to: {save_file}")

    return df


if __name__ == "__main__":
    # Example usage
    updated_df = recalculate_and_compare_checksums("3f2.csv", save_file="3f2_compared.csv")
