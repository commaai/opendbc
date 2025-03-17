import pandas as pd
from opendbc.can.packer import CANPacker
from opendbc.car.psa.psacan import calculate_checksum

# PSA DBC name for your dataset
DBC_NAME = 'AEE2010_R3'
MSG_NAME = 'HS2_DYN1_MDD_ETAT_2B6'
CHECKSUM_FIELD = 'DYN_ACC_CHECKSUM'
CHECKSUM_INIT = 0xC
BUS = 1


def recalculate_and_compare_checksums(csv_file: str, save_file: str = None):
    # Load the CSV data
    df = pd.read_csv(csv_file)

    # Init CANPacker for PSA ADAS (or your specific DBC)
    packer = CANPacker(DBC_NAME)

    # Statistics
    matches = 0
    mismatches = 0

    for idx, row in df.iterrows():
        # Build the dictionary of DBC values from the row
        values = {
            'MDD_DESIRED_DECELERATION': row['MDD_DESIRED_DECELERATION'],
            'POTENTIAL_WHEEL_TORQUE_REQUEST': row['POTENTIAL_WHEEL_TORQUE_REQUEST'],
            'MIN_TIME_FOR_DESIRED_GEAR': row['MIN_TIME_FOR_DESIRED_GEAR'],
            'GMP_POTENTIAL_WHEEL_TORQUE': row['GMP_POTENTIAL_WHEEL_TORQUE'],
            'ACC_STATUS': row['ACC_STATUS'],
            'GMP_WHEEL_TORQUE': row['GMP_WHEEL_TORQUE'],
            'WHEEL_TORQUE_REQUEST': row['WHEEL_TORQUE_REQUEST'],
            'AUTO_BRAKING_STATUS': row['AUTO_BRAKING_STATUS'],
            'MDD_DECEL_TYPE': row['MDD_DECEL_TYPE'],
            'MDD_DECEL_CONTROL_REQ': row['MDD_DECEL_CONTROL_REQ'],
            'GEAR_TYPE': row['GEAR_TYPE'],
            'PREFILL_REQUEST': row['PREFILL_REQUEST'],
            'DYN_ACC_PROCESS_COUNTER': row['DYN_ACC_PROCESS_COUNTER'],
            # Important: zero out checksum before packing
            CHECKSUM_FIELD: 0,
        }

        # Pack the message to get raw bytes
        msg_bytes = packer.make_can_msg(MSG_NAME, BUS, values)[1]

        # Calculate the checksum
        calculated_checksum = calculate_checksum(msg_bytes, CHECKSUM_INIT)

        # Compare with the recorded checksum from CSV
        recorded_checksum = row[CHECKSUM_FIELD]

        if calculated_checksum == recorded_checksum:
            result = "MATCH"
            matches += 1
        else:
            result = f"MISMATCH (Recorded: {recorded_checksum}, Calc: {calculated_checksum})"
            mismatches += 1

        # Print result for each row
        print(f"Row {idx}: {result}")

        # Optional: store the recalculated checksum in the DataFrame (for further analysis or saving)
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
    updated_df = recalculate_and_compare_checksums("2b6.csv", save_file="2b6_compared.csv")
