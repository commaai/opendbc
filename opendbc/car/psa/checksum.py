import pandas as pd
from opendbc.can.packer import CANPacker
from opendbc.car.psa.psacan import calculate_checksum

# PSA DBC for the radar messages on bus 1
DBC_NAME = 'AEE2010_R3'
MSG_NAME = 'HS2_DYN_MDD_ETAT_2F6'
CHECKSUM_FIELD = 'CHECKSUM_TRANSM_DYN_ACC2'
CHECKSUM_INIT = 0x8
BUS = 1


def recalculate_and_compare_checksums(csv_file: str, save_file: str = None):
    # Load the CSV data
    df = pd.read_csv(csv_file)

    # Init CANPacker for PSA ADAS
    packer = CANPacker(DBC_NAME)

    # Statistics
    matches = 0
    mismatches = 0

    for idx, row in df.iterrows():
        # Build the dictionary of DBC values from the row
        values = {
            'TARGET_DETECTED': row['TARGET_DETECTED'],
            'REQUEST_TAKEOVER': row['REQUEST_TAKEOVER'],
            'BLIND_SENSOR': row['BLIND_SENSOR'],
            'REQ_VISUAL_COLL_ALERT_ARC': row['REQ_VISUAL_COLL_ALERT_ARC'],
            'REQ_AUDIO_COLL_ALERT_ARC': row['REQ_AUDIO_COLL_ALERT_ARC'],
            'REQ_HAPTIC_COLL_ALERT_ARC': row['REQ_HAPTIC_COLL_ALERT_ARC'],
            'INTER_VEHICLE_DISTANCE': row['INTER_VEHICLE_DISTANCE'],
            'ARC_STATUS': row['ARC_STATUS'],
            'AUTO_BRAKING_IN_PROGRESS': row['AUTO_BRAKING_IN_PROGRESS'],
            'AEB_ENABLED': row['AEB_ENABLED'],
            'DRIVE_AWAY_REQUEST': row['DRIVE_AWAY_REQUEST'],
            'DISPLAY_INTERVEHICLE_TIME': row['DISPLAY_INTERVEHICLE_TIME'],
            'MDD_DECEL_CONTROL_REQ': row['MDD_DECEL_CONTROL_REQ'],
            'AUTO_BRAKING_STATUS': row['AUTO_BRAKING_STATUS'],
            'PROCESS_COUNTER_4B_ACC2': row['PROCESS_COUNTER_4B_ACC2'],
            'TARGET_POSITION': row['TARGET_POSITION'],
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
    updated_df = recalculate_and_compare_checksums("2f6.csv", save_file="2f6_compared.csv")
