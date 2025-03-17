import argparse
import pandas as pd
from opendbc.can.packer import CANPacker
from opendbc.car.psa.psacan import calculate_checksum

"""
python checksum.py --mode 3f2 --input input.csv --output output.csv
"""

def recalculate_and_compare_checksums(csv_file: str, save_file: str, mode: str):
    if mode == '3f2':
        dbc_name = 'AEE2010_R3'
        msg_name = 'LANE_KEEP_ASSIST'
        checksum_field = 'CHECKSUM'
        checksum_init = 0xB
        bus = 2

        def extract_values(row):
            return {
                'DRIVE': row['DRIVE'],
                'COUNTER': row['COUNTER'],
                checksum_field: 0,
                'STATUS': row['STATUS'],
                'LXA_ACTIVATION': row['LXA_ACTIVATION'],
                'TORQUE_FACTOR': row['TORQUE_FACTOR'],
                'SET_ANGLE': row['SET_ANGLE'],
                'unknown2': row.get('unknown2', 0),
                'unknown4': row.get('unknown4', 0),
            }

    elif mode == '2f6':
        dbc_name = 'AEE2010_R3'
        msg_name = 'HS2_DYN_MDD_ETAT_2F6'
        checksum_field = 'CHECKSUM_TRANSM_DYN_ACC2'
        checksum_init = 0x8
        bus = 1

        def extract_values(row):
            return {
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
                checksum_field: 0,
            }

    elif mode == '2b6':
        dbc_name = 'AEE2010_R3'
        msg_name = 'HS2_DYN1_MDD_ETAT_2B6'
        checksum_field = 'DYN_ACC_CHECKSUM'
        checksum_init = 0xC
        bus = 1

        def extract_values(row):
            return {
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
                checksum_field: 0,
            }

    else:
        raise ValueError(f"Unknown mode: {mode}")

    # Load CSV
    df = pd.read_csv(csv_file)
    packer = CANPacker(dbc_name)

    matches, mismatches = 0, 0

    for idx, row in df.iterrows():
        values = extract_values(row)

        msg_bytes = packer.make_can_msg(msg_name, bus, values)[1]
        calculated_checksum = calculate_checksum(msg_bytes, checksum_init)
        recorded_checksum = row[checksum_field]

        if calculated_checksum == recorded_checksum:
            result = "MATCH"
            matches += 1
        else:
            result = f"MISMATCH (Recorded: {recorded_checksum}, Calc: {calculated_checksum})"
            mismatches += 1

        print(f"Row {idx}: {result}")

        df.at[idx, 'RECALCULATED_CHECKSUM'] = calculated_checksum
        df.at[idx, 'MATCH'] = (calculated_checksum == recorded_checksum)

    # Summary
    print(f"\nMode: {mode}")
    print(f"Total rows: {len(df)}")
    print(f"Matching checksums: {matches}")
    print(f"Mismatched checksums: {mismatches}")

    if save_file:
        df.to_csv(save_file, index=False)
        print(f"\nUpdated CSV saved to: {save_file}")

    return df


def main():
    parser = argparse.ArgumentParser(description="Recalculate and compare CAN message checksums from CSV data.")
    parser.add_argument('--mode', choices=['3f2', '2f6', '2b6'], required=True, help="Select the message type to process.")
    parser.add_argument('--input', required=True, help="Input CSV file path.")
    parser.add_argument('--output', required=False, help="Output CSV file path (optional).")

    args = parser.parse_args()

    recalculate_and_compare_checksums(args.input, args.output, args.mode)


if __name__ == "__main__":
    main()
