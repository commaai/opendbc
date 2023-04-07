import argparse
import cantools
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import mplcursors

def generate_c_code(message, signal):
    signal = message.get_signal_by_name(signal)
    if signal is None:
        print(f"Error: signal '{signal}' not found in message '{message.name}'")
        return None
    start_bit = signal.start
    length = signal.length
    scale, offset = signal.scale, signal.offset
    byte_order = signal.byte_order

    if byte_order == "little_endian":
        byte_indices = [start_bit // 8 + i for i in range((length + 7) // 8)]
    else:
        byte_indices = [start_bit // 8 - i for i in range((length + 7) // 8)]

    mask = (1 << length) - 1

    c_code = f"uint{8 * len(byte_indices)}_t raw_value = ("
    for i, byte_index in enumerate(byte_indices):
        if i > 0:
            if byte_order == "little_endian":
                c_code += " << 8) | "
            else:
                c_code += ") << 8 | "

        if byte_order == "little_endian":
            c_code += f"GET_BYTE(to_send, {byte_index})"
        else:
            if byte_index < 0:
                byte_index = len(byte_indices) + byte_index
            c_code += f"GET_BYTE(to_send, {byte_index})"

        if byte_order == "little_endian" and i < len(byte_indices) - 1:
            bit_shift = start_bit % 8
            c_code += f" & 0x{mask >> (8 * i):02X}U"
            if bit_shift > 0:
                c_code += f") >> {bit_shift}"
        elif byte_order == "big_endian" and i == 0:
            bit_shift = 8 - (start_bit+1 % 8) - length % 8
            if bit_shift > 0:
                c_code += f" >> {bit_shift}"

    c_code += ");\n"

    if scale != 1 or offset != 0:
        c_code += f"int {signal.name}_value = (int)((raw_value & 0x{mask:04X}U)"
        if scale != 1:
            c_code += f" * {scale}"
        if offset != 0:
            if scale != 1:
                c_code += " +"
            c_code += f" {offset}"
        c_code += ");\n"
    else:
        c_code += f"int {signal.name}_value = (int)(raw_value & 0x{mask:04X}U);\n"

    return c_code

def visualize_bitfield_8x8(db, message):
    fig, ax = plt.subplots()
    ax.set_title(f"Message: {message.name} (ID: {message.frame_id})")
    ax.set_ylim(0, 8)
    ax.set_xlim(0, 8)
    ax.set_xlabel("Bit position")
    ax.set_ylabel("Byte number")
    ax.set_xticks(range(8))
    ax.set_yticks(range(8))
    ax.set_yticklabels(reversed(range(8)))
    # add a grid to the plot
    ax.grid(color='k', linestyle='-', linewidth=0.5)

    for tick_label in ax.yaxis.get_majorticklabels():
        tick_label.set_verticalalignment("center")

    colormap = plt.cm.viridis
    colors = [colormap(i) for i in np.linspace(0, 1, len(message.signals))]

    signal_rectangles = {}
    for idx, signal in enumerate(message.signals):
        remaining_length = signal.length
        current_bit = signal.start

        if signal.byte_order == 'little_endian':
            bit_step = 1
        else:
            bit_step = -1

        while remaining_length > 0:
            byte = current_bit // 8
            bit = current_bit % 8
            rect = plt.Rectangle((bit, 7 - byte), 1, 1, color=colors[idx])
            ax.add_patch(rect)
            signal_rectangles[rect] = signal.name

            remaining_length -= 1
            current_bit += bit_step

            if signal.byte_order == 'little_endian' and current_bit % 8 == 0:
                current_bit += 8 - (bit_step * 8)
            elif signal.byte_order != 'little_endian' and current_bit % 8 == 7:
                current_bit += 8 - (bit_step * 8)

    legend_elements = [plt.Line2D([0], [0], color=colors[i], lw=6, label=message.signals[i].name)
                       for i in range(len(message.signals))]
    ax.legend(handles=legend_elements, bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)

    plt.tight_layout(rect=[0, 0, 0.8, 1])
    
       # Define function to generate C code on click events
    def on_click(event):
        if event.button == 1:  # Left button clicked
            for patch in ax.patches:
                if patch.contains(event)[0]:
                    #signal = signal_rectangles[patch]
                    c_code = generate_c_code(message, signal)
                    print(c_code)

    # Add click event handler to the figure
    fig.canvas.mpl_connect('button_press_event', 
                        lambda event: [print(generate_c_code(message, signal)) for rect, signal in signal_rectangles.items() if rect.contains(event)[0]])

    
    # Add interactivity with mplcursors
    cursor = mplcursors.cursor(ax, hover=True)
    # In the mplcursors event handler
    cursor.connect("add", lambda sel: (sel.annotation.set_text(signal_rectangles[sel.artist])))
    # Add selection callback for click events
    
    plt.show()


def main():
    debug = False
    if not debug:
        # Parse command line arguments
        parser = argparse.ArgumentParser(description="Visualize CAN message bit fields from a DBC file.")
        parser.add_argument("dbc_file", help="Path to the DBC file.")
        parser.add_argument("message", help="Message name or frame ID.")
        args = parser.parse_args()
    else:
        args = argparse.Namespace()
        args.dbc_file = "mazda_2017.dbc"
        args.message = "514"
        
    # Load the DBC file
    db = cantools.database.load_file(args.dbc_file)

    # Check if the provided message is a name or a frame ID
    try:
        frame_id = int(args.message)
        message_to_visualize = db.get_message_by_frame_id(frame_id)
    except ValueError:
        message_to_visualize = db.get_message_by_name(args.message)

    if message_to_visualize:
        visualize_bitfield_8x8(db, message_to_visualize)
    else:
        print("Message not found in the DBC file.")

if __name__ == "__main__":
    main()
    
