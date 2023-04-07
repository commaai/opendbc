"""Visualize a DBC file and extract signals from CAN messages"""
import argparse
import cantools
import matplotlib.pyplot as plt
import numpy as np
import mplcursors

def visualize_bitfield_8x8(message):
    """Visualize the bit fields of a CAN message in a 8x8 grid"""
    _, ax = plt.subplots()
    ax.set_title(f"Message: {message.name} (ID: {message.frame_id})")
    ax.set_ylim(0, message.length)
    ax.set_xlim(0, 8)
    ax.set_xlabel("Bit position")
    ax.set_ylabel("Byte number")
    ax.set_xticks(range(8))
    ax.set_yticks(range(message.length))
    ax.set_yticklabels(reversed(range(message.length)))
    # add a grid to the plot
    ax.grid(color='k', linestyle='-', linewidth=0.5)

    for tick_label in ax.yaxis.get_majorticklabels():
        tick_label.set_verticalalignment("bottom")

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
            rect = plt.Rectangle((bit, (message.length-1) - byte), 1, 1, color=colors[idx])
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

    # Add click event handler to the figure
    #fig.canvas.mpl_connect('button_press_event',
    #                    lambda event: [print(generate_c_code(message, signal)) for rect,
    #                    signal in signal_rectangles.items() if rect.contains(event)[0]])

    # Add interactivity with mplcursors
    cursor = mplcursors.cursor(ax, hover=True)
    # In the mplcursors event handler
    cursor.connect("add", lambda sel: (sel.annotation.set_text(signal_rectangles[sel.artist])))

    plt.show()


def main():
    """Parse command line arguments and load the DBC file"""
    # Parse command line arguments
    parser =argparse.ArgumentParser(
        description= "Visualize CAN message bit fields from a DBC file.")
    parser.add_argument("dbc_file", help="Path to the DBC file.")
    parser.add_argument("message", help="Message name or frame ID.")
    args = parser.parse_args()

    # Load the DBC file
    db = cantools.database.load_file(args.dbc_file)

    # Check if the provided message is a name or a frame ID
    try:
        frame_id = int(args.message)
        message_to_visualize = db.get_message_by_frame_id(frame_id)
    except ValueError:
        message_to_visualize = db.get_message_by_name(args.message)

    if message_to_visualize:
        visualize_bitfield_8x8(message_to_visualize)
    else:
        print("Message not found in the DBC file.")

if __name__ == "__main__":
    main()
