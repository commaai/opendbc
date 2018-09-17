#!/usr/bin/env python2
import os
import re
from collections import namedtuple

cur_path = os.path.dirname(os.path.realpath(__file__))
generator_path = os.path.join(cur_path, '../')

# match `// IMPORT filename.dbc`, `CM_ "IMPORT filename.dbc"`, or `#include "filename.dbc"`
include_pattern = re.compile(r'^(?:// IMPORT|CM_ \"IMPORT|#include)\s+\"?([^\"]+)\"?\s*$')
blank_line_pattern = re.compile(r'^\s*$')

# The .DBC file is separated into sections, and the sections are expected
# to be encountered in a specific order.
DbcSection = namedtuple('DbcSection', ['order', 'name', 'symbol', 'pattern'])
DbcStructure = [
    DbcSection(0, 'version', 'VERSION', re.compile(r'^\s*VERSION.*$')),  # only one allowed
    DbcSection(1, 'new_symbols', 'NS_', re.compile(r'^\s*NS_\s*:.*$')),  # only one allowed
    DbcSection(2, 'bit_timing', 'BS_', re.compile(r'^\s*BS_\s*:.*$')),  #  obsolete but required, only one allowed
    DbcSection(3, 'nodes', 'BU_', re.compile(r'^\s*BU_\s*:.*$')),  # only one allowed
    DbcSection(4, 'value_tables', 'VAL_TABLE_', re.compile(r'^\s*VAL_TABLE_\s+.*$')),
    DbcSection(5, 'messages', 'BO_', re.compile(r'^\s*BO_\s+.*$')),
    DbcSection(6, 'message_transmitters', 'BO_TX_BU_', re.compile(r'^\s*BO_TX_BU_\s+.*$')),  # not used to define CAN layer-2 communication
    DbcSection(7, 'environment_variables', 'EV_', re.compile(r'^\s*EV_\s+.*$')),
    DbcSection(8, 'environment_variables_data', 'ENVVAR_DATA_', re.compile(r'^\s*ENVVAR_DATA_\s+.*$')),
    DbcSection(9, 'signal_types', 'SGTYPE_', re.compile(r'^\s*SGTYPE_\s+\S+\s*:.*$')),  # not normally used
    DbcSection(10, 'comments', 'CM_', re.compile(r'^\s*CM_\s+.*$')),
    DbcSection(11, 'attribute_definitions', 'BA_DEF_', re.compile(r'^\s*BA_DEF_\s+.*$')),
    DbcSection(12, 'sigtype_attr_list', 'BA_SGTYPE_', re.compile(r'^\s*BA_SGTYPE_.*$')),  # obsolete
    DbcSection(13, 'attribute_defaults', 'BA_DEF_DEF_', re.compile(r'^\s*BA_DEF_DEF_\s+.*$')),
    DbcSection(14, 'attribute_values', 'BA_', re.compile(r'^\s*BA_\s+.*$')),
    DbcSection(15, 'value_descriptions', 'VAL_', re.compile(r'^\s*VAL_\s+.*$')),
    DbcSection(16, 'category_definitions', 'CAT_DEF_', re.compile(r'^\s*CAT_DEF_.*$')),  # obsolete
    DbcSection(17, 'categories', 'CAT_', re.compile(r'^\s*CAT_.*$')),  # obsolete
    DbcSection(18, 'filter', 'FILTER', re.compile(r'^\s*FILTER.*$')),  # obsolete
    DbcSection(19, 'signal_type_refs', 'SGTYPE_', re.compile(r'^\s*SGTYPE_\s+\S+\s+\S+\s*:.*$')),  # not normally used
    DbcSection(20, 'signal_groups', 'SIG_GROUP_', re.compile(r'^\s*SIG_GROUP_\s+.*$')),
    DbcSection(21, 'signal_extended_value_type_list', 'SIG_VALTYPE_', re.compile(r'^\s*SIG_VALTYPE_\s+.*$')),  # only with 'float' and 'double'
]


def set_defaults(combined_contents):
    combined_contents.setdefault(0, ['VERSION ""', ''])
    combined_contents.setdefault(1, ['NS_ : ',
                                     '\tNS_DESC_',
                                     '\tCM_',
                                     '\tBA_DEF_',
                                     '\tBA_',
                                     '\tVAL_',
                                     '\tCAT_DEF_',
                                     '\tCAT_',
                                     '\tFILTER',
                                     '\tBA_DEF_DEF_',
                                     '\tEV_DATA_',
                                     '\tENVVAR_DATA_',
                                     '\tSGTYPE_',
                                     '\tSGTYPE_VAL_',
                                     '\tBA_DEF_SGTYPE_',
                                     '\tBA_SGTYPE_',
                                     '\tSIG_TYPE_REF_',
                                     '\tVAL_TABLE_',
                                     '\tSIG_GROUP_',
                                     '\tSIG_VALTYPE_',
                                     '\tSIGTYPE_VALTYPE_',
                                     '\tBO_TX_BU_',
                                     '\tBA_DEF_REL_',
                                     '\tBA_REL_',
                                     '\tBA_DEF_DEF_REL_',
                                     '\tBU_SG_REL_',
                                     '\tBU_EV_REL_',
                                     '\tBU_BO_REL_',
                                     '\tSG_MUL_VAL_',
                                     '',
                                     ])
    combined_contents.setdefault(2, ['BS_:', ''])


def read_dbc(dir_name, filename, combined_contents):
    with open(os.path.join(dir_name, filename)) as file_in:
        lines = [l.rstrip('\r\n') for l in file_in.readlines()]

    section_index = 0
    expecting_index = None

    # read file contents in expected section order.
    # if an include pattern is encountered, then the file will be queued for reading in the next pass.
    for line_index, line in enumerate(lines):
        include_match = include_pattern.match(line)
        if include_match:
            match_filename = include_match.group(1)
            if not match_filename.lower().endswith(".dbc"):
                formatted_error = "ERROR: {0}:{1} - Can only include .dbc files, instead encountered \"{2}\".".format(
                    filename, line_index + 1, match_filename)
                raise ValueError(formatted_error)

            read_dbc(dir_name, match_filename, combined_contents)

        else:
            if section_index == 1:
                # only BS_ is allowed after NS_, assume end of NS_ section on blank line
                if blank_line_pattern.match(line) or DbcStructure[2].pattern.match(line):
                    expecting_index = 2

            if section_index != 1 or expecting_index is not None:
                for dbc_section in DbcStructure:
                    match_section = dbc_section.pattern.match(line)

                    if match_section:
                        if expecting_index is not None and dbc_section.order != expecting_index:
                            formatted_error = "ERROR {0}:{1} - BS_ section needed after NS_ section".format(filename,
                                                                                                            line_index + 1)
                            raise ValueError(formatted_error)
                        else:
                            expecting_index = None

                        if dbc_section.order < section_index:
                            formatted_error = "ERROR {0}:{1} - Section {2} out of order".format(filename, line_index + 1,
                                                                                              dbc_section.symbol)
                            raise ValueError(formatted_error)
                        section_index = dbc_section.order
                        break

            section_lines = combined_contents.setdefault(section_index, [])
            section_lines.append(line)


def create_dbc(dir_name, filename):
    combined_contents = dict()

    read_dbc(dir_name, filename, combined_contents)

    # set defaults in case they weren't set
    set_defaults(combined_contents)

    output_filename = filename.replace('.dbc', '_generated.dbc')
    output_file_location = os.path.join(generator_path, output_filename)

    with open(output_file_location, 'w') as dbc_file_out:
        dbc_file_out.write('// AUTOGENERATED FILE, DO NOT EDIT\n')

        for index, lines in sorted(combined_contents.items()):
            for line in lines:
                dbc_file_out.write(line)
                dbc_file_out.write('\n')


for dir_name, _, filenames in os.walk(cur_path):
    if dir_name == cur_path or not (dir_name.endswith("toyota") or dir_name.endswith("honda")):
        continue

    print dir_name
    for filename in filenames:
        if filename.startswith('_') or not filename.lower().endswith('.dbc'):
            continue

        print filename

        try:
            create_dbc(dir_name, filename)
        except ValueError as ex:
            print(ex)
