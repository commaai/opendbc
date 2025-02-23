import jinja2
import itertools
import pprint

from jinja2 import Environment, StrictUndefined, Template


# Each identifier must already exist as a boolean in C.
# Each row is a list of mutually exclusive conditions.
# A default, naked else case is automatically added.
CONDITIONS = [
  ["hyundai_canfd_lka_steering", "hyundai_camera_scc"],
  ["hyundai_ev_gas_signal", "hyundai_hybrid_gas_signal"],
  ["hyundai_canfd_alt_buttons"],
  ["hyundai_longitudinal"],
]

# kwargs: dict[str, bool]
def gen_rx_check(**kwargs):
  return env.from_string("""
{%- set pt_bus = 1 if hyundai_canfd_lka_steering else 0 -%}
{%- set scc_bus = 1 if hyundai_canfd_lka_steering else (2 if hyundai_camera_scc else 0) -%}

{#- RX Common checks. #}
{.msg = { {0x175, ({{pt_bus}}), 24, .check_checksum = true, .max_counter = 0xffU, .frequency = 50U}, { 0 }, { 0 }}},
{.msg = { {0xa0, ({{pt_bus}}), 24, .check_checksum = true, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}},
{.msg = { {0xea, ({{pt_bus}}), 24, .check_checksum = true, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}},

{#- Accel signals. -#}
{%- if hyundai_ev_gas_signal -%}
  {.msg = { {0x35, ({{pt_bus}}), 32, .check_checksum = true, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}},
{%- elif hyundai_hybrid_gas_signal %}
  {.msg = { {0x105, ({{pt_bus}}), 32, .check_checksum = true, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}},
{%- else %}
  {.msg = { {0x100, ({{pt_bus}}), 32, .check_checksum = true, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}},
{%- endif -%}

{#- Cruise signals. -#}
{%- if hyundai_canfd_alt_buttons %}
  {.msg = { {0x1aa, ({{pt_bus}}), 16, .check_checksum = false, .max_counter = 0xffU, .frequency = 50U}, { 0 }, { 0 }}},
{% else %}
  {.msg = { {0x1cf, ({{pt_bus}}), 8, .check_checksum = false, .max_counter = 0xfU, .frequency = 50U}, { 0 }, { 0 }}},
{% endif -%}

{%- if hyundai_longitudinal %}
  {#- SCC_CONTROL sent, not read. -#}
{% else %}
  {#- // SCC_CONTROL read. -#}
  {.msg = { {0x1a0, ({{scc_bus}}), 32, .check_checksum = true, .max_counter = 0xffU, .frequency = 50U}, { 0 }, { 0 }}},
{% endif -%}
""").render(**kwargs)


# Require every param to be defined.
env = Environment(undefined=StrictUndefined)


def generate_code(condition_dict):
    # This function would be implemented externally
    # It takes a dict of boolean names and their values
    # and returns the appropriate code string
    condition_str = ",\n".join([f"// {k}: {v}" for k, v in condition_dict.items()])
    return f"// Case:\n{condition_str}\n" + gen_rx_check(**condition_dict)

template = Template('''\
{%- macro generate_conditions(levels, current_conditions) -%}
{%- if levels -%}
{%- set current_level = levels[0] -%}
{%- set remaining_levels = levels[1:] -%}
{%- for condition in current_level -%}
{%- set new_conditions = current_conditions.copy() -%}
{%- set _ = new_conditions.update({condition: True}) -%}
{%- for other in current_level -%}
    {%- if other != condition -%}
        {%- set _ = new_conditions.update({other: False}) -%}
    {%- endif -%}
{%- endfor %}
if ({{ condition }}) {
{{ generate_conditions(remaining_levels, new_conditions) | indent(2) }}
}
{%- endfor %}
else {
{%- set new_conditions = current_conditions.copy() -%}
{%- for condition in current_level -%}
    {%- set _ = new_conditions.update({condition: False}) -%}
{%- endfor -%}
{{ generate_conditions(remaining_levels, new_conditions) | indent(2) }}
}
{%- else %}
{{ generate_code(current_conditions) }}
{%- endif -%}
{%- endmacro -%}

void generated_function() {
{{ generate_conditions(condition_groups, {}) | indent(2) }}
}
''')


output = template.render(condition_groups=CONDITIONS, generate_code=generate_code)
print(output)
print(len(output.split("\n")))




