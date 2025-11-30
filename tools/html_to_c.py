#!/usr/bin/env python3
"""
HTML to C Header Converter

Converts an HTML file into a C header file with the HTML embedded as a string constant.
"""

import sys
import os

def html_to_c_header(html_file, output_file):
    """
    Convert HTML file to C header file

    Args:
        html_file: Path to input HTML file
        output_file: Path to output C header file
    """
    # Read HTML file
    try:
        with open(html_file, 'r', encoding='utf-8') as f:
            html_content = f.read()
    except FileNotFoundError:
        print(f"Error: Input file '{html_file}' not found")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading input file: {e}")
        sys.exit(1)

    # Escape special characters for C string
    # Replace backslashes first to avoid double-escaping
    escaped = html_content.replace('\\', '\\\\')
    escaped = escaped.replace('"', '\\"')
    escaped = escaped.replace('\n', '\\n')
    escaped = escaped.replace('\r', '')  # Remove carriage returns

    # Generate C header content
    header_guard = "WEB_INDEX_HTML_H"
    header_content = f"""/*
 * Auto-generated file from {os.path.basename(html_file)}
 * DO NOT EDIT MANUALLY
 */

#ifndef {header_guard}
#define {header_guard}

static const char web_index_html[] =
"{escaped}";

#endif // {header_guard}
"""

    # Write output file
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(header_content)
        print(f"Successfully generated: {output_file}")
        print(f"  - Input size: {len(html_content)} bytes")
        print(f"  - Output size: {len(header_content)} bytes")
    except Exception as e:
        print(f"Error writing output file: {e}")
        sys.exit(1)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 html_to_c.py <input.html> <output.h>")
        print("Example: python3 html_to_c.py web_index.html web_index_html.h")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    html_to_c_header(input_file, output_file)
