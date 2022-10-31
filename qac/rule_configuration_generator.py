#!/usr/bin/env python3

import csv
import os
import sys
import xml.etree.ElementTree

#
# Script performs a basic XML file modification to the Helix QAC's ASCM Rule
# Configuration File (RCF).
#
# Simply run this script without any other arguments, it will open and read
# through the 'rule_configuration_deviation.csv' located on the same folder
# as this script as well as the ASCM RCF expected to be located in:
#
#   path: /opt/Perforce/Helix-QAC-2020.1/config/rcf/crit_ascm.rcf
#
# It will generate a modified version of the RCF file and output via the
# stdout. The output can be streamed out to a file and imported into Helix QAC
# with the compliance deviations expressed in the CSV file.
#
# NOTE: due to xml.etree.ElementTree class only recently introducing attribute
# order preservation, this script is best executed with python version 3.8+,
# otherwise there would be too many differences between the original ASCM RFC
# and the output XML file.
#

if __name__ != '__main__':
    print('error: script must be run as the startup script', file=sys.stderr)
    exit(1)

if sys.version_info.major <= 3 and sys.version_info.minor < 8:
    print('warning: python version 3.8+ is required to produce similar formatted RCF files as to the original', file=sys.stderr)

try:
    rcf_in = xml.etree.ElementTree.parse('/opt/Perforce/Helix-QAC-2020.1/config/rcf/crit_ascm.rcf')
except xml.etree.ParseError as err:
    print('error: parsing error raised', file=sys.stderr)
    print('message: {}'.format(err), file=sys.stderr)
    exit(1)

with open(os.path.join(os.path.dirname(__file__), 'rule_configuration_deviation.csv')) as csv_file:
    csv_reader = csv.DictReader(csv_file)

    for csv_row in csv_reader:
        xpath = csv_row['XPath']
        enforced = csv_row['Enforced']

        nodes = list(rcf_in.getroot().findall('.{}'.format(xpath)))  # we need to add a dot before the xpath

        if len(nodes) == 0:
            print('error: no matches identified for xpath "{}"'.format(xpath), file=sys.stderr)
            exit(1)
        elif len(nodes) > 1:
            print('error: multiple matches identified for xpath "{}"'.format(xpath), file=sys.stderr)
            exit(1)

        node = nodes[0]

        if enforced != 'no' and enforced != 'yes':
            print('error: xpath "{}" has unsupported enforced value "{}"'.format(xpath, enforced), file=sys.stderr)
            print('message: only "yes" and "no" enforced values are supported', file=sys.stderr)
            exit(1)

        node.attrib['mapped'] = enforced

rcf_out = xml.etree.ElementTree.tostring(rcf_in.getroot()).decode()
rcf_out = rcf_out.replace(' />', '/>')  # python library adds an extra space before the > to comply with standard
rcf_out = rcf_out.replace('&gt;', '>')  # *.rcf file format does not properly encode its > value
rcf_out = '<?xml version="1.0" encoding="UTF-8"?>\n{}'.format(rcf_out)  # need to manually add in xml declaration

print(rcf_out)
