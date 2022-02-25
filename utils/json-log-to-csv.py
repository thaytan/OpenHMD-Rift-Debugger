#!/usr/bin/env python
import csv, json, sys

if len(sys.argv) < 4:
    print("Usage {} in.json out.csv row-type".format(sys.argv[0]))
    sys.exit(1)

infile = open(sys.argv[1])
outfile = open(sys.argv[2], 'w')
first_row = True

output = csv.writer(outfile)
 
for line in infile:
    row = json.loads(line)

    if row['type'] != sys.argv[3]:
        continue
    if first_row:
        output.writerow(row.keys())  # header row
        first_row = False
    output.writerow(row.values())

infile.close()
