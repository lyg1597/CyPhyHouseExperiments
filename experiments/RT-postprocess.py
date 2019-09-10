import csv

file_name = raw_input("Which file do you wish to process?\n")

with open(file_name, 'r') as in_file:
    stripped = (line.strip() for line in in_file)
    lines = (line.split(" ") for line in stripped if line)
    with open('RTstats.csv', 'w') as out_file:
        writer = csv.writer(out_file)
        writer.writerow(('Factor', 'Sim', 'Real'))
        writer.writerows(lines)

with open('RTstats.csv', 'r') as my_csv:
    reader = csv.DictReader(my_csv)
    RTfactor = []
    for row in reader:
        RTfactor.append(float(row['Factor'].strip('Factor[]')))

print "Maximum Realtime Factor is ", max(RTfactor)
print "Minimum Realtime Factor is ", min(RTfactor)
print "Mean Realtime Factor is ", sum(RTfactor)/len(RTfactor)