import csv
import sys
import pathlib
from pathlib import Path

NUM_CORES = 32

folder = Path(sys.argv[1])

exp_config = folder.stem
rt_stats_csv = folder / 'RTStats.csv'
packet_stats_csv = folder / 'PacketStats.csv'
usage_stats_csv = folder / 'UsageStats.csv'

stats_csv = Path("./Stats.csv")

with rt_stats_csv.open('r') as f:
    reader = csv.DictReader(f)
    RTfactor = []
    for row in reader:
        RTfactor.append(float(row['# real-time factor (percent)'].strip()))

with packet_stats_csv.open('r') as f:
    reader = csv.DictReader(f)
    packets = {}
    time = 0
    for row in reader:
        if not row['udp.dstport']:
            continue
        dstport = int(row['udp.dstport'])
        packet_len = int(row['udp.length'])
        if dstport not in packets:  # Init for this dstport
            packets[dstport] = {'num': 0, 'len': 0}

        packets[dstport]['num'] += 1
        packets[dstport]['len'] += packet_len
        time = float(row['frame.time_relative'])

cpu = 'cpu_percent'
threads = 'num_threads'
mem = 'mem_percent'
nodes = {}
with usage_stats_csv.open('r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        node_name = row['node']
        if node_name not in nodes:
            nodes[node_name] = []
        nodes[node_name].append({
            cpu: float(row[cpu]),
            threads: int(row[threads]),
            mem: float(row[mem])
        })

usage_summary = {
    cpu: {'min': 0.0, 'avg': 0.0, 'max': 0.0},
    threads: {'min': 0.0, 'avg': 0.0, 'max': 0.0},
    mem: {'min': 0.0, 'avg': 0.0, 'max': 0.0},
}
for node_name in nodes:
    cpu_list = [entry[cpu] for entry in nodes[node_name]]
    threads_list = [entry[threads] for entry in nodes[node_name]]
    mem_list = [entry[mem] for entry in nodes[node_name]]

    usage_summary[cpu]['min'] += min(cpu_list)
    usage_summary[cpu]['avg'] += sum(cpu_list) / len(cpu_list)
    usage_summary[cpu]['max'] += max(cpu_list)
    usage_summary[threads]['min'] += min(threads_list)
    usage_summary[threads]['avg'] += sum(threads_list) / len(threads_list)
    usage_summary[threads]['max'] += max(threads_list)
    usage_summary[mem]['min'] += min(mem_list)
    usage_summary[mem]['avg'] += sum(mem_list) / len(mem_list)
    usage_summary[mem]['max'] += max(mem_list)

write_header = not stats_csv.exists()
with stats_csv.open('a') as csvfile:
    fieldnames = ['Configuration',
                  'Min RT Factor', 'Avg RT Factor', 'Max RT Factor',
                  'Total #packets per agent', '#packets per agent per second',
                  'Total packet length per agent', 'Avg packet length per agent per second',
                  'Min %CPU', 'Avg %CPU', 'Max %CPU',
                  'Min %MEM', 'Avg %MEM', 'Max %MEM',
                  'Min #threads', 'Avg #threads', 'Max #threads',
    ]
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    if write_header:
        writer.writeheader()

    sum_packet = sum([p['num'] for p in packets.values()]) / len(packets)
    avg_packet = sum_packet / time
    sum_packet_len = sum([p['len'] for p in packets.values()]) / len(packets)
    avg_packet_len = sum_packet_len / time

    writer.writerow({
        'Configuration': exp_config,
        'Min RT Factor': min(RTfactor),
        'Avg RT Factor': sum(RTfactor) / len(RTfactor),
        'Max RT Factor': max(RTfactor),
        'Total #packets per agent': sum_packet,
        '#packets per agent per second': avg_packet,
        'Total packet length per agent': sum_packet_len,
        'Avg packet length per agent per second': avg_packet_len,
        'Min %CPU': usage_summary[cpu]['min'] / NUM_CORES,
        'Avg %CPU': usage_summary[cpu]['avg'] / NUM_CORES,
        'Max %CPU': usage_summary[cpu]['max'] / NUM_CORES,
        'Min %MEM': usage_summary[mem]['min'],
        'Avg %MEM': usage_summary[mem]['avg'],
        'Max %MEM': usage_summary[mem]['max'],
        'Min #threads': usage_summary[threads]['min'],
        'Avg #threads': usage_summary[threads]['avg'],
        'Max #threads': usage_summary[threads]['max'],
    })
